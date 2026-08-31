#!/usr/bin/env python3
"""摩擦モデルを tanh から Karnopp（真の固着を持つ）に差し替えた検証。

■ なぜやるか

design_and_simulate.py の摩擦は tanh 近似で、
  - 速度ゼロでの**停留（固着）**
  - 静止摩擦 > 動摩擦（Stribeck）による**離脱の跳び**
を表現できていない。「摩擦は大した問題ではない」という結論が
この近似のせいで出ていないかを確かめる。

■ Karnopp モデル

速度が閾値 Δv 未満のとき「固着」とみなす。
このとき摩擦は、関節を止めておくのに必要なトルクをそのまま出す（ただし静止摩擦の上限まで）。
上限を超えたら離脱し、以後は動摩擦になる。

多自由度系なので「止めておくのに必要なトルク」は連成を考慮して解く必要がある。
M·q̈ = r + [τ_f, 0]ᵀ に θ̈=0 を課すと

    行2:  M22·α̈ = r2            → α̈ = r2 / M22
    行1:  M12·α̈ = r1 + τ_f      → τ_f = M12·r2/M22 − r1

これが静止摩擦以下なら固着、超えたら離脱。

■ 実測値（2026/08/12）

    静止摩擦  正転 0.139 / 逆転 0.155 N·m
    動摩擦    正転 0.109 / 逆転 0.143 N·m
    → Stribeck 比は 1.28 / 1.08。正転側で顕著
"""

from __future__ import annotations

import numpy as np

import furuta_model as fm
from design_and_simulate import HW, design_kalman, design_lqr, friction_feedforward

FRIC = dict(
    static_pos=0.139,
    static_neg=0.155,
    kinetic_pos=0.109,
    kinetic_neg=0.143,
    dv=1.0e-3,      # 固着とみなす速度の閾値 [rad/s]
)


def karnopp(x, u, model, fric):
    """Karnopp 型の摩擦トルクと、固着しているかを返す。"""
    v = x[2]
    M, r = model["M_rhs"](x, u)

    if abs(v) < fric["dv"]:
        # 固着候補：θ̈=0 を保つのに必要な摩擦トルク
        tau_hold = M[0, 1] * r[1] / M[1, 1] - r[0]
        tau_s = fric["static_pos"] if tau_hold >= 0 else fric["static_neg"]

        if abs(tau_hold) <= tau_s:
            return tau_hold, True          # 静止摩擦の範囲内 → 動かない

        # 離脱する。動き出す向きは −sign(tau_hold)、摩擦はそれに逆らう
        direction = -np.sign(tau_hold)
        tau_c = fric["kinetic_pos"] if direction > 0 else fric["kinetic_neg"]
        return -tau_c * direction, False

    # すべり中
    tau_c = fric["kinetic_pos"] if v > 0 else fric["kinetic_neg"]
    return -tau_c * np.sign(v), False


def tanh_friction(x, u, model, fric):
    """比較用。design_and_simulate.py と同じ tanh 近似。"""
    v = x[2]
    tau_c = fric["kinetic_pos"] if v >= 0 else fric["kinetic_neg"]
    return -tau_c * np.tanh(v / 0.005), False


def simulate(model, K, L, hw, x0, friction_fn, fric, duration=6.0,
             disturbance=None, seed=0, sub_dt=1e-4):
    """閉ループシミュレーション。摩擦モデルを差し替えられるようにしてある。"""
    rng = np.random.default_rng(seed)
    A, B, f = model["A"], model["B"], model["f"]
    C = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])

    x = np.array(x0, dtype=float)
    xhat = np.zeros(4)
    xhat[:2] = x[:2]

    delay_steps = max(1, int(round(hw["transport_delay"] / sub_dt)))
    pipe = [0.0] * delay_steps
    u_cmd = 0.0
    t = 0.0
    next_control = 0.0
    stuck_time = 0.0

    log = {k: [] for k in ("t", "theta", "alpha", "u", "stuck")}

    while t < duration:
        if t >= next_control:
            y = np.array([
                np.round(x[0] / hw["enc_theta_lsb"]) * hw["enc_theta_lsb"],
                np.round(x[1] / hw["enc_alpha_lsb"]) * hw["enc_alpha_lsb"],
            ])
            dt_ctrl = 1.0 / hw["control_hz"]
            xhat = xhat + dt_ctrl * (A @ xhat + B.reshape(4) * u_cmd + L @ (y - C @ xhat))

            u_cmd = float((-K @ xhat).item()) + friction_feedforward(xhat[2], hw)
            u_cmd = float(np.clip(u_cmd, -hw["torque_max"], hw["torque_max"]))
            u_cmd = np.round(u_cmd / hw["torque_lsb"]) * hw["torque_lsb"]

            period = dt_ctrl + rng.normal(0, hw["jitter_std"])
            if rng.random() < hw["outlier_prob"]:
                period = hw["outlier_s"]
            next_control = t + max(period, 0.2 * dt_ctrl)

            log["t"].append(t)
            log["theta"].append(x[0])
            log["alpha"].append(x[1])
            log["u"].append(u_cmd)

        pipe.append(u_cmd)
        applied = pipe.pop(0)
        if disturbance is not None:
            applied = applied + disturbance(t)

        # 摩擦はステップ開始時に一度だけ決める。
        # RK の途中で固着判定が反転すると数値的な暴れの原因になるため。
        tau_f, stuck = friction_fn(x, applied, model, fric)
        if stuck:
            stuck_time += sub_dt

        k1 = f(x, applied, tau_f)
        k2 = f(x + 0.5 * sub_dt * k1, applied, tau_f)
        x = x + sub_dt * k2

        if stuck:
            x[2] = 0.0          # 固着中は速度をゼロに保つ（数値ドリフトの防止）

        t += sub_dt
        log["stuck"].append(stuck)

        if abs(x[1]) > 0.6:
            return {k: np.array(v) for k, v in log.items()}, False, stuck_time / max(t, 1e-9)

    return {k: np.array(v) for k, v in log.items()}, True, stuck_time / duration


def summarize(name, logd, ok, stuck_frac):
    if not ok:
        return f"  {name:28s} ★転倒★（{logd['t'][-1]:.2f} 秒）"
    tail = np.abs(logd["alpha"][logd["t"] > 4.0])
    theta_span = np.rad2deg(logd["theta"].max() - logd["theta"].min())
    return (f"  {name:28s} 成立  "
            f"リミットサイクル {np.rad2deg(tail.max()):5.2f}°  "
            f"アーム振れ幅 {theta_span:5.1f}°  "
            f"固着時間 {stuck_frac*100:4.1f}%")


def main():
    np.set_printoptions(precision=4, suppress=True)
    print("モデルを導出中...")
    model = fm.build()
    A, B = model["A"], model["B"]
    C = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])

    K, _, _ = design_lqr(A, B, [0.5, 0.05, 3.0, 3.0], 0.5)
    L = design_kalman(A, C, sigma_u=0.05, B=B,
                      meas_lsb=[HW["enc_theta_lsb"], HW["enc_alpha_lsb"]])

    x0 = np.deg2rad([0, 5, 0, 0])

    print("\n" + "=" * 78)
    print("摩擦モデルの比較（初期傾き 5°、摩擦補償 comp_scale=0.9）")
    print("=" * 78)
    for label, fn in (("tanh 近似（従来）", tanh_friction), ("Karnopp（真の固着あり）", karnopp)):
        logd, ok, sf = simulate(model, K, L, HW, x0, fn, FRIC, seed=7)
        print(summarize(label, logd, ok, sf))

    print("\n" + "=" * 78)
    print("摩擦補償の効き × 摩擦モデル")
    print("=" * 78)
    for scale in (0.0, 0.5, 0.7, 0.9, 1.0, 1.1, 1.3):
        hw = dict(HW, comp_scale=scale)
        row = f"  comp_scale={scale:4.1f} |"
        for fn in (tanh_friction, karnopp):
            logd, ok, sf = simulate(model, K, L, hw, x0, fn, FRIC, seed=7)
            if ok:
                tail = np.abs(logd["alpha"][logd["t"] > 4.0])
                row += f"  {np.rad2deg(tail.max()):5.2f}°(固着{sf*100:4.1f}%)"
            else:
                row += f"   ★転倒 {logd['t'][-1]:.1f}s★  "
        print(row + "   ← 左:tanh  右:Karnopp")

    print("\n" + "=" * 78)
    print("Karnopp で各ケースを検証（摩擦補償 0.9）")
    print("=" * 78)

    # 外乱は静止摩擦（0.139〜0.155 N·m）を超えないと関節が動かない。
    # 0.15 N·m では「何も起きない」ので試験にならず、大小2種類を用意する。
    def poke_small(t):
        return 0.15 if 2.0 <= t < 2.03 else 0.0

    def poke_large(t):
        return 0.6 if 2.0 <= t < 2.03 else 0.0

    cases = [
        ("初期傾き 3°", np.deg2rad([0, 3, 0, 0]), None),
        ("初期傾き 6°", np.deg2rad([0, 6, 0, 0]), None),
        ("初期傾き 10°", np.deg2rad([0, 10, 0, 0]), None),
        ("外乱 0.15N·m（摩擦以下）", np.zeros(4), poke_small),
        ("外乱 0.60N·m（摩擦超え）", np.zeros(4), poke_large),
    ]
    for name, xi, dist in cases:
        logd, ok, sf = simulate(model, K, L, HW, xi, karnopp, FRIC,
                                disturbance=dist, seed=7)
        print(summarize(name, logd, ok, sf))

    # 素の QUBE 寸法（アーム 85mm・振子 24g）だと本当に破綻するのかも確かめる。
    # 「設計変更が効いた」という主張の裏付けになる。
    print("\n" + "=" * 78)
    print("参考: 改造しなかった場合（アーム 85mm・振子 24g）")
    print("=" * 78)
    p = dict(fm.PARAMS)
    p.update(L_r=0.085, m_p=0.024, l_p=0.0645, J_p=3.33e-5, J_r=0.0004)
    bare = fm.build(p)
    Kb, _, _ = design_lqr(bare["A"], bare["B"], [0.5, 0.05, 3.0, 3.0], 0.5)
    Lb = design_kalman(bare["A"], C, sigma_u=0.05, B=bare["B"],
                       meas_lsb=[HW["enc_theta_lsb"], HW["enc_alpha_lsb"]])
    # 対照実験：摩擦をゼロにしても転ぶなら、原因は摩擦ではなくゲインやモデルの側にある。
    # これを確かめずに「摩擦のせい」と言うのは根拠が足りない。
    zero_fric = dict(FRIC, static_pos=0.0, static_neg=0.0,
                     kinetic_pos=0.0, kinetic_neg=0.0)
    logd, ok, sf = simulate(bare, Kb, Lb, dict(HW, comp_scale=0.0),
                            np.deg2rad([0, 3, 0, 0]), karnopp, zero_fric, seed=7)
    print(summarize("素の寸法・摩擦ゼロ（対照）", logd, ok, sf))

    for scale in (0.0, 0.9, 1.0):
        hw = dict(HW, comp_scale=scale)
        logd, ok, sf = simulate(bare, Kb, Lb, hw, np.deg2rad([0, 3, 0, 0]),
                                karnopp, FRIC, seed=7)
        print(summarize(f"素の寸法・実測摩擦 comp={scale:.1f}", logd, ok, sf))

    # 摩擦をどこまで下げれば素の寸法でも立つのか
    print("\n  摩擦を実測値の何倍まで下げれば素の寸法で立つか（comp=0.9）:")
    for ratio in (1.0, 0.5, 0.25, 0.1, 0.05):
        scaled = {k: (v * ratio if k != "dv" else v) for k, v in FRIC.items()}
        logd, ok, sf = simulate(bare, Kb, Lb, HW, np.deg2rad([0, 3, 0, 0]),
                                karnopp, scaled, seed=7)
        mark = "成立" if ok else f"転倒({logd['t'][-1]:.2f}s)"
        print(f"    摩擦 ×{ratio:5.2f}（{FRIC['kinetic_pos']*ratio:.3f} N·m）  {mark}")


if __name__ == "__main__":
    main()
