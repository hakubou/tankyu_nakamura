#!/usr/bin/env python3
"""LQR / カルマンゲインの設計と、実機の非理想性を入れた閉ループ検証。

■ ここで立たないものは実機でも絶対に立たない

一見遠回りだが、実機デバッグ時に「制御則が悪いのか実装が悪いのか」を
切り分ける唯一の基準になる。実測した遅延・量子化・摩擦をすべて入れてある。

■ 実機で測った非理想性（2026/08/12）

    制御周期      300 Hz（3.333 ms）、標準偏差 0.563 ms、まれに 22 ms
    往復遅延      約 2.3 ms
    トルク分解能  12bit / ±2.0 N·m → 0.977 mN·m
    トルク上限    ±2.0 N·m
    クーロン摩擦  正転 0.109 / 逆転 0.143 N·m（30% 非対称）
    粘性摩擦      0.027 N·m·s/rad
    モータ角分解能 16bit / ±12.5 rad → 0.382 mrad
    振子角分解能  2048 count/rev → 3.07 mrad

■ 成果物

ゲイン K, L を C# の定数として出力する。実行時に Python は呼ばない。
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import solve_continuous_are

import furuta_model as fm

rng = np.random.default_rng(12345)

# ===========================================================================
# 実機の非理想性
# ===========================================================================
HW = dict(
    control_hz=300.0,
    jitter_std=0.563e-3,       # 制御周期の標準偏差 [s]
    outlier_prob=1 / 3000,     # まれな長周期の発生確率（12秒に1回程度）
    outlier_s=22e-3,
    transport_delay=2.3e-3,    # USB-CAN の往復遅延 [s]
    torque_max=2.0,
    torque_lsb=4.0 / 4095,     # ±2.0 N·m を 12bit
    enc_theta_lsb=25.0 / 65535,     # ±12.5 rad を 16bit
    enc_alpha_lsb=2 * np.pi / 2048,  # 512CPR × 4逓倍
    # 真のクーロン摩擦（非対称）
    tau_c_pos=0.109,
    tau_c_neg=0.143,
    eps_true=0.005,            # 真の摩擦の滑らかさ [rad/s]（実際はもっと急峻）
    # 制御側の摩擦補償
    comp_scale=0.9,            # 補償の効き（1.0 = 完全に既知）。過補償を避けて少し弱める
    eps_comp=0.15,             # 補償の平滑化幅 [rad/s]。速度推定のノイズより大きく取る
)


def coulomb(dtheta, hw):
    """真のクーロン摩擦トルク [N·m]。正逆で大きさが違う。"""
    tau = hw["tau_c_pos"] if dtheta >= 0 else hw["tau_c_neg"]
    return -tau * np.tanh(dtheta / hw["eps_true"])


def friction_feedforward(dtheta_est, hw):
    """制御側の摩擦補償。sign() を直接使うとチャタリングするので tanh で平滑化する。"""
    tau = hw["tau_c_pos"] if dtheta_est >= 0 else hw["tau_c_neg"]
    return hw["comp_scale"] * tau * np.tanh(dtheta_est / hw["eps_comp"])


# ===========================================================================
# ゲイン設計
# ===========================================================================
def design_lqr(A, B, max_dev, max_u):
    """Bryson の法則で重みを作って LQR を解く。

    Q_ii = 1/(許容できる x_i の最大値)²、R = 1/(許容トルク)²。
    「何を何 rad まで許すか」という物理的な言葉で調整できるのが利点。
    """
    Q = np.diag(1.0 / np.asarray(max_dev) ** 2)
    R = np.array([[1.0 / max_u**2]])
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    # ★8/28追加: P はリヤプノフ関数 V(x) = xᵀPx の重み行列。
    # 指導教員ご指摘の最小射影法 min(V(x), (E(x)-Er)²+c) に使うため返す。
    return K, Q, R, P


def design_kalman(A, C, sigma_u, B, meas_lsb, jitter_std=0.0, speed_typ=5.0):
    """定常カルマンフィルタ（Luenberger オブザーバ）のゲイン。

    観測は θ と α の2つだけ。角速度2つはモデルが予測する。
    差分＋LPF と違い、モデルが速度を予測するぶん位相遅れが原理的に小さい。
    ここを素朴な差分にすると、量子化ノイズを潰すための LPF の位相遅れで発振する。

    ★2026/08/17：観測雑音を量子化だけで見積もっていたのは誤り★
    θ の LSB は 25/65535 = 3.8e-4 rad と細かいので、分散が 1.2e-8 になり、
    フィルタが θ をほぼ無雑音と信じて猛烈に微分する。J_r を実測値
    （従来の 5.3 倍）に更新したところ L が 2e5 に達し、量子化ノイズを
    増幅して全ケース転倒した。

    実機の位置測定にはタイミングジッタが乗る。標本化時刻が σ_t ずれると、
    速度 v で動いている軸の位置は v·σ_t だけずれて見える。実測 σ_t=0.563ms、
    v~5rad/s では 2.8e-3 rad となり**量子化の 7 倍**。これが支配的な雑音源。
    """
    # プロセス雑音は「トルクの不確かさ」として入れる（摩擦モデルの誤差が主因）
    Qn = (B * sigma_u) @ (B * sigma_u).T + np.diag([1e-9, 1e-9, 1e-6, 1e-6])
    # 観測雑音 = 量子化 q²/12 ＋ ジッタ由来 (v·σ_t)²
    Rn = np.diag([(lsb**2) / 12.0 + (speed_typ * jitter_std) ** 2
                  for lsb in meas_lsb])
    P = solve_continuous_are(A.T, C.T, Qn, Rn)
    L = P @ C.T @ np.linalg.inv(Rn)
    return L


# ===========================================================================
# 閉ループシミュレーション
# ===========================================================================
def simulate(model, K, L, hw, x0, duration=6.0, disturbance=None, seed=0):
    """非線形プラント＋離散制御器＋遅延＋量子化＋クーロン摩擦。"""
    local_rng = np.random.default_rng(seed)
    A, B, f = model["A"], model["B"], model["f"]
    C = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])

    sub_dt = 1e-4                      # プラント積分の刻み
    x = np.array(x0, dtype=float)
    xhat = np.zeros(4)
    xhat[:2] = x[:2]                   # 角度は測れるので初期値に入れる

    delay_steps = max(1, int(round(hw["transport_delay"] / sub_dt)))
    torque_pipe = [0.0] * delay_steps
    applied = 0.0

    t = 0.0
    next_control = 0.0
    log = {k: [] for k in ("t", "theta", "alpha", "dtheta", "dalpha",
                           "u", "u_fb", "u_fric", "alpha_hat", "dtheta_hat")}
    u_cmd = 0.0

    while t < duration:
        # ---- 制御器（離散、300 Hz、ジッタあり）---------------------------
        if t >= next_control:
            # 量子化した測定値
            y = np.array([
                np.round(x[0] / hw["enc_theta_lsb"]) * hw["enc_theta_lsb"],
                np.round(x[1] / hw["enc_alpha_lsb"]) * hw["enc_alpha_lsb"],
            ])

            dt_ctrl = 1.0 / hw["control_hz"]
            # オブザーバ更新（前回の指令 u_cmd を使う）
            xhat = xhat + dt_ctrl * (A @ xhat + B.reshape(4) * u_cmd + L @ (y - C @ xhat))

            u_fb = float((-K @ xhat).item())
            u_fric = friction_feedforward(xhat[2], hw)
            u_cmd = u_fb + u_fric

            # 飽和 → 量子化（実機と同じ順序）
            u_cmd = float(np.clip(u_cmd, -hw["torque_max"], hw["torque_max"]))
            u_cmd = np.round(u_cmd / hw["torque_lsb"]) * hw["torque_lsb"]

            # 次の制御時刻。ジッタとまれな外れ値を入れる
            period = dt_ctrl + local_rng.normal(0, hw["jitter_std"])
            if local_rng.random() < hw["outlier_prob"]:
                period = hw["outlier_s"]
            next_control = t + max(period, 0.2 * dt_ctrl)

            for key, val in (("u_fb", u_fb), ("u_fric", u_fric), ("u", u_cmd),
                             ("alpha_hat", xhat[1]), ("dtheta_hat", xhat[2])):
                log[key].append(val)
            log["t"].append(t)
            for i, key in enumerate(("theta", "alpha", "dtheta", "dalpha")):
                log[key].append(x[i])

        # ---- 伝送遅延 -------------------------------------------------------
        torque_pipe.append(u_cmd)
        applied = torque_pipe.pop(0)

        # ---- プラント（RK4）------------------------------------------------
        tau_ext = 0.0
        if disturbance is not None:
            tau_ext = disturbance(t)

        def rhs(state):
            # クーロン摩擦はモータ軸に効く。線形化モデルには含まれない項
            return f(state, applied + coulomb(state[2], hw) + tau_ext)

        k1 = rhs(x)
        k2 = rhs(x + 0.5 * sub_dt * k1)
        k3 = rhs(x + 0.5 * sub_dt * k2)
        k4 = rhs(x + sub_dt * k3)
        x = x + sub_dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        t += sub_dt

        if abs(x[1]) > 0.6:            # 35度を超えたら転倒とみなす
            return {k: np.array(v) for k, v in log.items()}, False

    return {k: np.array(v) for k, v in log.items()}, True


# ===========================================================================
def emit_csharp(K, L, A, B, P, path):
    """C# に埋め込む定数を書き出す。実行時に Python を呼ばない方針のため。"""
    def fmt(mat):
        rows = np.atleast_2d(mat)
        return ",\n".join("            { " + ", ".join(f"{v:+.9e}" for v in row) + " }"
                          for row in rows)

    text = f'''namespace DamiaoCan;

/// <summary>
/// Furuta 振子の制御ゲイン。
///
/// model/design_and_simulate.py で設計した値をそのまま埋め込んである。
/// 実行時に Python を呼ぶ構成にはしない（CLAUDE.md の方針）。
/// 機構やパラメータを変えたら、スクリプトを再実行してここを貼り替えること。
///
/// 状態 x = [θ, α, θ̇, α̇]ᵀ   θ:アーム角[rad]、α:振子角[rad]（上向きが0）
/// 入力 u = モータトルク [N·m]
/// 観測 y = [θ, α]ᵀ
/// </summary>
public static class FurutaGains
{{
    /// <summary>状態フィードバックゲイン K。u = -K·x̂ + 摩擦補償。</summary>
    public static readonly double[] StateFeedback =
    {{
        {", ".join(f"{v:+.9e}" for v in np.asarray(K).reshape(-1))}
    }};

    /// <summary>オブザーバゲイン L（4×2）。x̂̇ = A·x̂ + B·u + L·(y − C·x̂)。</summary>
    public static readonly double[,] ObserverGain = new double[,]
    {{
{fmt(L)}
    }};

    /// <summary>線形化した系の A 行列（4×4）。オブザーバの予測に使う。</summary>
    public static readonly double[,] A = new double[,]
    {{
{fmt(A)}
    }};

    /// <summary>線形化した系の B 行列（4×1）。</summary>
    public static readonly double[] B =
    {{
        {", ".join(f"{v:+.9e}" for v in np.asarray(B).reshape(-1))}
    }};

    /// <summary>
    /// リヤプノフ関数 V(x) = xᵀPx の重み行列 P（4×4）。リカッチ方程式の解。
    ///
    /// ★8/28追加: 指導教員ご指摘の最小射影法（Minimum Projection Method）で使う。
    /// W(x) = min( V(x), (E(x) - Er)² + c ) を全体のリヤプノフ関数とすると、
    /// V が小さい側（＝上向き付近でLQRが実際に立て直せる領域）では LQR、
    /// そうでない側ではエネルギー整形則、という切り替えが min そのもので決まり、
    /// 閉ループの安定性が理論的に保証される。
    /// 従来の「|α|&lt;25° かつ |α̇|&lt;3.0」という発見的な条件を置き換える。
    /// </summary>
    public static readonly double[,] RiccatiP = new double[,]
    {{
{fmt(P)}
    }};

    /// <summary>
    /// P の逆行列の (α,α) 成分。最小射影法で「V(x) ≤ c ならば |α| はいくつ以下か」を
    /// 求めるのに使う。制約付き最小化 min xᵀPx s.t. α=a の解が a²/(P⁻¹)[1,1] であることから、
    ///
    ///     V(x) ≤ c  ⟹  |α| ≤ √(c · RiccatiPinvAlphaAlpha)  [rad]
    ///
    /// ★8/28、c=100 では |α|≤150° まで通ってしまい、振り上げ開始直後に誤ってキャッチへ
    /// 切り替わる不具合が実機で出た。P の最小固有値が 1.16e-3 と極端に小さく、V には
    /// 「ほとんど増えない方向」があるため、角度が大きくても V が小さくなり得るのが原因。
    /// c は必ずこの式で保証角度に直してから決めること。
    /// </summary>
    public const double RiccatiPinvAlphaAlpha = {np.linalg.inv(P)[1, 1]:+.9e};

    /// <summary>クーロン摩擦補償の係数 [N·m]。正転・逆転で値が違う（実測）。</summary>
    public const double FrictionPositive = {HW["tau_c_pos"]};
    public const double FrictionNegative = {HW["tau_c_neg"]};

    /// <summary>摩擦補償の平滑化幅 [rad/s]。速度推定のノイズより大きく取ること。</summary>
    public const double FrictionEpsilon = {HW["eps_comp"]};

    /// <summary>制御周期 [Hz]。実測でこれ以上はジッタが増える。</summary>
    public const double ControlHz = {HW["control_hz"]};
}}
'''
    with open(path, "w", encoding="utf-8") as fp:
        fp.write(text)


def main():
    np.set_printoptions(precision=5, suppress=True, linewidth=120)

    print("モデルを導出中...")
    model = fm.build()
    A, B = model["A"], model["B"]
    C = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])

    # --- LQR ---------------------------------------------------------------
    # Bryson: 「θ は ±0.10 rad、α は ±0.05 rad まで許す。トルクは ±0.5 N·m まで」
    #
    # ★8/26再改訂: 0.15radでも実機で「−6〜−8°付近で少し粘るが、結局逃げる」を
    # 2回連続で確認（離し方はクリーンだったことを確認済み）。振子側は最後まで
    # 崩れておらず、αの制御自体は機能している。θの復元をさらに一段強める。
    # シミュレーション感度解析（0.5/0.15/0.10の3点）は最大トルクが単調に
    # 0.196→1.673→1.911N·mと増えており、0.10でもT_MAX=2.0N·mに約0.09N·mの
    # 余裕を残す（10°初期傾きのような大きい外乱ではほぼ飽和するため、
    # これより先へ絞る場合はmax_uも見直すこと）。
    #
    # ★8/26最初の改訂: θの許容偏差を0.5→0.15radに縮小（メンバーA）。
    # 実機のBalanceControlで、αは終始2〜3°以内に収まる一方、θがゆっくり流れて
    # 安全ガード(±40°)に達する現象が繰り返し観測された。原因を遡ると、旧設定
    # (θ許容0.5rad=28.6°)ではQ_θθ=1/0.5²=4がQ_αα=1/0.05²=400の1/100しかなく、
    # LQRが最初からθをほとんど気にしない設計だった（このシミュレーションでも
    # θドリフトが-17〜-21°と、実機と同じ傾向がすでに出ていた）。
    # θ許容を0.15rad(8.6°)まで絞ると、感度解析（摩擦補償comp_scale 0〜1.1、
    # アーム慣性J_r ×0.5〜3.0）を通しても全ケース成立を維持したまま、
    # θドリフトが最悪でも±10.6°まで縮小した。最大トルクも1.68N·m程度で
    # T_MAX=2.0N·mに対して余裕を残す。K要素の個別手動調整（thetaGainBoost等）は
    # かえって悪化することを実機で確認済みなので、必ずこの重み経由で
    # 4要素をまとめて再設計すること。
    # ★8/26 3回目の改訂: θ̇の許容偏差を3.0→1.5rad/sに縮小（メンバーA）。
    # スイングアップ成功後、保持中にアームが40〜84°の範囲で揺れ続ける
    # （静止しない）現象が残った。θ・αのズレ自体は小さいので位置の重みでは
    # なく減衰不足とみて、θ̇の重みQ_θ̇θ̇を1/3.0²→1/1.5²で4倍にする。
    max_dev = [0.10, 0.05, 1.5, 3.0]
    max_u = 0.5
    K, Q, R, P = design_lqr(A, B, max_dev, max_u)

    print("\n=== LQR ===")
    print(f"許容偏差 θ={max_dev[0]} rad, α={max_dev[1]} rad, "
          f"θ̇={max_dev[2]} rad/s, α̇={max_dev[3]} rad/s, u={max_u} N·m")
    print("K =", K.reshape(-1))
    cl = np.linalg.eigvals(A - B @ K)
    print("閉ループ極:")
    for e in sorted(cl, key=lambda z: -z.real):
        print(f"  {e.real:+8.3f} {e.imag:+8.3f}j")

    # --- カルマン -----------------------------------------------------------
    L = design_kalman(A, C, sigma_u=0.05, B=B,
                      meas_lsb=[HW["enc_theta_lsb"], HW["enc_alpha_lsb"]],
                      jitter_std=HW["jitter_std"], speed_typ=5.0)
    print("\n=== オブザーバ ===")
    print("L =\n", L)
    obs = np.linalg.eigvals(A - L @ C)
    print("オブザーバ極:", np.sort_complex(obs))

    # --- シミュレーション ---------------------------------------------------
    print("\n=== 閉ループ検証 ===")
    cases = [
        ("初期傾き 3°", np.deg2rad([0, 3, 0, 0]), None),
        ("初期傾き 6°", np.deg2rad([0, 6, 0, 0]), None),
        ("初期傾き 10°", np.deg2rad([0, 10, 0, 0]), None),
    ]

    # 外乱：2秒時点で 0.15 N·m を 30ms 加える（指で突く相当）
    def poke(t):
        return 0.15 if 2.0 <= t < 2.03 else 0.0

    cases.append(("外乱 0.15N·m×30ms", np.zeros(4), poke))

    results = {}
    for name, x0, dist in cases:
        logd, ok = simulate(model, K, L, HW, x0, duration=6.0, disturbance=dist, seed=7)
        results[name] = (logd, ok)
        if ok:
            tail = logd["alpha"][logd["t"] > 4.0]
            drift = logd["theta"][-1]
            print(f"  {name:20s} 成立  "
                  f"|α|の定常振幅 {np.rad2deg(np.abs(tail).max()):5.2f}°  "
                  f"θドリフト {np.rad2deg(drift):+7.1f}°  "
                  f"最大トルク {np.abs(logd['u']).max():.3f} N·m")
        else:
            print(f"  {name:20s} ★転倒★（{logd['t'][-1]:.2f} 秒）")

    # --- 感度解析 -----------------------------------------------------------
    # J_r（アーム慣性）は推定値、摩擦補償の効きも仮定値。
    # ここが外れたときに破綻しないかを確認しておかないと、設計として意味がない。
    print("\n=== 感度解析① 摩擦補償の効き（comp_scale）===")
    print("  0.0 = 補償なし、1.0 = 完全に既知、1.1 = 過補償")
    for scale in (0.0, 0.5, 0.7, 0.9, 1.0, 1.1):
        hw = dict(HW, comp_scale=scale)
        logd, ok = simulate(model, K, L, hw, np.deg2rad([0, 5, 0, 0]),
                            duration=6.0, seed=7)
        if ok:
            tail = np.abs(logd["alpha"][logd["t"] > 4.0])
            print(f"  comp_scale={scale:4.1f}  成立  "
                  f"リミットサイクル {np.rad2deg(tail.max()):5.2f}°")
        else:
            print(f"  comp_scale={scale:4.1f}  ★転倒★（{logd['t'][-1]:.2f} 秒）")

    print("\n=== 感度解析② アーム慣性 J_r（推定値なので要確認）===")
    for factor in (0.5, 0.75, 1.0, 1.5, 2.0, 3.0):
        p = dict(fm.PARAMS)
        p["J_r"] = fm.PARAMS["J_r"] * factor
        plant = fm.build(p)     # プラントだけ変え、ゲインは元のまま（=モデル誤差）
        logd, ok = simulate(plant, K, L, HW, np.deg2rad([0, 5, 0, 0]),
                            duration=6.0, seed=7)
        label = f"  J_r×{factor:<4.2f}（{p['J_r']*1000:5.1f} g·m²）"
        if ok:
            tail = np.abs(logd["alpha"][logd["t"] > 4.0])
            print(f"{label}  成立  リミットサイクル {np.rad2deg(tail.max()):5.2f}°")
        else:
            print(f"{label}  ★転倒★（{logd['t'][-1]:.2f} 秒）")

    emit_csharp(K, L, A, B, P, "FurutaGains.cs")
    print("\n保存: FurutaGains.cs（C# に貼り付ける定数）")

    np.savez("gains.npz", K=K, L=L, A=A, B=B, P=P)
    plot(results, model, K, L)
    return model, K, L, results


def plot(results, model, K, L):
    """代表ケースの時系列を描く。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    logd, ok = results["初期傾き 6°"]
    dist, _ = results["外乱 0.15N·m×30ms"]

    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

    ax[0].plot(logd["t"], np.rad2deg(logd["alpha"]), lw=1.2, label="alpha (true)")
    ax[0].plot(logd["t"], np.rad2deg(logd["alpha_hat"]), lw=0.9, ls="--",
               label="alpha (observer)")
    ax[0].axhline(0, color="k", lw=0.5)
    ax[0].set_ylabel("pendulum [deg]")
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    ax[0].set_title("Furuta pendulum: 6 deg initial tilt, with measured delay / "
                    "quantisation / Coulomb friction", fontsize=10)

    ax[1].plot(logd["t"], np.rad2deg(logd["theta"]), lw=1.2, color="#2471a3")
    ax[1].set_ylabel("arm [deg]"); ax[1].grid(alpha=0.3)

    ax[2].plot(logd["t"], logd["u"], lw=1.0, color="#c0392b", label="total")
    ax[2].plot(logd["t"], logd["u_fric"], lw=0.9, ls="--", color="#e67e22",
               label="friction FF")
    ax[2].plot(logd["t"], logd["u_fb"], lw=0.8, ls=":", color="#7d3c98",
               label="LQR")
    ax[2].axhline(2.0, color="k", lw=0.5, ls="--")
    ax[2].axhline(-2.0, color="k", lw=0.5, ls="--")
    ax[2].set_ylabel("torque [N.m]"); ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)

    zoom = logd["t"] > 4.0
    ax[3].plot(logd["t"][zoom], np.rad2deg(logd["alpha"][zoom]), lw=1.0)
    ax[3].set_ylabel("limit cycle [deg]"); ax[3].set_xlabel("time [s]")
    ax[3].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("simulation.png", dpi=140)
    print("保存: simulation.png")


if __name__ == "__main__":
    main()
