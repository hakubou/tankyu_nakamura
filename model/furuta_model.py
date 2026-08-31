#!/usr/bin/env python3
"""Furuta 振子（回転型倒立振子）のモデル。

■ 方針

運動方程式は **SymPy で機械的に導出する**。手計算すると Coriolis 項の符号を必ず誤るため。
導出結果から
  - 非線形の状態方程式 f(x, u)  … シミュレーション用
  - 平衡点まわりの線形化 A, B    … LQR / カルマンゲイン設計用
の両方を作る。

■ 座標系

    θ : アーム角 [rad]（鉛直軸まわり。モータが直接駆動する）
    α : 振子角 [rad]（**上向きを 0**、ぶら下がった状態が π）

    ê_r = (cosθ, sinθ, 0)   アームの向き（半径方向・水平）
    ê_t = (-sinθ, cosθ, 0)  接線方向・水平
    ê_z = (0, 0, 1)         鉛直上向き

    振子はアーム先端で ê_r 軸まわりに回り、接線-鉛直面内で振れる。
    振子重心の位置:  p = L_r·ê_r + l_p·(sinα·ê_t + cosα·ê_z)

    状態 x = [θ, α, θ̇, α̇]ᵀ、入力 u = モータトルク [N·m]

■ このモデルに含めるもの／含めないもの

含める : アームの粘性 b_r、振子軸の粘性 b_p、重力、慣性結合（Coriolis 含む）
含めない: クーロン摩擦（不連続なので線形化できない）
          → シミュレーション側で別途加える。線形化モデルには入れない。
"""

from __future__ import annotations

import numpy as np
import sympy as sp

G = 9.80665

# ===========================================================================
# パラメータ
# ===========================================================================
# 出典の区別を明確にしておくこと。実測値と推定値を混同すると
# 「モデルが合わない」ときにどこを疑うべきか分からなくなる。
# 2026/08/12 更新: 摩擦検証（friction_study.py）でアーム延長が不要と分かったため、
# **QUBE モジュールをアダプタで直付けする素の構成**に切り替えた。
# 振子側は同日の自由振動測定（pendulum_identification.py）で実測値に置き換え済み。
PARAMS = dict(
    # --- アーム側 -----------------------------------------------------------
    # L_r: モータ回転軸 → 振子の回転軸 までの距離。
    #      銀の丸棒（全長 112.20mm 実測）のうちブロックから出ている分 ＋
    #      アダプタでのオフセット。
    #      2026/08/21 組み立て後に実測: 113〜114mm。
    #      85〜120mm の範囲では結果がほぼ変わらないことを確認済みのため、
    #      この値でもゲイン（K, L, A, B）の再計算は不要と判断。
    L_r=0.1135,
    # J_r: アーム組立体（アダプタ板＋QUBEモジュール本体）のモータ軸まわり慣性。
    #      振子質量による寄与 m_p·L_r² はモデル側で別に足すのでここには含めない。
    #
    #      ★2026/08/17 実測★ 質量からの概算 0.0004 は **5倍以上の過小評価**だった。
    #      原因はほぼ確実に**反射ロータ慣性**。減速比 10:1 なので
    #      ロータ慣性は出力軸から見て 10² = 100 倍に見える。
    #      逆算すると J_rotor ≈ 1.7e-5 kg·m² で、この規模のモータとして妥当。
    #      概算では機構部品しか数えておらず、これを完全に落としていた。
    #
    #      測定法: アームを ±5°・4〜9 Hz で正弦波駆動し、トルク RMS から
    #      既知の摩擦分を差し引いて J を出す（OpenLoopSwing --mode inertia）。
    #      慣性項は sin、摩擦項は cos に同相で**直交する**ため分離できる。
    #      6〜9 Hz の 4 点で実効慣性 2.20e-3 ± 0.25e-3（摩擦係数の仮定を
    #      振っても 1.95〜2.37e-3 に収まる）。そこから m_p·L_r² − M12²/M22 を
    #      引いて J_r = 2.13e-3 を得た。
    #
    #      4 Hz 以下は振子の共振（1.7 Hz）に近く振子が反力を返すため使えない。
    #      高周波では振子が慣性で取り残され、アーム側だけが測れる。
    J_r=0.00213,

    # --- 振子側（すべて 2026/08/12 実測）-------------------------------------
    # m_p: 赤いパイプの質量。★カタログ値★ 現物が外せず未実測。
    #      なお周期は質量に依存しないため、形状の検証には影響しない。
    m_p=0.024,
    # l_p: 振子の回転軸 → 重心。実測 128.70mm の一様棒を、
    #      端から 7.0mm の位置で吊っているとして 128.70/2 − 7.0 = 57.35mm
    l_p=0.05735,
    # J_p: 振子の重心まわり慣性。自由振動の周期 0.5730s から逆算。
    #      幾何形状からの予測 3.313e-5 と 0.17% で一致したので、
    #      「一様棒＋ピボット 7mm」の仮定は妥当と確認できた。
    J_p=3.332e-5,

    # --- 摩擦 ---------------------------------------------------------------
    # b_r: アーム側（モータ）の粘性。★実測値★（2026/08/12）
    b_r=0.027,
    # b_p: 振子軸の粘性。★実測値★ 対数減衰 δ=0.0624/往復 から。
    #      減衰の当てはまりは粘性 R²=0.98 / クーロン R²=0.87 で、
    #      実際は両者の混合とみられるが、いずれも 1e-5 オーダー。
    b_p=2.446e-5,
)


# ===========================================================================
# 記号による導出
# ===========================================================================
def derive():
    """Lagrange 法で運動方程式を導出し、(質量行列 M, 右辺 rhs, 記号) を返す。

    M(q)·q̈ = rhs(q, q̇, u)  の形に整理する。
    """
    t = sp.symbols("t", real=True)
    L_r, J_r, m_p, l_p, J_p, b_r, b_p, g, u = sp.symbols(
        "L_r J_r m_p l_p J_p b_r b_p g u", real=True, positive=False)

    th = sp.Function("theta")(t)
    al = sp.Function("alpha")(t)
    dth, dal = sp.diff(th, t), sp.diff(al, t)

    # --- 幾何 ---------------------------------------------------------------
    e_r = sp.Matrix([sp.cos(th), sp.sin(th), 0])
    e_t = sp.Matrix([-sp.sin(th), sp.cos(th), 0])
    e_z = sp.Matrix([0, 0, 1])

    # 振子重心の位置と速度
    p = L_r * e_r + l_p * (sp.sin(al) * e_t + sp.cos(al) * e_z)
    v = sp.diff(p, t)

    # --- 運動エネルギー -----------------------------------------------------
    # アームの回転
    T_arm = sp.Rational(1, 2) * J_r * dth**2

    # 振子重心の並進
    T_trans = sp.Rational(1, 2) * m_p * (v.T * v)[0]

    # 振子自身の回転。細長い棒なので長手軸まわりの慣性は無視し、
    # それに直交する2軸まわりを J_p とする → I = J_p·(単位行列 − û·ûᵀ)
    omega = dal * e_r + dth * e_z              # 振子の角速度
    u_hat = sp.sin(al) * e_t + sp.cos(al) * e_z  # 振子の長手方向
    T_rot = sp.Rational(1, 2) * J_p * ((omega.T * omega)[0] - ((omega.T * u_hat)[0])**2)

    T = sp.simplify(T_arm + T_trans + T_rot)

    # --- 位置エネルギー -----------------------------------------------------
    # α=0（上向き）で最大。ぶら下がり（α=π）で最小
    U = m_p * g * l_p * sp.cos(al)

    # --- Euler-Lagrange -----------------------------------------------------
    Lag = T - U
    eq_th = sp.diff(sp.diff(Lag, dth), t) - sp.diff(Lag, th) - (u - b_r * dth)
    eq_al = sp.diff(sp.diff(Lag, dal), t) - sp.diff(Lag, al) - (-b_p * dal)

    ddth, ddal = sp.symbols("ddtheta ddalpha", real=True)
    subs_acc = {sp.diff(th, t, 2): ddth, sp.diff(al, t, 2): ddal}
    eqs = [sp.expand(eq_th.subs(subs_acc)), sp.expand(eq_al.subs(subs_acc))]

    # M·q̈ = rhs の形へ
    M = sp.zeros(2, 2)
    rhs = sp.zeros(2, 1)
    for i, eq in enumerate(eqs):
        for j, acc in enumerate((ddth, ddal)):
            M[i, j] = sp.simplify(sp.diff(eq, acc))
        rhs[i] = sp.simplify(-eq.subs({ddth: 0, ddal: 0}))

    # 関数を素の記号に置き換えて、後で数値化しやすくする
    s_th, s_al, s_dth, s_dal = sp.symbols("theta alpha dtheta dalpha", real=True)
    swap = {dth: s_dth, dal: s_dal, th: s_th, al: s_al}
    M = M.subs(swap)
    rhs = rhs.subs(swap)

    syms = dict(theta=s_th, alpha=s_al, dtheta=s_dth, dalpha=s_dal, u=u, g=g,
                L_r=L_r, J_r=J_r, m_p=m_p, l_p=l_p, J_p=J_p, b_r=b_r, b_p=b_p)
    return M, rhs, syms


# ===========================================================================
# 数値化
# ===========================================================================
def build(params: dict | None = None):
    """導出結果を数値関数に変換し、モデル一式を返す。"""
    p = dict(PARAMS if params is None else params)
    M, rhs, s = derive()

    const = {s["g"]: G, **{s[k]: p[k] for k in
                           ("L_r", "J_r", "m_p", "l_p", "J_p", "b_r", "b_p")}}
    Mn = M.subs(const)
    rn = rhs.subs(const)

    state = (s["theta"], s["alpha"], s["dtheta"], s["dalpha"])
    args = (*state, s["u"])

    M_fn = sp.lambdify(args, Mn, "numpy")
    r_fn = sp.lambdify(args, rn, "numpy")

    def M_rhs(x, u):
        """M(q) と rhs を返す。M·q̈ = rhs。

        固着判定（Karnopp 型の摩擦モデル）には加速度そのものではなく
        「θ̈=0 に保つために必要な摩擦トルク」が要るので、この分解が必要になる。
        """
        th, al, dth, dal = x
        M = np.asarray(M_fn(th, al, dth, dal, u), dtype=float)
        r = np.asarray(r_fn(th, al, dth, dal, u), dtype=float).reshape(2)
        return M, r

    def f(x, u, tau_friction=0.0):
        """非線形の状態方程式 ẋ = f(x, u)。

        粘性は含む。クーロン摩擦は tau_friction で外から与える
        （不連続なのでモデル内には入れられない）。
        """
        M, r = M_rhs(x, u)
        r = r + np.array([tau_friction, 0.0])
        acc = np.linalg.solve(M, r)
        return np.array([x[2], x[3], acc[0], acc[1]])

    # --- 平衡点（振子が上向き）まわりの線形化 -------------------------------
    # ẋ = f(x,u) のヤコビアンを記号的に作ってから代入する。
    # 数値差分より正確で、符号ミスも起きない。
    q_dd = Mn.LUsolve(rn)
    f_sym = sp.Matrix([s["dtheta"], s["dalpha"], q_dd[0], q_dd[1]])
    A_sym = f_sym.jacobian(sp.Matrix(state))
    B_sym = f_sym.jacobian(sp.Matrix([s["u"]]))

    equil = {s["theta"]: 0, s["alpha"]: 0, s["dtheta"]: 0, s["dalpha"]: 0, s["u"]: 0}
    A = np.array(A_sym.subs(equil).evalf(), dtype=float)
    B = np.array(B_sym.subs(equil).evalf(), dtype=float)

    return dict(params=p, f=f, M_rhs=M_rhs, A=A, B=B)


# ===========================================================================
def main():
    np.set_printoptions(precision=5, suppress=True, linewidth=110)
    print("Furuta 振子の運動方程式を SymPy で導出中...")
    model = build()
    A, B, p = model["A"], model["B"], model["params"]

    print("\n=== パラメータ ===")
    for k, v in p.items():
        print(f"  {k:5s} = {v}")

    print("\n=== 線形化モデル（x = [θ, α, θ̇, α̇]ᵀ, u = トルク[N·m]）===")
    print("A =\n", A)
    print("B =\n", B.reshape(-1))

    eig = np.linalg.eigvals(A)
    print("\n開ループ極:")
    for e in sorted(eig, key=lambda z: -z.real):
        print(f"  {e.real:+8.4f} {e.imag:+8.4f}j")

    unstable = [e.real for e in eig if e.real > 1e-6]
    if unstable:
        lam = max(unstable)
        print(f"\n不安定極 λ = {lam:.3f} rad/s  →  時定数 {1/lam*1000:.0f} ms")
        print("（この時定数に対して制御周期 3.33ms・遅延 5ms なので、遅延は約"
              f" {5/(1000/lam)*100:.0f}%）")

    # 可制御性。ここが落ちていたらモデルか座標系が間違っている
    C = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(4)])
    print(f"\n可制御性行列のランク: {np.linalg.matrix_rank(C)} / 4")

    np.savez("furuta_linear.npz", A=A, B=B, **{f"p_{k}": v for k, v in p.items()})
    print("\n保存: furuta_linear.npz")


if __name__ == "__main__":
    main()
