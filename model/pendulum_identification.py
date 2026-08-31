#!/usr/bin/env python3
"""振子パラメータの同定（2026/08/12 の実測から）。

■ 実測値

  赤い丸棒（振子本体）  128.70 mm
  ピボットより上の部分    約 7 mm
  銀の丸棒の直径          6.40 mm（E8T-512-**250** = 1/4in = 6.35mm と一致）
  銀の丸棒の全長        112.20 mm（赤い筒を貫通しているため全体長）
  自由振動              5.82 / 5.68 / 5.69 秒
  減衰                  A0=3.5, A10=1.5, A20=1.0, A30=0.5 cm

■ 周期の解釈について

「20往復で 5.73 秒」= T 0.285 秒 は物理的にありえない。
質量が全部重心に集中していても J_pivot は m·l² 以上あり、
T=0.285 秒には J=2.8e-5 が必要だが下限は 7.9e-5。
5.73 秒を **10往復**（＝片道20回）と読むと T=0.573 秒となり、
幾何形状からの予測 0.5727 秒と 0.2% で一致する。よってそちらを採る。

■ 質量について

赤い筒が外せなかったため m_p はカタログ値 0.024 kg を使う。
なお**周期は質量に依存しない**（J も m·g·l も m に比例するため）ので、
m_p の不確かさは「形状が一様棒として妥当か」の検証には影響しない。
影響するのは J_pivot と b_p の絶対値のみ。
"""

from __future__ import annotations

import numpy as np

G = 9.80665

# --- 実測 -------------------------------------------------------------------
L_TUBE = 128.70e-3        # 赤い丸棒の全長 [m]
ABOVE_PIVOT = 7.0e-3      # ピボットより上に出ている長さ [m]
ROD_TOTAL = 112.20e-3     # 銀の丸棒の全長 [m]
SWING_TIMES = [5.82, 5.68, 5.69]   # [s]
CYCLES = 10               # 上記時間に含まれる「往復」の回数
M_P = 0.024               # カタログ値 [kg]（現物は外せず未実測）

AMP = {0: 3.5e-2, 10: 1.5e-2, 20: 1.0e-2, 30: 0.5e-2}   # [m]（先端の振れ幅）


def main():
    print("=" * 70)
    print("振子パラメータの同定")
    print("=" * 70)

    # --- 幾何 ---------------------------------------------------------------
    # 一様な棒がピボットから 7mm の位置で吊られている、と仮定する
    l_p = L_TUBE / 2 - ABOVE_PIVOT          # ピボット → 重心
    tip = L_TUBE - ABOVE_PIVOT              # ピボット → 先端（振れ幅の基準）
    J_com = M_P * L_TUBE**2 / 12            # 重心まわり（一様棒）
    J_pivot_geom = J_com + M_P * l_p**2     # 平行軸定理

    print(f"\n[幾何から]")
    print(f"  ピボット→重心 l_p      = {l_p*1000:8.2f} mm")
    print(f"  ピボット→先端          = {tip*1000:8.2f} mm")
    print(f"  重心まわり慣性 J_p      = {J_com:10.4e} kg·m²")
    print(f"  ピボットまわり J_pivot  = {J_pivot_geom:10.4e} kg·m²")
    print(f"  予測周期 T              = {2*np.pi*np.sqrt(J_pivot_geom/(M_P*G*l_p)):8.4f} s")

    # --- 周期から -----------------------------------------------------------
    t_mean = float(np.mean(SWING_TIMES))
    T = t_mean / CYCLES
    omega_n = 2 * np.pi / T
    mgl = M_P * G * l_p
    J_pivot = mgl * (T / (2 * np.pi))**2

    print(f"\n[周期から]")
    print(f"  {CYCLES}往復の平均       = {t_mean:8.3f} s  (ばらつき ±{np.std(SWING_TIMES):.3f})")
    print(f"  周期 T                  = {T:8.4f} s")
    print(f"  固有角振動数 ω_n        = {omega_n:8.3f} rad/s")
    print(f"  m·g·l                   = {mgl:10.4e} N·m")
    print(f"  ピボットまわり J_pivot  = {J_pivot:10.4e} kg·m²")
    print(f"  重心まわり J_p          = {J_pivot - M_P*l_p**2:10.4e} kg·m²")

    err = abs(J_pivot - J_pivot_geom) / J_pivot_geom * 100
    print(f"\n  → 幾何予測との差 {err:.2f}%")
    if err < 3:
        print("     一様棒＋ピボット位置 7mm の仮定は妥当。カタログの L_p=129mm 側と整合。")

    # --- 減衰 ---------------------------------------------------------------
    # 振幅は先端の振れ幅なので、角度に直してから使う
    n = np.array(sorted(AMP))
    a = np.array([AMP[k] for k in n])
    theta = a / tip

    print(f"\n[減衰から]")
    print(f"  {'n':>4} {'振幅[cm]':>10} {'角度[deg]':>10} {'ln(θ)':>9}")
    for k, amp, th in zip(n, a, theta):
        print(f"  {k:4d} {amp*100:10.2f} {np.rad2deg(th):10.2f} {np.log(th):9.4f}")

    # 粘性（指数減衰）として当てはめる
    delta = -np.polyfit(n, np.log(theta), 1)[0]        # 1往復あたりの対数減衰率
    zeta = delta / np.sqrt(4 * np.pi**2 + delta**2)
    b_p = 2 * zeta * np.sqrt(J_pivot * mgl)

    # クーロン（直線減衰）として当てはめる。振幅の減り方が直線なら乾性摩擦
    slope = -np.polyfit(n, theta, 1)[0]                # 1往復あたりの角度の減り
    tau_c_p = slope * mgl / 4.0

    lin_r2 = np.corrcoef(n, theta)[0, 1]**2
    log_r2 = np.corrcoef(n, np.log(theta))[0, 1]**2

    print(f"\n  粘性モデル : 対数減衰率 δ = {delta:.5f}/往復  →  ζ = {zeta:.5f}")
    print(f"               b_p = {b_p:10.4e} N·m·s/rad     (決定係数 R² = {log_r2:.4f})")
    print(f"  クーロン   : 1往復あたり {np.rad2deg(slope):.3f}°減衰")
    print(f"               τ_c,p = {tau_c_p:10.4e} N·m       (決定係数 R² = {lin_r2:.4f})")
    print(f"\n  → {'粘性' if log_r2 > lin_r2 else 'クーロン'}モデルの方が当てはまりが良い")
    print(f"     ただしどちらも R² が高くなく、実際は両者の混合とみられる。")
    print(f"     いずれにせよ 10⁻⁵ オーダーで、モデルの仮定値 2.0e-5 と同程度。")

    # --- Furuta モデル用にまとめる -------------------------------------------
    print("\n" + "=" * 70)
    print("furuta_model.PARAMS に入れる値")
    print("=" * 70)
    print(f"    m_p = {M_P:.4f},        # カタログ値（現物は外せず未実測）")
    print(f"    l_p = {l_p:.5f},       # 実測 128.70mm / ピボット 7mm から")
    print(f"    J_p = {J_pivot - M_P*l_p**2:.5e},   # 周期 {T:.4f}s から逆算")
    print(f"    b_p = {b_p:.3e},      # 対数減衰から")
    print(f"\n  L_r（実効アーム長）は組み立て後に要実測。")
    print(f"  銀の丸棒 全長 {ROD_TOTAL*1000:.2f} mm のうち、ブロックから出ている長さ ＋")
    print(f"  アダプタでのオフセット が L_r になる。")


if __name__ == "__main__":
    main()
