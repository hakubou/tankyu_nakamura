#!/usr/bin/env python3
"""QUBE 振子モジュールを DM-J4310 出力軸に直付けするアダプタの STL を生成する。

■ 何のための部品か

2026/08/12 の摩擦検証（model/friction_study.py）で、**アームを延長しなくても
素の QUBE 寸法のまま立つ**ことが分かった。そこで、モジュールをモータ出力軸へ
そのまま固定するだけのアダプタを作る。オフセットブラケット（215mm）は不要。

■ 構成：2枚の平板をボルトで重ねる

    [部品2 クレードル板]  厚 12mm。中央に角穴。ここへモジュールの円筒が落ちる
           ↓ M3 × 4 で結合
    [部品1 ベース板]      厚 5mm。モータ出力フランジへ固定。角穴の床になる
           ↓ M3 × n（半径方向スロット）
    DM-J4310 出力フランジ

なぜ2枚に分けるか: どちらも「一定厚の押し出し」になり、検証済みの
三角形分割器（generate_calibration_arm.py）をそのまま使えるため。
1部品で座ぐりや袋穴を作ろうとすると 2.5D のメッシャが要る。

■ 姿勢：取り付け平面を「下向き」にする

モジュールの取り付け面（磁石のある平らな彫り込み）をベース板の上に伏せて置く。
こうすると
  - 平面 対 平面 の面接触になり、姿勢が安定する
  - **磁石が本来の役割（下向きの保持）で効く**
  - ピボットが低くなる（平面を横向きにする案より 5mm 低い）

■ トルクの伝達経路（磁石には持たせない）

    モータ → ベース板 → M3×4 → クレードル板 → **角穴の壁** → 円筒の側面

    ヨー（鉛直軸まわりの回転）: 角穴の両壁が円筒の側面を挟んで止める
    ロール（円筒軸まわりの転がり）: 平面がベース板に伏せているので止まる

円筒の最大幅は平面から flat_offset の高さにあるので、クレードル厚は
それを越える値にすること（validate で確認している）。
磁石は保持のみ、中央の丸穴（ネジ山なし）は位置決めのみ。

■ 向き

モジュールの円筒軸＝振子の回転軸が**半径方向**を向くように角穴を切ってある。
銀の丸棒が外側へ伸び、その先の赤いパイプが接線-鉛直面内で振れる。

■ ★未確定の寸法★

以下は写真からの推定値。**実測して差し替えること。**
    --motor-bolt-circle   出力フランジのボルト円
    --motor-bolt-count    ボルト本数
    --body-d              モジュール円筒の外径
    --flat-offset         円筒軸から取り付け平面までの距離
    --body-length         円筒の長さ

使い方:
    python3 generate_qube_adapter.py
    python3 generate_qube_adapter.py --body-d 30 --flat-offset 12 --gauge
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

from generate_calibration_arm import (arc, bridge_holes, check_manifold, dedupe,
                                      extrude, signed_area, triangulate, write_stl)

DEFAULTS = dict(
    # --- モータ側（2026/08/12 に公式図面で確定）------------------------------
    # 出力フランジ（回転する中央部）: 6×M3↓5、ボルト円 Ø27、3×Ø4 の精密位置決め穴あり。
    # 外側の 6×M3 / Ø50 は固定ハウジング側なので使わない。
    motor_bolt_circle=27.0,
    motor_bolt_count=6,
    motor_bolt_d=3.4,          # M3 バカ穴
    slot_tolerance=0.6,        # 半径方向スロットの遊び。ボルト円が Ø27 と確定したので印刷公差ぶんのみ
    center_clear_d=14.0,       # 中央の逃がし穴

    # --- スペーサリング -------------------------------------------------------
    # 回転ハブは Ø35 で、固定ハウジング面から 1mm しか出ていない（図面の 45/46 の差）。
    # ベース板を直付けすると Ø35 より外側がハウジングと 1mm 差で対面し、擦る危険がある。
    # 擦れば摩擦が増えて制御が壊れるので、リングを挟んで隙間を稼ぐ。
    # 出っ張り量が未実測（図面の 45/46 の差から 1mm と推定）なので、
    # スペーサを厚めにして「出っ張り 0mm でも成立する」設計にしてある。
    # 実測できたら薄くしてピボットを下げてよい。
    spacer_od=34.0,            # 回転フランジ Ø34.95（実測）に収まること
    spacer_id=18.0,
    spacer_t=4.0,              # 出っ張り 1mm（実測）+ これ = 隙間 5mm

    # --- 位置決めピン（3×Ø4、PCD Ø24.30。公差 +0.02/+0.04 の精密穴）----------
    # ボルトの摩擦ではなく**形状**でトルクを受けるために使う。
    # PLA はボルト予圧でクリープするので、摩擦だけに頼る締結は経時的に緩む。
    #
    # 2026/08/21 実測: 隣り合うM3穴とØ4ピン穴の中心間距離が約6.875mm。
    # 半径13.5mm(M3)と12.15mm(ダウエル)から理論値を計算すると、
    # 角度差0°なら約1.35mm、30°なら約6.8mmになるため、角度差は30°で確定。
    #
    # ★★ dowel_angle=30 を試したところ validate() の干渉チェック（195行目）は
    # 余裕を持って通過する（必要4.7mm、実際6.3mm）が、build_base_levels() で
    # 「耳が見つかりませんでした。形状が自己交差している可能性があります」で
    # 落ちる。999(無効)なら4部品とも問題なく生成できるため、ピン穴を有効化する
    # コードパス自体に未修正のバグがある（このコードパスは今まで dowel_angle=999
    # 固定だったため一度も実行されたことがなかった）。原因未特定。
    # 直すまでは無効(999)のまま、ボルトのみの締結で印刷すること。
    dowel_pcd=24.30,
    dowel_count=3,
    dowel_d=4.05,              # Ø4 平行ピン用。PLA なので実測合わせが要る
    dowel_angle=999.0,         # 角度は30°で確定済みだが、有効化すると生成が落ちるバグ未修正のため無効のまま

    # --- QUBE モジュール側（★要実測★）--------------------------------------
    # 2026/08/12 実測:
    #   外径 28.0 / 平面から外側まで 24.5（→ 軸から平面は 10.5）/ 全長 52.5（エンコーダ込み）
    #   後端から 8mm に 23×23 の窪み（磁石）、窪みの 4mm 先に M4 のネジ穴
    body_d=28.0,               # 赤い円筒の外径（実測）
    flat_offset=10.5,          # 円筒軸 → 取り付け平面（実測 24.5 − 外径の半分）
    body_length=52.5,          # 円筒の全長（実測、エンコーダ部を含む）
    pocket_length=42.0,        # 角穴で**捕捉する**長さ。エンコーダ側は外へ張り出させる
    fit_clearance=0.4,         # 印刷公差ぶんの隙間（片側）
    # 溝を外側へずらす量。窪み中心（＝ペグ位置）を M3 スロットの外へ逃がすために必要。
    # 24mm でペグは R22.1 となり、スロット外端 15.8 から 3.3mm の肉厚が取れる。
    pocket_offset=24.0,
    m4_x=37.6,                 # M4 締結穴の**絶対半径**。後端から 35mm ＋ 溝の配置から算出
    m4_y=0.0,
    m4_clear_d=4.5,            # M4 バカ穴

    # --- 板 ------------------------------------------------------------------
    base_d=86.0,               # ベース板の外径
    # 板厚 8mm は M4 ネジ（全長 19.4 / 頭 4.1 → 軸 15.3）に合わせた値。
    # モジュール側へのねじ込みが 7.3mm となり、M4 として十分かつ底付きしにくい。
    base_t=8.0,
    cradle_t=20.0,             # 円筒の最大幅は「半径 14mm の高さ」なのでそれを越える
    join_bolt_d=3.4,           # 2枚を結合する M3
    join_circle=70.0,          # 結合ボルトのボルト円。角穴の角を避ける必要がある
    join_count=4,
    tie_slot_w=3.5,            # 結束バンド用スロットの幅
    tie_slot_l=12.0,

    recess_size=23.0,          # 取り付け面の正方形の窪み（実測 23×23）
    recess_depth=3.3,          # 窪みの深さ 3.5 よりわずかに薄くし、円筒面が確実に接するようにする
    # 窪み中央の穴が Ø6 か Ø7 か確定していないため、両方を出力する。
    # 穴径 −0.2mm を狙う（印刷の膨らみを見込んだ嵌め合い）。
    # 窪み中央の穴は実測 Ø6。ペグはベース板と一体成形にした
    # （別部品のタイルだとネジがベース板の穴と干渉し、かつ固定しないと
    #   トルクの経路にならなかったため）。
    peg_d=5.8,                 # Ø6 の穴に対して −0.2mm
    peg_h=5.0,

    # --- 側面レール（クレードル板の置き換え）----------------------------------
    # 振子モジュールの後端は形状が不定なので、そこには触らず**横方向だけ**支える。
    # 円板のクレードル（93cm3）より遥かに小さく、印刷も速い。
    rail_x0=10.0,              # レールの内端（モジュール後端 x=2.6 より外側）
    rail_x1=40.0,              # 外端（ベース板 R43 の内側）
    rail_w=12.0,               # レールの幅（y方向）
    rail_bolt_d=3.4,
    rail_bolt_pilot=2.6,
    rail_bolt_inset=6.0,       # レール内端からボルトまでの距離

    build_volume=180.0,
)


# ===========================================================================
def validate(p):
    errors = []
    margin = 1.0

    slot_in = p["motor_bolt_circle"] / 2 - p["slot_tolerance"] - p["motor_bolt_d"] / 2
    slot_out = p["motor_bolt_circle"] / 2 + p["slot_tolerance"] + p["motor_bolt_d"] / 2

    if slot_in < p["center_clear_d"] / 2 + margin:
        errors.append(
            f"モータ用スロットの内端 R{slot_in:.1f} が中央逃がし穴 "
            f"R{p['center_clear_d']/2:.1f} に近すぎます。"
            f"\n    → --center-clear-d を {2*(slot_in-margin):.0f} 以下に")

    # 取り付け平面は円筒に 3.5mm 掘り込まれた窪みなので、板に載せると
    # **平面ではなく円筒面が当たる**。よって溝の幅は円筒の外径に合わせる。
    slot_w = p["body_d"] + 2 * p["fit_clearance"]
    slot_l = p["pocket_length"] + 2 * p["fit_clearance"]

    if p["pocket_length"] > p["body_length"]:
        errors.append("--pocket-length が円筒の全長を超えています。")

    # レールのボルトがベース板に収まり、他の穴と干渉しないか
    for bx, by in rail_bolt_positions(p):
        r = math.hypot(bx, by)
        if r + p["rail_bolt_d"] / 2 > p["base_d"] / 2 - margin:
            errors.append(f"レールのボルト（R{r:.1f}）がベース板からはみ出します。"
                          f"--base-d を {2*(r+p['rail_bolt_d']/2+margin):.0f} 以上に")
        if slot_in - 2 < r < slot_out + 2:
            errors.append(f"レールのボルト（R{r:.1f}）がモータ用スロットと干渉します。")
    if p["rail_x1"] > p["base_d"] / 2:
        errors.append(f"レールの外端 x={p['rail_x1']:.0f} がベース板（R{p['base_d']/2:.0f}）を超えます。")

    sp_slot_out = p["motor_bolt_circle"] / 2 + p["slot_tolerance"] + p["motor_bolt_d"] / 2
    if p["spacer_od"] / 2 < sp_slot_out + margin:
        errors.append(f"スペーサ外径 Ø{p['spacer_od']:.0f} がボルトスロット "
                      f"R{sp_slot_out:.1f} を覆えません。")
    if p["spacer_id"] / 2 > p["motor_bolt_circle"] / 2 - p["slot_tolerance"] - p["motor_bolt_d"] / 2 - margin:
        errors.append(f"スペーサ内径 Ø{p['spacer_id']:.0f} がボルトスロットに食い込みます。")

    if p["dowel_angle"] < 900:
        # M3 スロットと Ø4 ピン穴が半径方向に重なっているので、
        # 角度で離れていないと穴が繋がってしまう
        r_d = p["dowel_pcd"] / 2
        need = p["dowel_d"] / 2 + p["motor_bolt_d"] / 2 + 1.0
        worst = min(abs(((p["dowel_angle"] + 360.0 * k / p["dowel_count"])
                         - 360.0 * j / p["motor_bolt_count"] + 180) % 360 - 180)
                    for k in range(p["dowel_count"])
                    for j in range(p["motor_bolt_count"]))
        gap = 2 * r_d * math.sin(math.radians(worst) / 2)
        if gap < need:
            errors.append(
                f"ピン穴と M3 スロットが近すぎます（最小角度差 {worst:.0f}°、"
                f"実距離 {gap:.1f} mm < 必要 {need:.1f} mm）。"
                f"\n    → --dowel-angle を 30 付近（M3 穴の中間）にしてください。")

    # ペグ（＝窪み中心）が M3 スロットや中央逃がし穴と干渉しないか
    r_peg = tile_center(p)
    pr = p["peg_d"] / 2 + 1.5
    if r_peg - pr < p["center_clear_d"] / 2:
        errors.append(f"ペグ（R{r_peg:.1f}）が中央逃がし穴に掛かります。"
                      f"\n    → --pocket-offset を大きくして外へ逃がしてください。")
    if not (r_peg + pr < slot_in or r_peg - pr > slot_out):
        errors.append(
            f"ペグ（R{r_peg:.1f}±{pr:.1f}）が M3 スロット（R{slot_in:.1f}〜{slot_out:.1f}）"
            f"と干渉します。\n    → --pocket-offset を "
            f"{p['pocket_offset'] + (slot_out + pr - r_peg) + 0.5:.0f} 以上にしてください。")

    if p["m4_x"] < 900 and p["m4_x"] + p["m4_clear_d"] / 2 > p["base_d"] / 2 - margin:
        errors.append(f"M4 穴（R{p['m4_x']:.1f}）がベース板（R{p['base_d']/2:.1f}）から"
                      f"はみ出します。--base-d を "
                      f"{2*(p['m4_x']+p['m4_clear_d']/2+margin):.0f} 以上に。")

    if p["base_d"] > p["build_volume"]:
        errors.append(f"外径 {p['base_d']:.0f} mm が造形サイズ "
                      f"{p['build_volume']:.0f} mm を超えます。")

    # 円筒面がベース板に接するので、最大幅は「半径の高さ」に来る。
    # 溝の壁がそこを越えて立っていないとヨー拘束が効かない
    if p["cradle_t"] < p["body_d"] / 2 + 3.0:
        errors.append(
            f"クレードルが薄すぎます（{p['cradle_t']:.0f} mm）。"
            f"円筒の最大幅はベース板から {p['body_d']/2:.0f} mm の高さなので、"
            f"\n    --cradle-t は {p['body_d']/2+3:.0f} mm 以上にしてください。")

    # M4 穴が M3 スロットやピン穴と干渉しないか
    if p["m4_x"] < 900:
        if slot_in - p["m4_clear_d"] / 2 < p["m4_x"] < slot_out + p["m4_clear_d"] / 2:
            errors.append(
                f"M4 穴（R{p['m4_x']:.1f}）が M3 スロット（R{slot_in:.1f}〜{slot_out:.1f}）"
                f"と干渉します。\n    → --pocket-offset を増やして外へ逃がしてください。")

    if errors:
        print("寸法が不正です:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        sys.exit(1)


def radial_slot(r_mid, half_len, half_w, theta, seg):
    """半径方向に伸ばした長穴（時計回り）。ボルト円の不確かさを吸収する。"""
    r0, r1 = r_mid - half_len, r_mid + half_len
    local = arc(r1, 0, half_w, -math.pi / 2, math.pi / 2, seg)
    local += arc(r0, 0, half_w, math.pi / 2, 3 * math.pi / 2, seg)
    c, s = math.cos(theta), math.sin(theta)
    return [(x * c - y * s, x * s + y * c) for x, y in local][::-1]


def circle_hole(cx, cy, d, seg):
    return arc(cx, cy, d / 2, 2 * math.pi, 0.0, seg)


def rounded_rect_hole(w, h, r, seg):
    """角丸長方形の穴（時計回り）。w は x 方向（半径方向）、h は y 方向。"""
    hw, hh = w / 2 - r, h / 2 - r
    pts = []
    pts += arc(hw, hh, r, 0, math.pi / 2, seg)
    pts += arc(-hw, hh, r, math.pi / 2, math.pi, seg)
    pts += arc(-hw, -hh, r, math.pi, 3 * math.pi / 2, seg)
    pts += arc(hw, -hh, r, 3 * math.pi / 2, 2 * math.pi, seg)
    return pts[::-1]


def join_holes(p, seg):
    r = p["join_circle"] / 2
    out = []
    for k in range(p["join_count"]):
        # 角穴の長辺を避けるため 45° ずらして配置する
        th = 2 * math.pi * k / p["join_count"] + math.pi / 4
        out.append(circle_hole(r * math.cos(th), r * math.sin(th), p["join_bolt_d"], seg))
    return out


# ===========================================================================
def rail_bolt_positions(p):
    """側面レールをベース板へ留めるボルトの位置。"""
    y = p["rail_w"] / 2 + p["body_d"] / 2 + p["fit_clearance"]
    return [(p["rail_x0"] + p["rail_bolt_inset"], sy * y) for sy in (1, -1)] + \
           [(p["rail_x1"] - p["rail_bolt_inset"], sy * y) for sy in (1, -1)]


def build_base_levels(p, seg=64):
    """部品1: ベース板＋窪みに嵌るパッド＋中央のペグ。多段押し出しで一体成形。

    ■ なぜ一体にしたか
    別部品のキータイルにすると
      - タイルの固定ネジがベース板の他の穴と干渉する
      - そもそもタイルをベース板に固定しないと**トルクの経路にならない**
    という二つの問題が出た。一体にすれば両方消える。

    ■ トルクの経路
    モータ → ベース板 → パッド（23×23）→ 窪みの壁 → モジュール
    パッドの腕の長さは 11.5mm。2 N·m なら面圧 1.1 MPa 程度で、PLA に十分収まる。
    """
    outline = arc(0, 0, p["base_d"] / 2, 0, 2 * math.pi, seg)
    holes = [circle_hole(0, 0, p["center_clear_d"], seg)]

    for k in range(p["motor_bolt_count"]):
        th = 2 * math.pi * k / p["motor_bolt_count"]
        holes.append(radial_slot(p["motor_bolt_circle"] / 2, p["slot_tolerance"],
                                 p["motor_bolt_d"] / 2, th, seg))
    # 側面レールを留める下穴
    for bx, by in rail_bolt_positions(p):
        holes.append(circle_hole(bx, by, p["rail_bolt_pilot"], seg))

    # 位置決めピン穴（角度関係が分かっている場合のみ）
    if p["dowel_angle"] < 900:
        for k in range(p["dowel_count"]):
            th = math.radians(p["dowel_angle"]) + 2 * math.pi * k / p["dowel_count"]
            r = p["dowel_pcd"] / 2
            holes.append(circle_hole(r * math.cos(th), r * math.sin(th),
                                     p["dowel_d"], seg))

    # モジュール側面の M4 ネジ穴に下から通すバカ穴（位置が分かっている場合のみ）
    if p["m4_x"] < 900:
        holes.append(circle_hole(p["m4_x"], p["m4_y"], p["m4_clear_d"], seg))

    holes = [dedupe(h) for h in holes]

    # 段2: 窪み（23×23）に嵌るパッド。モジュール側の窪みは 3.5mm 深いので
    # 3.3mm にして、円筒面が確実にベース板へ接するようにする
    cx = tile_center(p)
    a = (p["recess_size"] - 2 * p["fit_clearance"]) / 2
    pad = [(cx - a, -a), (cx + a, -a), (cx + a, a), (cx - a, a)]
    # パッドの下に入るモータ用スロットは、パッドにも貫通させて工具が届くようにする
    pad_holes = [h for h in holes if point_in_polygon(centroid(h), pad)]

    # 段3: 窪み中央の Ø6 穴に挿すペグ
    peg = arc(cx, 0, p["peg_d"] / 2, 0, 2 * math.pi, seg)

    return [(dedupe(outline), holes, p["base_t"]),
            (dedupe(pad), pad_holes, p["recess_depth"]),
            (dedupe(peg), [], p["peg_h"])]


def build_rail(p, seg=64):
    """部品2: 側面レール（同じものを2個印刷して左右に置く）。

    モジュールの円筒側面を、最大幅の高さで挟んでヨーを止める。
    後端の不定形状には触れないよう、内端を x=rail_x0 から始めている。
    """
    L = p["rail_x1"] - p["rail_x0"]
    w = p["rail_w"]
    outline = [(0, 0), (L, 0), (L, w), (0, w)]
    holes = [circle_hole(p["rail_bolt_inset"], w / 2, p["rail_bolt_d"], seg),
             circle_hole(L - p["rail_bolt_inset"], w / 2, p["rail_bolt_d"], seg)]
    return dedupe(outline), [dedupe(h) for h in holes]


def build_cradle(p, seg=64):
    """部品2: モジュールの円筒を抱える溝を持つ板。

    円筒面がベース板に接し、溝の壁が円筒の側面（最大幅の位置）を挟んでヨーを止める。

    溝は**外側に開いた U 字**にしてある。円筒 52.5mm のうち捕捉するのは 42mm で、
    エンコーダ側が外へはみ出すため、閉じた角穴だとそこが板に当たってしまう。
    """
    R = p["base_d"] / 2
    w = (p["body_d"] + 2 * p["fit_clearance"]) / 2      # 溝の半幅
    slot_l = p["pocket_length"] + 2 * p["fit_clearance"]
    x_closed = p["pocket_offset"] - slot_l / 2 + w      # 閉じた側の半円の中心
    xm = math.sqrt(R**2 - w**2)                         # 溝が外周と交わる x

    a1 = math.asin(w / R)
    outline = [(xm, w)]
    outline += arc(0, 0, R, a1, 2 * math.pi - a1, seg)  # 外周を一周（溝の口を除く）
    outline.append((xm, -w))
    outline.append((x_closed, -w))
    outline += arc(x_closed, 0, w, -math.pi / 2, -3 * math.pi / 2, seg)  # 閉じた側の半円
    outline.append((x_closed, w))
    outline = dedupe(outline)
    if signed_area(outline) < 0:
        outline = outline[::-1]

    dx = p["pocket_offset"]
    holes = []

    # 結束バンドで上から押さえるためのスロット。角穴の両脇に置く
    tw, tl = p["tie_slot_w"], p["tie_slot_l"]
    gap = w + 4.0 + tw / 2
    for sy in (-1, 1):
        holes.append([(x + dx, y + sy * gap)
                      for x, y in rounded_rect_hole(tl, tw, tw / 2 - 0.01, seg)])

    holes += join_holes(p, seg)
    return outline, [dedupe(h) for h in holes]


def tile_center(p):
    """窪みの中心が、モータ軸から見てどこに来るか。

    窪みはモジュール後端から 8mm 〜 31mm にあるので中心は 19.5mm。
    モジュール後端は溝の内端に合わせる。
    """
    slot_l = p["pocket_length"] + 2 * p["fit_clearance"]
    return p["pocket_offset"] - slot_l / 2 + 19.5


def tile_outline(p, marks, seg=64):
    """タイルの外形。上辺に半円のノッチを marks 個入れて版を識別できるようにする。

    ペグ径だけが違う似た部品が複数あると必ず取り違えるので、
    手に取って区別できる印を形状として持たせる。
    """
    a = (p["recess_size"] - 2 * p["fit_clearance"]) / 2
    r = p["mark_d"] / 2
    pts = [(-a, -a), (a, -a), (a, a)]
    if marks > 0:
        # 上辺を右から左へ進みながら、等間隔にノッチを彫る
        span = 2 * a
        for k in range(marks):
            cx = a - span * (k + 1) / (marks + 1)
            pts.append((cx + r, a))
            pts += arc(cx, a, r, 0, -math.pi, seg)   # 内側（下）へ凹ませる
            pts.append((cx - r, a))
    pts.append((-a, a))
    return dedupe(pts)


def build_key_tile_levels(p, seg=64, peg_d=None, marks=0):
    """部品3: モジュールの窪み（23×23）に嵌る板＋中央の棒。多段押し出し。

    ■ なぜ棒が要るか
    窪みの中央には Ø6〜7 の穴があり、そこへ棒を挿すと位置決めと抜け止めが効く。
    丸棒なので回り止めにはならないが、それはタイルの四辺が担う。

    ■ なぜタイルをベース板にネジ止めするか
    タイルが浮いていると「モジュールとタイル」が結合するだけで、
    **タイルとベース板が結合しない**ため、トルクの経路にならない。
    ネジ2本でベース板に固定して初めて意味を持つ。
    """
    square = tile_outline(p, marks, seg)
    screws = [circle_hole(sx * p["tile_screw_dx"], sy * p["tile_screw_dy"],
                          p["tile_screw_d"], seg)
              for sx, sy in ((1, 1), (1, -1))]
    peg = arc(0, 0, (peg_d if peg_d else 5.8) / 2, 0, 2 * math.pi, seg)
    return [(square, [dedupe(h) for h in screws], p["recess_depth"]),
            (dedupe(peg), [], p["peg_h"])]


def build_spacer(p, seg=64):
    """部品0: 回転ハブとベース板の間に挟むリング。固定ハウジングとの隙間を稼ぐ。"""
    outline = arc(0, 0, p["spacer_od"] / 2, 0, 2 * math.pi, seg)
    holes = [circle_hole(0, 0, p["spacer_id"], seg)]
    for k in range(p["motor_bolt_count"]):
        th = 2 * math.pi * k / p["motor_bolt_count"]
        holes.append(radial_slot(p["motor_bolt_circle"] / 2, p["slot_tolerance"],
                                 p["motor_bolt_d"] / 2, th, seg))
    if p["dowel_angle"] < 900:
        for k in range(p["dowel_count"]):
            th = math.radians(p["dowel_angle"]) + 2 * math.pi * k / p["dowel_count"]
            r = p["dowel_pcd"] / 2
            holes.append(circle_hole(r * math.cos(th), r * math.sin(th),
                                     p["dowel_d"], seg))
    return dedupe(outline), [dedupe(h) for h in holes]


def build_gauge(p, seg=64):
    """試し刷り用：角穴まわりだけを切り出した小片。嵌合の確認を数分で行う。"""
    slot_l = p["pocket_length"] + 2 * p["fit_clearance"]
    slot_w = p["body_d"] + 2 * p["fit_clearance"]
    w, h = slot_l + 16, slot_w + 16
    outline = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
    holes = [rounded_rect_hole(slot_l, slot_w, 1.5, seg)]
    return dedupe(outline), [dedupe(h) for h in holes]


def point_in_polygon(pt, poly):
    """レイキャスティング。多段押し出しで「どの穴が上段の内側にあるか」の判定に使う。"""
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            xin = (x1 - x0) * (y - y0) / (y1 - y0) + x0
            if x < xin:
                inside = not inside
    return inside


def centroid(poly):
    return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))


def emit_stack(name, levels):
    """入れ子になった段を積み上げた立体を出力する。

    levels = [(outline, holes, thickness), ...] を下から順に。
    各段の外形は下段の外形の内側にあり、穴は下段の穴の部分集合であること
    （＝穴は下まで貫通していること）。この制約のもとでは、段の境界の水平面が
    「下段の外形＋（上段の外形を穴として追加）」という穴あきポリゴンで表現でき、
    検証済みの三角形分割器がそのまま使える。

    座ぐりのような「内側に窪む」形状は表現できない。必要なら部品を分ける。
    """
    faces = []
    z = 0.0
    wall_loops = []

    for i, (outline, holes, t) in enumerate(levels):
        upper = levels[i + 1][0] if i + 1 < len(levels) else None

        # この段の下面（最下段のみ、外向き法線は下）
        if i == 0:
            poly, tris = triangulate(bridge_holes(outline, holes))
            for a, b, c in tris:
                faces.append([(*poly[c], z), (*poly[b], z), (*poly[a], z)])

        # 側壁
        for loop in [outline] + holes:
            n = len(loop)
            for k in range(n):
                x0, y0 = loop[k]
                x1, y1 = loop[(k + 1) % n]
                faces.append([(x0, y0, z), (x1, y1, z), (x1, y1, z + t)])
                faces.append([(x0, y0, z), (x1, y1, z + t), (x0, y0, z + t)])

        z += t

        # 上面（または段差面）
        if upper is None:
            poly, tris = triangulate(bridge_holes(outline, holes))
            for a, b, c in tris:
                faces.append([(*poly[a], z), (*poly[b], z), (*poly[c], z)])
        else:
            # 上段に覆われる部分を除いた環状の面。
            # 上段の内側に入る穴は、上段の外形で既に除かれるので二重に数えない
            keep = [h for h in holes if not point_in_polygon(centroid(h), upper)]
            poly, tris = triangulate(bridge_holes(outline, keep + [upper[::-1]]))
            for a, b, c in tris:
                faces.append([(*poly[a], z), (*poly[b], z), (*poly[c], z)])

    bad = check_manifold(faces)
    write_stl(name, faces, name.replace(".stl", ""))
    total_t = sum(t for _, _, t in levels)
    print(f"  {name:26s} 全高 {total_t:4.1f} mm  三角形 {len(faces):5d}  "
          f"{'多様体OK' if not bad else f'★非多様体 {bad} 辺★'}")


def emit(name, outline, holes, thickness):
    merged = bridge_holes(outline, holes)
    poly, tris = triangulate(merged)
    faces = extrude(poly, tris, [outline] + holes, thickness)
    bad = check_manifold(faces)
    write_stl(name, faces, name.replace(".stl", ""))

    area = signed_area(outline) + sum(signed_area(h) for h in holes)
    # 2026/08/21: np.cross() は新しいnumpyで2次元ベクトルを受け付けなくなったため、
    # 2次元外積のスカラー式（ax*by-ay*bx）を直接計算する（値は従来と同じ）。
    def cross2(u, v):
        return u[0] * v[1] - u[1] * v[0]
    check = sum(abs(cross2(np.array(poly[b]) - np.array(poly[a]),
                           np.array(poly[c]) - np.array(poly[a]))) / 2
                for a, b, c in tris)
    print(f"  {name:24s} 厚 {thickness:4.1f} mm  三角形 {len(faces):5d}  "
          f"体積 {area*thickness/1000:5.1f} cm3  "
          f"{'多様体OK' if not bad else f'★非多様体 {bad} 辺★'}  "
          f"面積差 {abs(check-area):.4f}")


def main():
    ap = argparse.ArgumentParser(description="QUBE モジュール取付アダプタの STL 生成")
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k.replace('_','-')}", type=type(v), default=v)
    ap.add_argument("--gauge", action="store_true", help="嵌合確認用の小片も出力する")
    args = ap.parse_args()
    p = {k: getattr(args, k) for k in DEFAULTS}

    validate(p)

    print("QUBE モジュール取付アダプタ")
    print(f"  モータ側 : {p['motor_bolt_count']}×M3 スロット、"
          f"ボルト円 Ø{p['motor_bolt_circle']-2*p['slot_tolerance']:.0f}"
          f"〜Ø{p['motor_bolt_circle']+2*p['slot_tolerance']:.0f} 対応 ★要実測★")
    print(f"  溝(外開き): {p['pocket_length']+2*p['fit_clearance']:.1f} × "
          f"{p['body_d']+2*p['fit_clearance']:.1f} mm（実測寸法から）")
    print(f"  円筒全長 {p['body_length']:.1f} mm のうち {p['pocket_length']:.1f} mm を捕捉。"
          f"残り {p['body_length']-p['pocket_length']:.1f} mm は外側へ張り出す")
    print(f"  結合     : {p['join_count']}×M3、ボルト円 Ø{p['join_circle']:.0f}")
    print()

    o, h = build_spacer(p)
    emit("qube_adapter_spacer.stl", o, h, p["spacer_t"])
    emit_stack("qube_adapter_base.stl", build_base_levels(p))
    o, h = build_rail(p)
    emit("qube_adapter_rail.stl", o, h, p["cradle_t"])
    print("      → **2個印刷**して左右に置く")
    if args.gauge:
        o, h = build_gauge(p)
        emit("qube_adapter_gauge.stl", o, h, 6.0)

    print()
    # 円筒面が接するので、ピボット（円筒軸）はベース板上面から「半径ぶん」上
    pivot_h = p["spacer_t"] + p["base_t"] + p["body_d"] / 2
    print("組み立て:")
    print("  1. ベース板のスロットを**皿ネジ用に座ぐる**（手回しの面取りカッターで可）。")
    print("     ネジ頭が出ているとモジュールの平面が浮く。低頭ネジでも可")
    print("  2. **スペーサリング**を回転ハブ（Ø35）の上に載せる")
    print(f"  3. ベース板を重ね、M3 皿ネジ {int(p['base_t']+p['spacer_t']+5+2)}mm 以上で"
          "モータ出力フランジへ共締め")
    print("     ★固定側ハウジングの 6×M3/Ø50 ではなく、**回転する中央の Ø27** へ")
    print("  4. モジュールをベース板に載せる。")
    print("     **窪みのある面を下向き**にして、一体成形のパッド＋ペグに嵌める")
    print("     エンコーダ側（銀の丸棒が出ている側）を**外向き**にすること")
    print(f"  5. ベース板の R{p['m4_x']:.1f} の穴から下向きの M4 ネジ穴へ、")
    print("     M4（全長19.4/頭4.1）を**下から**ねじ込む ←主たる固定")
    print("  6. 側面レール2個を M3 タッピングでベース板へ留め、円筒を横から挟む")
    print()
    print("寸法の帰結:")
    print(f"  モータ出力面 → ピボット軸 = {pivot_h:.1f} mm"
          f"（スペーサ {p['spacer_t']:.0f} + ベース {p['base_t']:.0f} + 円筒半径 {p['body_d']/2:.0f}）")
    print(f"  固定ハウジングとの隙間   = {p['spacer_t']+1.0:.0f} mm（ハブの出っ張り 1mm 込み）")
    print(f"  → 振子（垂下 121.7mm）の先端はモータ背面より "
          f"{121.7 - 46.0 - pivot_h:.1f} mm 下に来る")
    print(f"  → **固定治具は机から最低 {121.7 - 46.0 - pivot_h + 20:.0f} mm 持ち上げること**"
          "（余裕20mm込み）")
    print("     または机の端に出して振子を宙に垂らす")
    print()
    print("★ 印刷前に --gauge の小片だけ刷って嵌合を確認すること。")
    print("  角穴は実測値で作ってあるが、印刷の収縮と公差は現物でしか分からない。")
    print("★ ピン穴の角度（--dowel-angle）は目視で要確認。")
    print("  Ø4 穴が M3 穴と一直線なら 0、中間なら 30。干渉すればツールが弾く。")
    print("★ レールは同じものを 2 個印刷すること。")


if __name__ == "__main__":
    main()
