#!/usr/bin/env python3
"""トルク校正用アームの STL を生成する。

DM-J4310-2EC の出力フランジに取り付け、先端に既知の錘を付けて
トルクスケールを校正するための治具。

■ 設計の要点：左右対称にしてある

アーム自身の重量による負荷トルクを完全に相殺するため。
非対称だと「腕の重心位置 × 腕の質量」という測りにくい量が誤差として乗るが、
対称なら左右で打ち消し合い、**先端に付けた錘の m·g·r だけ**が残る。
校正の基準そのものが正確になるので、この形にする価値は大きい。

錘は片側だけに付ける。反対側の穴は対称性を保つために同じ形で開けてある。

■ ボルト円の不確かさへの対処

出力フランジのボルト円が図面から断定できないため、取り付け穴は
**半径方向に長いスロット**にしてある。BOLT_CIRCLE_D が多少違っても入る。
実測値が分かったら BOLT_CIRCLE_D を書き換えて再生成するのが確実。

使い方:
    python3 generate_calibration_arm.py
    python3 generate_calibration_arm.py --bolt-circle 27 --bolt-count 6
"""

from __future__ import annotations

import argparse
import math
import struct
import sys

import numpy as np

# ---------------------------------------------------------------------------
# 既定パラメータ（すべて mm）
# ---------------------------------------------------------------------------
DEFAULTS = dict(
    thickness=8.0,          # 板厚。PLA でも 100mm 先に 100g で撓みは 0.2mm 未満
    hub_radius=26.0,        # 中央ハブの半径
    center_hole_d=16.0,     # 中央の逃がし穴。出力軸のボスを避ける
    bolt_circle_d=27.0,     # ★要実測★ 出力フランジの M3 ボルト円
    bolt_count=6,           # ★要実測★ ボルト本数
    bolt_d=3.4,             # M3 バカ穴
    slot_tolerance=3.0,     # スロットの半径方向の遊び（±この値ぶん伸ばす）
    arm_half_length=75.0,   # 中心から錘穴までの距離 r
    arm_width=14.0,
    pad_radius=10.0,        # 錘穴まわりの円形パッド
    mass_hole_d=6.5,        # M6 バカ穴。M6 ボルト＋ナットを錘にする
    segments=64,            # 円の分割数
    build_volume=180.0,     # プリンタの造形サイズ [mm]。Bambu Lab A1 mini は 180×180×180
)


# ===========================================================================
# 形状の生成
# ===========================================================================
def arc(cx, cy, r, a0, a1, segments):
    """中心 (cx,cy)、半径 r の円弧上の点列。a0→a1 [rad]。終点は含まない。"""
    n = max(2, int(abs(a1 - a0) / (2 * math.pi) * segments) + 1)
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n),
             cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n)]


def build_outline(p):
    """左右対称な「ハブ＋腕＋パッド」の外形を反時計回りで返す。"""
    hub_r = p["hub_radius"]
    half_w = p["arm_width"] / 2.0
    L = p["arm_half_length"]
    pad_r = p["pad_radius"]
    seg = p["segments"]

    if half_w >= hub_r or half_w >= pad_r:
        sys.exit("アーム幅がハブ半径またはパッド半径以上です。寸法を見直してください。")

    # 腕の直線部が、ハブ円・パッド円と交わる角度
    hub_a = math.asin(half_w / hub_r)     # ハブ円上で y=±half_w となる角度
    pad_a = math.asin(half_w / pad_r)
    hub_x = math.sqrt(hub_r**2 - half_w**2)
    pad_x = math.sqrt(pad_r**2 - half_w**2)

    pts: list[tuple[float, float]] = []

    # ハブ上側（+15.6°付近 → 164.4°付近）
    pts += arc(0, 0, hub_r, hub_a, math.pi - hub_a, seg)
    # 左腕の上辺
    pts.append((-hub_x, half_w))
    pts.append((-(L - pad_x), half_w))
    # 左パッド（内側上 → 外側 → 内側下）
    pts += arc(-L, 0, pad_r, pad_a, 2 * math.pi - pad_a, seg)
    # 左腕の下辺
    pts.append((-(L - pad_x), -half_w))
    pts.append((-hub_x, -half_w))
    # ハブ下側
    pts += arc(0, 0, hub_r, math.pi + hub_a, 2 * math.pi - hub_a, seg)
    # 右腕の下辺
    pts.append((hub_x, -half_w))
    pts.append((L - pad_x, -half_w))
    # 右パッド（内側下 → 外側 → 内側上）
    pts += arc(L, 0, pad_r, math.pi + pad_a, 2 * math.pi - pad_a + math.pi, seg)
    # 右腕の上辺
    pts.append((L - pad_x, half_w))
    pts.append((hub_x, half_w))

    return dedupe(pts)


def validate(p):
    """寸法の整合性を確認する。

    穴同士が重なると「穴あきポリゴン」として不正になり、三角形分割が破綻する。
    実際に slot_tolerance を大きくしすぎてスロットが中央穴に食い込む事故を起こしたので、
    ここで明示的に弾く。
    """
    errors = []
    margin = 0.5   # 最低限残したい肉厚 [mm]

    center_r = p["center_hole_d"] / 2.0
    bolt_r = p["bolt_d"] / 2.0
    slot_inner = p["bolt_circle_d"] / 2.0 - p["slot_tolerance"] - bolt_r
    slot_outer = p["bolt_circle_d"] / 2.0 + p["slot_tolerance"] + bolt_r

    if slot_inner < center_r + margin:
        errors.append(
            f"スロットの内端 R{slot_inner:.1f} が中央穴 R{center_r:.1f} に近すぎます。"
            f"\n    → --slot-tolerance を小さくするか、--center-hole-d を "
            f"{2 * (slot_inner - margin):.0f} 以下にしてください。")

    if slot_outer > p["hub_radius"] - margin:
        errors.append(
            f"スロットの外端 R{slot_outer:.1f} がハブ半径 R{p['hub_radius']:.1f} を超えます。"
            f"\n    → --hub-radius を {slot_outer + margin:.0f} 以上にしてください。")

    if p["mass_hole_d"] / 2.0 > p["pad_radius"] - margin:
        errors.append("錘穴がパッドからはみ出します。--pad-radius を大きくしてください。")

    if p["arm_width"] / 2.0 >= min(p["hub_radius"], p["pad_radius"]):
        errors.append("アーム幅がハブ半径またはパッド半径以上です。")

    if p["arm_half_length"] - p["pad_radius"] <= p["hub_radius"]:
        errors.append("アームが短すぎてパッドがハブに重なります。")

    # 造形サイズに収まるか。斜め置きでも入らないことが多いので素直に長辺で判定する
    total_length = 2 * (p["arm_half_length"] + p["pad_radius"])
    if p["build_volume"] > 0 and total_length > p["build_volume"]:
        errors.append(
            f"全長 {total_length:.0f} mm が造形サイズ {p['build_volume']:.0f} mm を超えます。"
            f"\n    → --arm-half-length を {p['build_volume'] / 2 - p['pad_radius']:.0f} 以下にするか、"
            f"\n      --build-volume でプリンタの造形サイズを指定してください。"
            f"\n    ※ r を縮めると m·g·r が小さくなるので、錘を重くして補うこと"
            f"（摩擦 0.126 N·m の 2〜3 倍が目安）。")

    if errors:
        print("寸法が不正です:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        sys.exit(1)


def build_holes(p):
    """穴（内側境界）を時計回りで返す。"""
    seg = p["segments"]
    holes = []

    # 中央の逃がし穴
    holes.append(arc(0, 0, p["center_hole_d"] / 2.0, 2 * math.pi, 0.0, seg))

    # 錘用の穴（左右対称に開ける。錘は片側だけに付ける）
    for sign in (-1, 1):
        holes.append(arc(sign * p["arm_half_length"], 0,
                         p["mass_hole_d"] / 2.0, 2 * math.pi, 0.0, seg))

    # 取り付けスロット（半径方向に伸ばした長穴）
    r_mid = p["bolt_circle_d"] / 2.0
    half_len = p["slot_tolerance"]
    w = p["bolt_d"] / 2.0
    for k in range(p["bolt_count"]):
        theta = 2 * math.pi * k / p["bolt_count"]
        holes.append(slot(r_mid - half_len, r_mid + half_len, w, theta, seg))

    return [dedupe(h) for h in holes]


def slot(r0, r1, w, theta, segments):
    """半径 r0〜r1、幅 2w の長穴（両端半円）を角度 theta の向きに作る。時計回り。"""
    local = []
    local += arc(r1, 0, w, -math.pi / 2, math.pi / 2, segments)   # 外側の半円
    local += arc(r0, 0, w, math.pi / 2, 3 * math.pi / 2, segments)  # 内側の半円
    c, s = math.cos(theta), math.sin(theta)
    rotated = [(x * c - y * s, x * s + y * c) for x, y in local]
    return rotated[::-1]   # 穴なので時計回りにする


def dedupe(pts, eps=1e-7):
    out = []
    for pt in pts:
        if not out or math.dist(pt, out[-1]) > eps:
            out.append(pt)
    while len(out) > 1 and math.dist(out[0], out[-1]) <= eps:
        out.pop()
    return out


# ===========================================================================
# 穴あきポリゴンの三角形分割（ブリッジ＋耳切り）
# ===========================================================================
def signed_area(poly):
    return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1]
                     - poly[(i + 1) % len(poly)][0] * poly[i][1]
                     for i in range(len(poly)))


def segments_cross(a, b, c, d):
    """線分 ab と cd が（端点を除いて）交差するか。"""
    def orient(p, q, r):
        v = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        return 0 if abs(v) < 1e-12 else (1 if v > 0 else -1)

    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    return o1 != o2 and o3 != o4 and o1 != 0 and o2 != 0 and o3 != 0 and o4 != 0


def bridge_holes(outer, holes):
    """各穴を外形にブリッジして、1本の弱単純ポリゴンにまとめる。"""
    merged = list(outer)
    # 右端が外側にある穴から順に処理すると、ブリッジが交差しにくい
    for hole in sorted(holes, key=lambda h: -max(x for x, _ in h)):
        m_idx = max(range(len(hole)), key=lambda i: hole[i][0])
        M = hole[m_idx]

        # M から見えて、かつ最も近い外形頂点を総当たりで探す
        best, best_d = None, None
        for i, P in enumerate(merged):
            if any(segments_cross(M, P, merged[j], merged[(j + 1) % len(merged)])
                   for j in range(len(merged))):
                continue
            if any(segments_cross(M, P, h[j], h[(j + 1) % len(h)])
                   for h in holes if h is not hole for j in range(len(h))):
                continue
            if any(segments_cross(M, P, hole[j], hole[(j + 1) % len(hole)])
                   for j in range(len(hole))):
                continue
            d = math.dist(M, P)
            if best_d is None or d < best_d:
                best, best_d = i, d

        if best is None:
            sys.exit("穴を外形にブリッジできませんでした。寸法が重なっている可能性があります。")

        rotated = hole[m_idx:] + hole[:m_idx]
        merged = (merged[:best + 1] + rotated + [rotated[0]] + merged[best:])

    return merged


def triangulate(poly):
    """耳切り法。反時計回りの単純ポリゴンを三角形に分割する。

    ブリッジ処理で同じ座標の頂点が複数できるため、
    「三角形の頂点と同一座標の点」は内包判定から除外する必要がある。
    これをやらないと、重複頂点が常に「内側」と判定されて耳が1つも見つからなくなる。
    """
    if signed_area(poly) < 0:
        poly = poly[::-1]

    pts = np.asarray(poly, dtype=np.float64)
    idx = list(range(len(poly)))
    triangles = []
    eps = 1e-9

    while len(idx) > 2:
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = pts[i0], pts[i1], pts[i2]

            if _cross2(b - a, c - a) <= eps:     # 凸でない（耳ではない）
                continue

            others = np.array([j for j in idx if j not in (i0, i1, i2)], dtype=int)
            if others.size and _any_inside(pts[others], a, b, c, eps):
                continue                          # 他の頂点を含むので耳ではない

            triangles.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            sys.exit("耳が見つかりませんでした。形状が自己交差している可能性があります。")

    return poly, triangles


def _cross2(u, v):
    """2次元の外積（スカラー）。NumPy 2.0 で np.cross の2次元用法が非推奨になったため自前で持つ。"""
    return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]


def _any_inside(p, a, b, c, eps):
    """点群 p のうち、三角形 abc の内部にあるものがあるか（頂点と同一座標の点は除外）。"""
    coincident = ((np.abs(p - a).max(axis=1) < 1e-7)
                  | (np.abs(p - b).max(axis=1) < 1e-7)
                  | (np.abs(p - c).max(axis=1) < 1e-7))

    d1 = _cross2(b - a, p - a)
    d2 = _cross2(c - b, p - b)
    d3 = _cross2(a - c, p - c)
    inside = (d1 > eps) & (d2 > eps) & (d3 > eps)   # 厳密に内部のみ
    return bool(np.any(inside & ~coincident))


# ===========================================================================
# 押し出しと STL 出力
# ===========================================================================
def extrude(poly, triangles, loops, thickness):
    """上下面と側壁を作って三角形リストを返す。"""
    t = thickness
    faces = []

    for i0, i1, i2 in triangles:
        top = [(*poly[i], t) for i in (i0, i1, i2)]
        bottom = [(*poly[i], 0.0) for i in (i2, i1, i0)]   # 法線を下向きにする
        faces.append(top)
        faces.append(bottom)

    # 側壁は元の境界ループから作る（ブリッジ由来の縮退辺を持ち込まない）
    for loop in loops:
        n = len(loop)
        for i in range(n):
            x0, y0 = loop[i]
            x1, y1 = loop[(i + 1) % n]
            faces.append([(x0, y0, 0.0), (x1, y1, 0.0), (x1, y1, t)])
            faces.append([(x0, y0, 0.0), (x1, y1, t), (x0, y0, t)])

    return faces


def check_manifold(faces):
    """各辺がちょうど2回、逆向きで現れるか確認する。"""
    edges: dict[tuple, int] = {}
    q = lambda v: (round(v[0], 5), round(v[1], 5), round(v[2], 5))
    for tri in faces:
        for i in range(3):
            a, b = q(tri[i]), q(tri[(i + 1) % 3])
            edges[(a, b)] = edges.get((a, b), 0) + 1
    bad = [e for e, c in edges.items() if c != 1 or edges.get((e[1], e[0]), 0) != 1]
    return len(bad)


def write_stl(path, faces, name="calibration_arm"):
    with open(path, "wb") as f:
        f.write(name.ljust(80, " ").encode("ascii")[:80])
        f.write(struct.pack("<I", len(faces)))
        for tri in faces:
            v = np.array(tri, dtype=np.float64)
            n = np.cross(v[1] - v[0], v[2] - v[0])
            norm = np.linalg.norm(n)
            n = n / norm if norm > 1e-15 else np.zeros(3)
            f.write(struct.pack("<3f", *n))
            for pt in v:
                f.write(struct.pack("<3f", *pt))
            f.write(struct.pack("<H", 0))


# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description="トルク校正用アームの STL を生成する")
    for key, value in DEFAULTS.items():
        ap.add_argument(f"--{key.replace('_', '-')}", type=type(value), default=value)
    ap.add_argument("--out", default="calibration_arm.stl")
    args = ap.parse_args()
    p = {k: getattr(args, k) for k in DEFAULTS}

    validate(p)
    outline = build_outline(p)
    holes = build_holes(p)

    merged = bridge_holes(outline, holes)
    poly, triangles = triangulate(merged)
    faces = extrude(poly, triangles, [outline] + holes, p["thickness"])

    bad = check_manifold(faces)
    write_stl(args.out, faces)

    volume_mm3 = sum(
        np.dot(np.array(t[0]), np.cross(np.array(t[1]), np.array(t[2]))) / 6.0
        for t in faces)

    print(f"出力: {args.out}")
    print(f"  三角形数   : {len(faces)}")
    print(f"  体積       : {volume_mm3 / 1000.0:.1f} cm3"
          f"（PLA 1.24 g/cm3 で約 {volume_mm3 / 1000.0 * 1.24:.0f} g、"
          f"充填 100% 換算）")
    print(f"  全長       : {2 * p['arm_half_length'] + 2 * p['pad_radius']:.0f} mm")
    print(f"  錘穴の半径 : r = {p['arm_half_length']:.0f} mm")
    print(f"  取り付け   : {p['bolt_count']} × M3 スロット、"
          f"ボルト円 Ø{p['bolt_circle_d'] - 2 * p['slot_tolerance']:.0f}"
          f"〜Ø{p['bolt_circle_d'] + 2 * p['slot_tolerance']:.0f} に対応")

    # 校正精度は m·g·r と摩擦の比で決まる。必要な錘の目安を出しておく
    r = p["arm_half_length"] / 1000.0
    print()
    print(f"  錘の目安（摩擦 0.126 N·m に対する比で決まる）:")
    for ratio in (2.0, 3.0):
        target = 0.126 * ratio
        print(f"    m·g·r = {target:.2f} N·m（摩擦の {ratio:.0f} 倍）→ 錘 "
              f"{target / (9.80665 * r) * 1000:.0f} g")
    if bad:
        print(f"  ★警告★ 非多様体の辺が {bad} 本あります。スライサで警告が出るかもしれません。")
    else:
        print("  多様体チェック: OK（すべての辺が2枚の三角形で共有されています）")


if __name__ == "__main__":
    main()


# ===========================================================================
# 錘（3Dプリント製）
# ===========================================================================
def build_weight(target_g, diameter, hole_d, density=1.24, segments=96):
    """指定質量になる円柱の錘を作る。中心に M6 のバカ穴。

    ■ なぜ 3Dプリント製の錘が成立するか

    当初は「摩擦 0.126 N·m の 2〜3倍」＝ 340〜510g を推奨していたが、
    これは摩擦の正逆非対称（0.109 対 0.143）が系統誤差 0.017 N·m として
    残ることを前提にした値だった。
    この非対称量は実測済みなので、解析側で差し引ける。差し引けば必要質量は
    150g 程度まで下がり、PLA（1.24 g/cm3）でも現実的な体積に収まる。

    ■ 必ず秤で量ること

    充填率・フロー・実際の密度で質量は数%変わる。
    設計値ではなく**実測した質量**を --mass に渡すこと。
    ここが狂うと校正結果がそのまま狂う。
    """
    volume_mm3 = target_g / density * 1000.0
    r = diameter / 2.0
    hole_area = math.pi * (hole_d / 2.0) ** 2
    height = volume_mm3 / (math.pi * r**2 - hole_area)

    outline = arc(0, 0, r, 0, 2 * math.pi, segments)
    holes = [arc(0, 0, hole_d / 2.0, 2 * math.pi, 0.0, segments)]
    return dedupe(outline), [dedupe(h) for h in holes], height


def emit_weight(target_g, diameter, hole_d, out, density=1.24):
    outline, holes, height = build_weight(target_g, diameter, hole_d, density)
    merged = bridge_holes(outline, holes)
    poly, tris = triangulate(merged)
    faces = extrude(poly, tris, [outline] + holes, height)
    bad = check_manifold(faces)
    write_stl(out, faces, "calibration_weight")

    area = signed_area(outline) + sum(signed_area(h) for h in holes)
    volume = area * height / 1000.0
    print(f"出力: {out}")
    print(f"  外径 Ø{diameter:.0f} × 高さ {height:.1f} mm、中心穴 Ø{hole_d:.1f}")
    print(f"  体積 {volume:.1f} cm3 → **充填100%の PLA で約 {volume*density:.0f} g**")
    print(f"  重心は中心穴の軸上（＝アームの r=75mm 位置）")
    print(f"  {'多様体OK' if not bad else f'★非多様体 {bad} 辺★'}")
    return volume * density
