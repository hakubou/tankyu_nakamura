"""
2026/8/6 に実際に成功した動作: 速度モードで 0.5 rad/s、12.6秒でちょうど1回転
実測結果: 359.5〜359.6度（1.00回転）、速度のばらつきはほぼ無し

実行前の確認:
  1. 24V電源が入っていること（これを忘れて動かず、原因究明に時間を使った）
  2. モータが固定されていること
  3. 出力軸の回転範囲に手や物が無いこと
  4. 電源スイッチにすぐ手が届くこと

実行:  py demo_1回転.py
"""

import math
from damiao_can import Motor, MODE_VELOCITY, MODE_NAMES

PORT     = "COM3"    # デバイスマネージャーの「ポート(COMとLPT)」で確認できる
MOTOR_ID = 4         # 本機のCAN ID
SPEED    = 0.5       # rad/s（約3rpm）ゆっくり
TURNS    = 1.0       # 回転数

with Motor(port=PORT, motor_id=MOTOR_ID) as m:

    # --- 1. 通信できるか確認する（この時点ではモータは動かない）-------------
    status = m.read_status()
    if status is None:
        print("モータが応答しません。24V電源とケーブルを確認してください。")
        raise SystemExit(1)

    print("通信OK  CAN ID=%d  位置=%+.2f rad  エラー=%d  温度 %d/%d℃"
          % (status["id"], status["pos_rad"], status["err"],
             status["temp_mos"], status["temp_rot"]))

    # --- 2. 制御モードを確認し、速度モードでなければ変更する ---------------
    mode = m.read_mode()
    print("制御モード: %s (%s)" % (mode, MODE_NAMES.get(mode, "不明")))

    if mode != MODE_VELOCITY:
        print("速度モードへ変更します...")
        if m.set_mode(MODE_VELOCITY):
            print("変更してフラッシュに保存しました")
        else:
            print("モード変更に失敗しました")
            raise SystemExit(1)

    # --- 3. 回す ------------------------------------------------------------
    print()
    print("=== %.1f 回転させます（約 %.1f 秒）==="
          % (TURNS, TURNS * 2 * math.pi / SPEED))
    moved = m.spin(speed=SPEED, turns=TURNS)

    print()
    print("結果: %.1f 度（%.2f 回転）動きました" % (moved, moved / 360.0))

# with を抜けると自動的に失能され、通信も閉じられる
print("完了。モータは安全な状態（失能）です。")
