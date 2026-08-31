"""
Damiao DM-J4310-2EC V1.1 を Python から CAN 経由で制御するモジュール

2026/8/6 の動作検証で実際に動作を確認した内容をまとめたもの。
メンバーA

必要なもの:
    pip install python-can pyserial

構成:
    PC --USB--> USB-CAN Analyzer --CAN_H/CAN_L--> モータ
                                  24V電源 -------> モータ

使い方:
    from damiao_can import Motor
    with Motor(port="COM3", motor_id=4) as m:
        print(m.read_status())
        m.spin(speed=0.5, duration=12.6)   # 0.5 rad/s で 1 回転
"""

import time
import struct
import math
import can

# ---- モータの特殊コマンド（モータIDあてに送る）----------------------------
# 注意: これらは 0x104 や 0x204 では受け付けられない。必ずモータIDあてに送ること。
CMD_ENABLE  = bytes([0xFF] * 7 + [0xFC])   # 使能（通電。LEDが緑になる）
CMD_DISABLE = bytes([0xFF] * 7 + [0xFD])   # 失能（非通電。LEDが赤。安全な状態）

# ---- パラメータ読み書き用 --------------------------------------------------
PARAM_CAN_ID  = 0x7FF   # パラメータ操作はこのIDあてに送る
PARAM_READ    = 0x33
PARAM_WRITE   = 0x55
PARAM_SAVE    = 0xAA    # フラッシュ保存（電源を切っても設定が残る）
RID_CTRL_MODE = 10      # 制御モードのパラメータ番号

MODE_MIT      = 1
MODE_POS_VEL  = 2
MODE_VELOCITY = 3
MODE_NAMES = {1: "MIT", 2: "POS-VEL", 3: "VELOCITY"}

# ---- MITモードの値域（フレームに詰めるときの換算に使う）--------------------
P_MAX, V_MAX, T_MAX = 12.5, 30.0, 10.0
KP_MAX, KD_MAX = 500.0, 5.0


def _float_to_uint(x, lo, hi, bits):
    """浮動小数を、指定ビット数の整数に線形変換する"""
    x = max(min(x, hi), lo)
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))


def _uint_to_float(x, lo, hi, bits):
    """上の逆変換"""
    return x / ((1 << bits) - 1) * (hi - lo) + lo


class Motor:
    """DM-J4310-2EC V1.1 を1台制御する"""

    def __init__(self, port="COM3", motor_id=4, bitrate=1_000_000, serial_baud=2_000_000):
        self.motor_id = motor_id
        self.bus = can.Bus(
            interface="seeedstudio",   # USB-CAN Analyzer 用のバックエンド
            channel=port,
            bitrate=bitrate,           # CANのボーレート。モータ側は 1Mbps 固定
            baudrate=serial_baud,      # アダプタとPC間のシリアル速度
            frame_type="STD",          # 標準フレーム
            operation_mode="normal",
        )
        # 重要: 開いた直後に送信すると応答を取りこぼす。必ず待つこと。
        time.sleep(0.5)
        for _ in range(3):
            self.bus.recv(timeout=0.05)

    # -- 後始末 --------------------------------------------------------------
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        """必ず失能させてから閉じる"""
        try:
            self.disable()
        except Exception:
            pass
        self.bus.shutdown()

    # -- 低レベル ------------------------------------------------------------
    def _send(self, can_id, data):
        self.bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))

    @staticmethod
    def _decode(data):
        """モータからのフィードバックフレームを解釈する"""
        if len(data) < 8:
            return None
        return {
            "id":       data[0] & 0x0F,
            "err":      (data[0] >> 4) & 0x0F,   # 0=正常、8〜E=故障、1=使能中
            "pos_raw":  (data[1] << 8) | data[2],
            "vel_raw":  (data[3] << 4) | (data[4] >> 4),
            "tau_raw":  ((data[4] & 0x0F) << 8) | data[5],
            "temp_mos": data[6],                  # ドライバ温度
            "temp_rot": data[7],                  # モータ内部温度
        }

    # -- 状態の取得 ----------------------------------------------------------
    def read_status(self, tries=20):
        """
        モータの状態を読む。動作させない安全な DISABLE を送って応答を得る方式。
        通信確認にも使える。応答が無ければ None（電源が入っていない可能性）。
        """
        for _ in range(tries):
            self._send(self.motor_id, CMD_DISABLE)
            time.sleep(0.05)
            msg = self.bus.recv(timeout=0.3)
            if msg:
                fb = self._decode(msg.data)
                if fb:
                    fb["pos_rad"] = _uint_to_float(fb["pos_raw"], -P_MAX, P_MAX, 16)
                    return fb
            time.sleep(0.05)
        return None

    def scan(self, id_range=range(1, 9)):
        """CAN IDを順に叩いて、応答するモータを探す"""
        found = {}
        for cid in id_range:
            self._send(cid, CMD_DISABLE)
            t0 = time.time()
            while time.time() - t0 < 0.15:
                msg = self.bus.recv(timeout=0.05)
                if msg:
                    found[cid] = self._decode(msg.data)
                    break
        return found

    # -- 使能／失能 ----------------------------------------------------------
    def enable(self):
        """通電する。LEDが緑になる"""
        self._send(self.motor_id, CMD_ENABLE)
        time.sleep(0.1)

    def disable(self):
        """通電を切る。LEDが赤に戻る。安全な状態"""
        self._send(self.motor_id, CMD_DISABLE)
        time.sleep(0.05)

    # -- 制御モードの読み書き ------------------------------------------------
    def read_mode(self):
        """現在の制御モードを読む（1=MIT, 2=位置速度, 3=速度）"""
        d = bytes([self.motor_id & 0xFF, (self.motor_id >> 8) & 0xFF,
                   PARAM_READ, RID_CTRL_MODE, 0, 0, 0, 0])
        self._send(PARAM_CAN_ID, d)
        t0 = time.time()
        while time.time() - t0 < 0.5:
            msg = self.bus.recv(timeout=0.1)
            if msg and len(msg.data) >= 8 \
               and msg.data[2] == PARAM_READ and msg.data[3] == RID_CTRL_MODE:
                return struct.unpack("<I", msg.data[4:8])[0]
        return None

    def set_mode(self, mode, save=True):
        """
        制御モードを変更する。save=True でフラッシュに保存し、電源を切っても維持される。
        変更が反映されたか読み返して確認し、成功なら True を返す。
        """
        self.disable()   # 念のため失能させてから変更する
        d = bytes([self.motor_id & 0xFF, (self.motor_id >> 8) & 0xFF,
                   PARAM_WRITE, RID_CTRL_MODE]) + struct.pack("<I", mode)
        self._send(PARAM_CAN_ID, d)
        time.sleep(0.2)

        if self.read_mode() != mode:
            return False

        if save:
            d = bytes([self.motor_id & 0xFF, (self.motor_id >> 8) & 0xFF,
                       PARAM_SAVE, 0x01, 0, 0, 0, 0])
            self._send(PARAM_CAN_ID, d)
            time.sleep(0.3)
        return True

    # -- 速度モードでの制御（一定速度で回すならこれが一番簡単）--------------
    def set_velocity(self, speed_rad_s):
        """速度指令を1回送る。速度モードのときのみ有効"""
        self._send(0x200 + self.motor_id, struct.pack("<f", speed_rad_s))

    def spin(self, speed=0.5, duration=None, turns=None, verbose=True):
        """
        速度モードで回す。

            speed    : 回転速度 [rad/s]。正で一方向、負で逆方向
            duration : 回す秒数。turns を指定した場合は不要
            turns    : 回転数。duration の代わりに指定できる
            戻り値   : 実際に回った角度 [deg]

        安全のため、終了時・例外時ともに必ず速度0→失能する。
        """
        if duration is None:
            if turns is None:
                raise ValueError("duration か turns のどちらかを指定してください")
            duration = abs(turns * 2 * math.pi / speed)

        cmd = struct.pack("<f", speed)
        total = 0.0      # エンコーダの一周をまたいでも累積できるようにする
        prev = None
        last = None

        try:
            self.enable()
            if verbose:
                print(">> %.2f rad/s で %.1f 秒 回転" % (speed, duration))

            t0 = time.time()
            next_log = 2.0
            while time.time() - t0 < duration:
                self._send(0x200 + self.motor_id, cmd)   # 周期的に送り続ける必要がある
                msg = self.bus.recv(timeout=0.005)
                if msg:
                    fb = self._decode(msg.data)
                    if fb:
                        if prev is not None:
                            d = fb["pos_raw"] - prev
                            if d > 32768:    # 一周して値が飛んだ分を補正
                                d -= 65536
                            elif d < -32768:
                                d += 65536
                            total += d
                        prev = fb["pos_raw"]
                        last = fb

                elapsed = time.time() - t0
                if verbose and elapsed >= next_log and last:
                    print("   t=%4.1fs  pos=%+.2f rad  温度 %d/%d℃"
                          % (elapsed,
                             _uint_to_float(last["pos_raw"], -P_MAX, P_MAX, 16),
                             last["temp_mos"], last["temp_rot"]))
                    next_log += 2.0

                time.sleep(0.005)

            # 停止させる
            for _ in range(10):
                self._send(0x200 + self.motor_id, struct.pack("<f", 0.0))
                time.sleep(0.03)
        finally:
            self.disable()   # 何があっても必ず失能させる

        return total / 65535 * 2 * P_MAX * 57.2958

    # -- MITモードでの制御（力加減を細かく決めたい場合）---------------------
    def _mit_frame(self, pos, vel, kp, kd, torque):
        pi = _float_to_uint(pos, -P_MAX, P_MAX, 16)
        vi = _float_to_uint(vel, -V_MAX, V_MAX, 12)
        kpi = _float_to_uint(kp, 0.0, KP_MAX, 12)
        kdi = _float_to_uint(kd, 0.0, KD_MAX, 12)
        ti = _float_to_uint(torque, -T_MAX, T_MAX, 12)
        return bytes([
            (pi >> 8) & 0xFF, pi & 0xFF,
            (vi >> 4) & 0xFF, ((vi & 0xF) << 4) | ((kpi >> 8) & 0xF), kpi & 0xFF,
            (kdi >> 4) & 0xFF, ((kdi & 0xF) << 4) | ((ti >> 8) & 0xF), ti & 0xFF,
        ])

    def mit_command(self, pos=0.0, vel=0.0, kp=20.0, kd=2.0, torque=0.0):
        """
        MITモードの指令を1回送る。

        重要: kp を 0 にすると力が出ず、モータはまったく動かない。
        また、遠い目標位置を一度に指定すると急加速する。
        滑らかに動かすには、目標位置を少しずつ進める（下の mit_move を参照）。
        """
        self._send(self.motor_id, self._mit_frame(pos, vel, kp, kd, torque))

    def mit_move(self, delta_rad, speed=0.5, kp=20.0, kd=2.0, verbose=True):
        """
        MITモードで、現在位置から delta_rad だけ滑らかに動かす。
        目標位置を5msごとに少しずつ進めることで、速度を制限している。
        """
        st = self.read_status()
        if st is None:
            raise RuntimeError("モータが応答しません。24V電源を確認してください")
        start = st["pos_rad"]
        target = start + delta_rad
        duration = abs(delta_rad) / speed
        sign = 1.0 if delta_rad > 0 else -1.0

        try:
            self.enable()
            if verbose:
                print(">> %.2f → %.2f rad へ %.1f 秒かけて移動" % (start, target, duration))
            t0 = time.time()
            while True:
                elapsed = time.time() - t0
                if elapsed >= duration:
                    break
                setpoint = start + sign * speed * elapsed   # 目標を少しずつ進める
                self.mit_command(setpoint, sign * speed, kp, kd)
                self.bus.recv(timeout=0.001)
                time.sleep(0.005)

            for _ in range(8):   # 最終位置で保持
                self.mit_command(target, 0.0, kp, kd)
                time.sleep(0.03)
        finally:
            self.disable()
