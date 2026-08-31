using System.Buffers.Binary;
using System.Diagnostics;

namespace DamiaoCan;

/// <summary>
/// Damiao DM-J4310-2EC V1.1 を1台制御する。
///
/// 2026/8/6 の動作検証で実際に動作を確認した Python 版 (damiao_can.py) を C# へ移植したもの。
///
/// 構成:
///     PC --USB--> USB-CAN Analyzer --CAN_H/CAN_L--> モータ
///                                   24V電源 -------> モータ
///
/// 使い方:
///     using var m = new Motor("COM3", motorId: 4);
///     Console.WriteLine(m.ReadStatus());
///     m.Spin(speed: 0.5, turns: 1.0);   // 0.5 rad/s で1回転
///
/// using を抜けると必ず失能される。途中で例外が出てもモータは安全な状態（非通電）に戻る。
/// </summary>
public sealed class Motor : IDisposable
{
    // ---- モータの特殊コマンド（モータIDあてに送る）----------------------------
    // 注意: これらは 0x104 や 0x204 では受け付けられない。必ずモータIDあてに送ること。

    /// <summary>使能（通電。LEDが緑になる）。</summary>
    private static readonly byte[] CmdEnable = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC];

    /// <summary>失能（非通電。LEDが赤。安全な状態）。</summary>
    private static readonly byte[] CmdDisable = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];

    // ---- パラメータ読み書き用 --------------------------------------------------
    private const uint ParamCanId = 0x7FF;   // パラメータ操作はこのIDあてに送る
    private const byte ParamRead = 0x33;
    private const byte ParamWrite = 0x55;
    private const byte ParamSave = 0xAA;     // フラッシュ保存（電源を切っても設定が残る）

    /// <summary>速度モードの指令はこのIDに送る。</summary>
    private const uint VelocityCommandBase = 0x200;

    private const double RadToDeg = 180.0 / Math.PI;

    private readonly ICanBus _bus;
    private readonly bool _ownsBus;
    private bool _disposed;

    /// <param name="port">Windows なら "COM3"、macOS/Linux なら "/dev/tty.usbserial-xxxx" 等。</param>
    /// <param name="motorId">モータのCAN ID。</param>
    /// <param name="bitrate">CANのビットレート。モータ側は 1Mbps 固定。</param>
    /// <param name="serialBaud">アダプタとPC間のシリアル速度。</param>
    public Motor(string port, int motorId = 4, int bitrate = 1_000_000, int serialBaud = 2_000_000)
    {
        MotorId = motorId;
        _ownsBus = true;

        var bus = new SeeedCanBus(port, bitrate, serialBaud);
        _bus = bus;

        try
        {
            // 重要: 開いた直後に送信すると応答を取りこぼす。必ず待つこと。
            Thread.Sleep(500);
            for (int i = 0; i < 3; i++) _bus.Receive(TimeSpan.FromMilliseconds(50));
        }
        catch
        {
            bus.Dispose();
            throw;
        }
    }

    /// <summary>
    /// 用意済みのCANバスを使う。1本のバスに複数台ぶら下げる場合や、
    /// テストで通信を差し替える場合に使う。
    /// </summary>
    /// <param name="ownsBus">true にすると、この Motor を破棄するときにバスも閉じる。</param>
    public Motor(ICanBus bus, int motorId = 4, bool ownsBus = false)
    {
        ArgumentNullException.ThrowIfNull(bus);
        _bus = bus;
        MotorId = motorId;
        _ownsBus = ownsBus;
    }

    public int MotorId { get; }

    // -- 低レベル ------------------------------------------------------------

    private void Send(uint canId, byte[] data) => _bus.Send(new CanFrame(canId, data));

    // -- 状態の取得 ----------------------------------------------------------

    /// <summary>
    /// モータの状態を読む。動作させない安全な失能コマンドを送って応答を得る方式。
    /// 通信確認にも使える。応答が無ければ null（24V電源が入っていない可能性）。
    /// </summary>
    public MotorFeedback? ReadStatus(int tries = 20)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        for (int i = 0; i < tries; i++)
        {
            Send((uint)MotorId, CmdDisable);
            Thread.Sleep(50);

            CanFrame? msg = _bus.Receive(TimeSpan.FromMilliseconds(300));
            if (msg is { } frame)
            {
                MotorFeedback? feedback = MotorFeedback.Decode(frame.Data);
                if (feedback is not null) return feedback;
            }

            Thread.Sleep(50);
        }
        return null;
    }

    /// <summary>CAN IDを順に叩いて、応答するモータを探す。</summary>
    public Dictionary<int, MotorFeedback?> Scan(int firstId = 1, int lastId = 8)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        var found = new Dictionary<int, MotorFeedback?>();
        for (int id = firstId; id <= lastId; id++)
        {
            Send((uint)id, CmdDisable);

            long deadline = Stopwatch.GetTimestamp() + (long)(0.15 * Stopwatch.Frequency);
            while (Stopwatch.GetTimestamp() < deadline)
            {
                CanFrame? msg = _bus.Receive(TimeSpan.FromMilliseconds(50));
                if (msg is { } frame)
                {
                    found[id] = MotorFeedback.Decode(frame.Data);
                    break;
                }
            }
        }
        return found;
    }

    // -- 使能／失能 ----------------------------------------------------------

    /// <summary>通電する。LEDが緑になる。</summary>
    public void Enable()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        Send((uint)MotorId, CmdEnable);
        Thread.Sleep(100);
    }

    /// <summary>通電を切る。LEDが赤に戻る。安全な状態。</summary>
    public void Disable()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        Send((uint)MotorId, CmdDisable);
        Thread.Sleep(50);
    }

    // -- 制御モードの読み書き ------------------------------------------------

    /// <summary>
    /// パラメータ（レジスタ）を1つ読み、応答の値部4バイトをそのまま返す。読めなければ null。
    ///
    /// 値の型は RID ごとに異なる（<see cref="RegisterId.ControlMode"/> は uint32、
    /// <see cref="RegisterId.PositionMax"/> などは float32）。呼び出し側で解釈すること。
    /// </summary>
    public byte[]? ReadParameter(byte rid)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        byte[] request =
        [
            (byte)(MotorId & 0xFF), (byte)((MotorId >> 8) & 0xFF),
            ParamRead, rid,
            0, 0, 0, 0,
        ];
        Send(ParamCanId, request);

        long deadline = Stopwatch.GetTimestamp() + (long)(0.5 * Stopwatch.Frequency);
        while (Stopwatch.GetTimestamp() < deadline)
        {
            // フィードバックフレームなど、無関係な応答が混ざるので RID まで見て選ぶ
            CanFrame? msg = _bus.Receive(TimeSpan.FromMilliseconds(100));
            if (msg is { } frame
                && frame.Data.Length >= 8
                && frame.Data[2] == ParamRead
                && frame.Data[3] == rid)
            {
                return frame.Data[4..8];
            }
        }
        return null;
    }

    /// <summary>パラメータを単精度浮動小数として読む（P_MAX / V_MAX / T_MAX など）。</summary>
    public float? ReadParameterSingle(byte rid)
        => ReadParameter(rid) is { } raw ? BinaryPrimitives.ReadSingleLittleEndian(raw) : null;

    /// <summary>パラメータを符号なし32bit整数として読む（CTRL_MODE など）。</summary>
    public uint? ReadParameterUInt32(byte rid)
        => ReadParameter(rid) is { } raw ? BinaryPrimitives.ReadUInt32LittleEndian(raw) : null;

    /// <summary>現在の制御モードを読む。読めなければ null。</summary>
    public ControlMode? ReadMode()
        => ReadParameterUInt32(RegisterId.ControlMode) is { } value ? (ControlMode)value : null;

    /// <summary>
    /// パラメータを単精度浮動小数として書き込み、読み返して一致を確認する。
    /// save=true でフラッシュに保存し、電源を切っても維持される。
    ///
    /// 注意: <see cref="RegisterId.TorqueMax"/> などスケーリングに関わる値を変えた場合、
    /// <see cref="MotorScaling"/> 側の定数も必ず同時に合わせること。
    /// ズレたままだと指令トルクが黙って定数倍狂う。
    /// </summary>
    /// <returns>読み返した値が一致すれば true。</returns>
    public bool WriteParameterSingle(byte rid, float value, bool save = true)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        Disable();   // 念のため失能させてから変更する

        byte[] write =
        [
            (byte)(MotorId & 0xFF), (byte)((MotorId >> 8) & 0xFF),
            ParamWrite, rid,
            0, 0, 0, 0,
        ];
        BinaryPrimitives.WriteSingleLittleEndian(write.AsSpan(4, 4), value);
        Send(ParamCanId, write);
        Thread.Sleep(200);

        if (ReadParameterSingle(rid) is not { } readback || Math.Abs(readback - value) > 1e-4f)
            return false;

        if (save) SaveToFlash();
        return true;
    }

    /// <summary>現在のパラメータをフラッシュに保存する。電源を切っても維持される。</summary>
    public void SaveToFlash()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        byte[] saveCommand =
        [
            (byte)(MotorId & 0xFF), (byte)((MotorId >> 8) & 0xFF),
            ParamSave, 0x01,
            0, 0, 0, 0,
        ];
        Send(ParamCanId, saveCommand);
        Thread.Sleep(300);
    }

    /// <summary>
    /// <see cref="MotorScaling"/> の定数がモータの実レジスタと一致するか検証する。
    ///
    /// トルクを出す前に必ず呼ぶこと。ここがズレていると指令トルクが定数倍狂い、
    /// しかも症状が「なんとなく効きが違う」としか出ないため発見が非常に遅れる。
    /// </summary>
    /// <returns>不一致だった項目の説明。すべて一致すれば空。</returns>
    public IReadOnlyList<string> VerifyScaling()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        var problems = new List<string>();
        Check("P_MAX", RegisterId.PositionMax, MotorScaling.PositionMax);
        Check("V_MAX", RegisterId.VelocityMax, MotorScaling.VelocityMax);
        Check("T_MAX", RegisterId.TorqueMax, MotorScaling.TorqueMax);
        return problems;

        void Check(string label, byte rid, double expected)
        {
            float? actual = ReadParameterSingle(rid);
            if (actual is not { } value)
                problems.Add($"{label}: レジスタを読み取れませんでした（RID={rid}）");
            else if (Math.Abs(value - expected) > 1e-3)
                problems.Add($"{label}: MotorScaling={expected} に対しモータの実値={value}");
        }
    }

    /// <summary>
    /// 制御モードを変更する。save=true でフラッシュに保存し、電源を切っても維持される。
    /// 変更が反映されたか読み返して確認し、成功なら true を返す。
    /// </summary>
    public bool SetMode(ControlMode mode, bool save = true)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        Disable();   // 念のため失能させてから変更する

        byte[] write =
        [
            (byte)(MotorId & 0xFF), (byte)((MotorId >> 8) & 0xFF),
            ParamWrite, RegisterId.ControlMode,
            0, 0, 0, 0,
        ];
        BinaryPrimitives.WriteUInt32LittleEndian(write.AsSpan(4, 4), (uint)mode);
        Send(ParamCanId, write);
        Thread.Sleep(200);

        if (ReadMode() != mode) return false;

        if (save)
        {
            byte[] saveCommand =
            [
                (byte)(MotorId & 0xFF), (byte)((MotorId >> 8) & 0xFF),
                ParamSave, 0x01,
                0, 0, 0, 0,
            ];
            Send(ParamCanId, saveCommand);
            Thread.Sleep(300);
        }
        return true;
    }

    // -- 速度モードでの制御（一定速度で回すならこれが一番簡単）--------------

    /// <summary>速度指令を1回送る。速度モードのときのみ有効。</summary>
    public void SetVelocity(double speedRadPerSec)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        Send(VelocityCommandBase + (uint)MotorId, EncodeSingle(speedRadPerSec));
    }

    /// <summary>
    /// 速度モードで回す。安全のため、終了時・例外時ともに必ず速度0→失能する。
    /// </summary>
    /// <param name="speed">回転速度 [rad/s]。正で一方向、負で逆方向。</param>
    /// <param name="durationSeconds">回す秒数。turns を指定した場合は不要。</param>
    /// <param name="turns">回転数。durationSeconds の代わりに指定できる。</param>
    /// <param name="verbose">途中経過を標準出力に表示する。</param>
    /// <param name="cancellationToken">中断すると、その場で減速停止して失能する。</param>
    /// <returns>実際に回った角度 [deg]。</returns>
    public double Spin(
        double speed = 0.5,
        double? durationSeconds = null,
        double? turns = null,
        bool verbose = true,
        CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        if (speed == 0.0)
            throw new ArgumentException("速度に0は指定できません", nameof(speed));

        double duration;
        if (durationSeconds is { } seconds)
        {
            duration = seconds;
        }
        else if (turns is { } turnCount)
        {
            duration = Math.Abs(turnCount * 2 * Math.PI / speed);
        }
        else
        {
            throw new ArgumentException("durationSeconds か turns のどちらかを指定してください");
        }

        byte[] command = EncodeSingle(speed);
        double total = 0.0;   // エンコーダの一周をまたいでも累積できるようにする
        int? previous = null;
        MotorFeedback? last = null;

        try
        {
            Enable();
            if (verbose)
                Console.WriteLine($">> {speed:0.00} rad/s で {duration:0.0} 秒 回転");

            var watch = Stopwatch.StartNew();
            double nextLog = 2.0;

            while (watch.Elapsed.TotalSeconds < duration && !cancellationToken.IsCancellationRequested)
            {
                Send(VelocityCommandBase + (uint)MotorId, command);   // 周期的に送り続ける必要がある

                CanFrame? msg = _bus.Receive(TimeSpan.FromMilliseconds(5));
                if (msg is { } frame)
                {
                    MotorFeedback? feedback = MotorFeedback.Decode(frame.Data);
                    if (feedback is not null)
                    {
                        if (previous is { } previousRaw)
                            total += UnwrapDelta(feedback.PositionRaw, previousRaw);
                        previous = feedback.PositionRaw;
                        last = feedback;
                    }
                }

                double elapsed = watch.Elapsed.TotalSeconds;
                if (verbose && elapsed >= nextLog && last is not null)
                {
                    Console.WriteLine(
                        $"   t={elapsed,4:0.0}s  pos={last.PositionRad:+0.00;-0.00} rad  " +
                        $"温度 {last.DriverTemperature}/{last.RotorTemperature}℃");
                    nextLog += 2.0;
                }

                Thread.Sleep(5);
            }

            if (verbose && cancellationToken.IsCancellationRequested)
                Console.WriteLine("   中断を受け付けました。停止します...");

            // 停止させる
            for (int i = 0; i < 10; i++)
            {
                Send(VelocityCommandBase + (uint)MotorId, EncodeSingle(0.0));
                Thread.Sleep(30);
            }
        }
        finally
        {
            TryDisable();   // 何があっても必ず失能させる
        }

        return RawTotalToDegrees(total);
    }

    /// <summary>
    /// 16bitのエンコーダ値が一周して飛んだ分を補正した、前回からの差分。
    /// 例: 65530 → 5 は「+11」であって「-65525」ではない。
    /// </summary>
    internal static int UnwrapDelta(int current, int previous)
    {
        int delta = current - previous;
        if (delta > 32768) delta -= 65536;
        else if (delta < -32768) delta += 65536;
        return delta;
    }

    /// <summary>累積したエンコーダ値を角度 [deg] に換算する。</summary>
    internal static double RawTotalToDegrees(double totalRaw)
        => totalRaw / 65535 * 2 * MotorScaling.PositionMax * RadToDeg;

    // -- MITモードでの制御（力加減を細かく決めたい場合）---------------------

    /// <summary>
    /// MITモードの指令を1回送る。
    ///
    /// 重要: kp を 0 にすると力が出ず、モータはまったく動かない。
    /// また、遠い目標位置を一度に指定すると急加速する。
    /// 滑らかに動かすには、目標位置を少しずつ進める（<see cref="MitMove"/> を参照）。
    /// </summary>
    public void MitCommand(double position = 0.0, double velocity = 0.0,
                           double kp = 20.0, double kd = 2.0, double torque = 0.0)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        Send((uint)MotorId, BuildMitFrame(position, velocity, kp, kd, torque));
    }

    /// <summary>
    /// 純粋トルク指令。kp=kd=0 とし、フィードフォワードトルクだけを与える。
    /// 倒立振子ではこれを使い、LQR の出力を直接流す。
    ///
    /// README の「MITモードでは kp を 0 にしてはいけない」は位置指令で動かす場合の話。
    /// kp=0 かつ t_ff=0 なら指令トルクが 0 なので動かないのは当然の挙動であり、故障ではない。
    /// ここでは t_ff でトルクを与えるので問題ない。
    ///
    /// 既定引数を持つ <see cref="MitCommand"/> と違い、位置ゲインが紛れ込む余地が無いことを
    /// 名前と引数で保証する。MITモードで kp が残ったまま指令すると、現在位置と p_des=0 の
    /// 差の分だけモータが暴れるため、この区別は安全上重要。
    /// </summary>
    /// <param name="torqueNm">フィードフォワードトルク [N·m]。±<see cref="MotorScaling.TorqueMax"/> で飽和する。</param>
    public void TorqueCommand(double torqueNm)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        Send((uint)MotorId, BuildMitFrame(0.0, 0.0, 0.0, 0.0, torqueNm));
    }

    /// <summary>
    /// フィードバックを1本読む。定周期ループで指令送信の直後に呼ぶ想定。
    /// パラメータ応答（0x7FF）は読み飛ばす。時間内に来なければ null。
    /// </summary>
    public MotorFeedback? ReadFeedback(TimeSpan timeout)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        long deadline = Stopwatch.GetTimestamp() + (long)(timeout.TotalSeconds * Stopwatch.Frequency);
        do
        {
            CanFrame? msg = _bus.Receive(TimeSpan.FromMilliseconds(1));
            if (msg is { } frame && frame.Id != ParamCanId)
            {
                MotorFeedback? feedback = MotorFeedback.Decode(frame.Data);
                if (feedback is not null) return feedback;
            }
        }
        while (Stopwatch.GetTimestamp() < deadline);

        return null;
    }

    /// <summary>
    /// MITモードで、現在位置から deltaRad だけ滑らかに動かす。
    /// 目標位置を5msごとに少しずつ進めることで、速度を制限している。
    /// </summary>
    public void MitMove(double deltaRad, double speed = 0.5, double kp = 20.0, double kd = 2.0,
                        bool verbose = true, CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        if (speed <= 0.0)
            throw new ArgumentException("速度は正の値で指定してください", nameof(speed));

        MotorFeedback? status = ReadStatus()
            ?? throw new InvalidOperationException("モータが応答しません。24V電源を確認してください");

        double start = status.PositionRad;
        double target = start + deltaRad;
        double duration = Math.Abs(deltaRad) / speed;
        double sign = deltaRad > 0 ? 1.0 : -1.0;

        try
        {
            Enable();
            if (verbose)
                Console.WriteLine($">> {start:0.00} → {target:0.00} rad へ {duration:0.0} 秒かけて移動");

            var watch = Stopwatch.StartNew();
            while (true)
            {
                double elapsed = watch.Elapsed.TotalSeconds;
                if (elapsed >= duration || cancellationToken.IsCancellationRequested) break;

                double setpoint = start + sign * speed * elapsed;   // 目標を少しずつ進める
                MitCommand(setpoint, sign * speed, kp, kd);
                _bus.Receive(TimeSpan.FromMilliseconds(1));
                Thread.Sleep(5);
            }

            // 最終位置で保持。中断されたときは、行き過ぎないようその場で保持する
            double holdPosition = cancellationToken.IsCancellationRequested
                ? start + sign * speed * watch.Elapsed.TotalSeconds
                : target;
            for (int i = 0; i < 8; i++)
            {
                MitCommand(holdPosition, 0.0, kp, kd);
                Thread.Sleep(30);
            }
        }
        finally
        {
            TryDisable();
        }
    }

    /// <summary>MITモードの8バイトフレームを、現在の <see cref="MotorScaling"/> の値域で組み立てる。</summary>
    internal static byte[] BuildMitFrame(double position, double velocity, double kp, double kd, double torque)
        => BuildMitFrame(position, velocity, kp, kd, torque,
                         MotorScaling.PositionMax, MotorScaling.VelocityMax, MotorScaling.TorqueMax);

    /// <summary>
    /// MITモードの8バイトフレームを、値域を明示して組み立てる。
    ///
    /// 値域を引数に出しているのは、T_MAX などのレジスタを変更しても
    /// 「Python版と同じバイト列を出す」ことを保証する回帰テストが壊れないようにするため。
    /// 詰め方そのものは値域と独立であり、そこを固定して検証できる必要がある。
    /// </summary>
    internal static byte[] BuildMitFrame(double position, double velocity, double kp, double kd, double torque,
                                         double positionMax, double velocityMax, double torqueMax)
    {
        int p = MotorScaling.FloatToUInt(position, -positionMax, positionMax, 16);
        int v = MotorScaling.FloatToUInt(velocity, -velocityMax, velocityMax, 12);
        int kpi = MotorScaling.FloatToUInt(kp, 0.0, MotorScaling.KpMax, 12);
        int kdi = MotorScaling.FloatToUInt(kd, 0.0, MotorScaling.KdMax, 12);
        int t = MotorScaling.FloatToUInt(torque, -torqueMax, torqueMax, 12);

        return
        [
            (byte)((p >> 8) & 0xFF),
            (byte)(p & 0xFF),
            (byte)((v >> 4) & 0xFF),
            (byte)(((v & 0xF) << 4) | ((kpi >> 8) & 0xF)),
            (byte)(kpi & 0xFF),
            (byte)((kdi >> 4) & 0xFF),
            (byte)(((kdi & 0xF) << 4) | ((t >> 8) & 0xF)),
            (byte)(t & 0xFF),
        ];
    }

    /// <summary>単精度浮動小数をリトルエンディアン4バイトにする。</summary>
    private static byte[] EncodeSingle(double value)
    {
        var buffer = new byte[4];
        BinaryPrimitives.WriteSingleLittleEndian(buffer, (float)value);
        return buffer;
    }

    /// <summary>失能を試みる。後始末の途中なので、失敗しても例外は投げない。</summary>
    private void TryDisable()
    {
        try
        {
            Send((uint)MotorId, CmdDisable);
            Thread.Sleep(50);
        }
        catch (Exception)
        {
            // 通信が既に切れている場合など。ここで例外を投げると本来の原因が隠れる
        }
    }

    /// <summary>必ず失能させてから通信を閉じる。</summary>
    public void Dispose()
    {
        if (_disposed) return;

        TryDisable();
        _disposed = true;
        if (_ownsBus) _bus.Dispose();
    }
}
