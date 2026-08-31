using System.Buffers.Binary;
using System.Diagnostics;
using System.IO.Ports;
using System.Runtime.CompilerServices;

[assembly: InternalsVisibleTo("DamiaoCan.Tests")]

namespace DamiaoCan;

public enum CanFrameType
{
    Standard = 0x01,
    Extended = 0x02,
}

public enum CanOperationMode
{
    Normal = 0x00,
    Loopback = 0x01,
    Silent = 0x02,
    LoopbackAndSilent = 0x03,
}

/// <summary>
/// USB-CAN Analyzer V8.00（Seeed 互換）を仮想COMポート経由で扱うCANバス。
///
/// このアダプタは CH340 でシリアルポートとして見えるだけで、CANフレームは
/// 独自のシリアルプロトコルで包まれている。Python版では python-can の
/// seeedstudio バックエンドが担っていた部分で、.NET には相当品が無いため
/// ここで同じプロトコルを実装している。
///
/// パケット形式:
///   初期化   AA 55 12 [速度] [フレーム種別] [フィルタ4B] [マスク4B] [動作モード] 01 00×4 [CRC]
///   送受信   AA [0xC0|拡張&lt;&lt;5|RTR&lt;&lt;4|DLC] [ID 2B(標準) or 4B(拡張) リトルエンディアン] [データ] 55
///   状態応答 AA 55 [18バイト]
/// CRC は先頭2バイトを除く総和の下位8bit。
/// </summary>
public sealed class SeeedCanBus : ICanBus
{
    /// <summary>CANのビットレートとアダプタ側コードの対応表。</summary>
    private static readonly Dictionary<int, byte> BitrateCodes = new()
    {
        [1_000_000] = 0x01,
        [800_000] = 0x02,
        [500_000] = 0x03,
        [400_000] = 0x04,
        [250_000] = 0x05,
        [200_000] = 0x06,
        [125_000] = 0x07,
        [100_000] = 0x08,
        [50_000] = 0x09,
        [20_000] = 0x0A,
        [10_000] = 0x0B,
        [5_000] = 0x0C,
    };

    private const byte StartByte = 0xAA;
    private const byte EndByte = 0x55;

    /// <summary>フレームを1バイト読む関数。期限までに来なければ -1 を返す。</summary>
    internal delegate int ByteReader(long deadlineTicks);

    private readonly SerialPort _port;
    private readonly long _serialTimeoutTicks;

    // 受信は必ずまとめ読みしてここに貯め、パースはこのバッファから行う。
    // SerialPort.ReadByte() は macOS で1回あたり約1.2ms かかり、
    // 1バイトずつ読むと13バイトのフレーム1本に15msも取られて制御周期が保てない。
    // Read(buffer,0,n) のまとめ読みなら同じ13バイトが 0.001ms で取れる。
    private readonly byte[] _receiveBuffer = new byte[4096];
    private int _receiveHead;
    private int _receiveTail;

    private bool _disposed;

    /// <param name="portName">Windows なら "COM3"、macOS/Linux なら "/dev/tty.usbserial-xxxx" 等。</param>
    /// <param name="bitrate">CANのビットレート [bps]。モータ側は 1Mbps 固定。</param>
    /// <param name="serialBaud">アダプタとPC間のシリアル速度。</param>
    /// <param name="serialTimeoutMs">フレーム途中で次のバイトを待つ上限 [ms]。</param>
    public SeeedCanBus(
        string portName,
        int bitrate = 1_000_000,
        int serialBaud = 2_000_000,
        CanFrameType frameType = CanFrameType.Standard,
        CanOperationMode operationMode = CanOperationMode.Normal,
        int serialTimeoutMs = 100)
    {
        if (string.IsNullOrWhiteSpace(portName))
            throw new ArgumentException("シリアルポート名を指定してください", nameof(portName));

        _serialTimeoutTicks = serialTimeoutMs * Stopwatch.Frequency / 1000;

        // ポートを開く前に組み立てておく（未対応ビットレートならここで弾く）
        byte[] initFrame = BuildInitFrame(bitrate, frameType, operationMode);

        _port = new SerialPort(portName, serialBaud, Parity.None, 8, StopBits.One)
        {
            Handshake = Handshake.None,
            ReadTimeout = serialTimeoutMs,
            WriteTimeout = serialTimeoutMs,
        };

        try
        {
            _port.Open();
        }
        catch (Exception ex) when (ex is UnauthorizedAccessException or IOException or ArgumentException)
        {
            _port.Dispose();
            throw new IOException(
                $"シリアルポート {portName} を開けませんでした。ポート名とCH340ドライバを確認してください。", ex);
        }

        _port.Write(initFrame, 0, initFrame.Length);
    }

    /// <summary>アダプタの初期化フレーム（20バイト）を組み立てる。</summary>
    internal static byte[] BuildInitFrame(int bitrate, CanFrameType frameType, CanOperationMode operationMode)
    {
        if (!BitrateCodes.TryGetValue(bitrate, out byte bitrateCode))
            throw new ArgumentException(
                $"対応していないビットレートです: {bitrate}。指定できる値: {string.Join(", ", BitrateCodes.Keys)}",
                nameof(bitrate));

        var msg = new byte[20];
        msg[0] = StartByte;
        msg[1] = 0x55;                      // 初期化フレームの開始バイト2
        msg[2] = 0x12;                      // 初期化メッセージID
        msg[3] = bitrateCode;
        msg[4] = (byte)frameType;
        // msg[5..9]  フィルタID  = 0（全通過）
        // msg[9..13] マスクID    = 0（全通過）
        msg[13] = (byte)operationMode;
        msg[14] = 0x01;                     // Windows版アプリの「Send once」に相当
        // msg[15..19] 手動ビットレート設定。詳細不明だが 0 でよい

        byte crc = 0;
        for (int i = 2; i < 19; i++) crc += msg[i];
        msg[19] = crc;

        return msg;
    }

    /// <summary>CANフレームを、アダプタへ送るシリアルパケットに包む。</summary>
    internal static byte[] BuildSerialPacket(CanFrame frame)
    {
        int idLength = frame.IsExtendedId ? 4 : 2;
        var buffer = new byte[1 + 1 + idLength + frame.Dlc + 1];

        byte type = 0xC0;
        if (frame.IsExtendedId) type += 1 << 5;
        if (frame.IsRemoteFrame) type += 1 << 4;
        type += (byte)frame.Dlc;

        buffer[0] = StartByte;
        buffer[1] = type;
        if (frame.IsExtendedId)
            BinaryPrimitives.WriteUInt32LittleEndian(buffer.AsSpan(2, 4), frame.Id);
        else
            BinaryPrimitives.WriteUInt16LittleEndian(buffer.AsSpan(2, 2), (ushort)frame.Id);
        frame.Data.CopyTo(buffer, 2 + idLength);
        buffer[^1] = EndByte;

        return buffer;
    }

    /// <summary>CANフレームを1本送る。</summary>
    public void Send(CanFrame frame)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        byte[] packet = BuildSerialPacket(frame);
        _port.Write(packet, 0, packet.Length);
    }

    /// <summary>
    /// CANフレームを1本受け取る。指定時間内に受け取れなければ null。
    /// アダプタの状態応答パケットは読み飛ばす。
    /// </summary>
    public CanFrame? Receive(TimeSpan timeout)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        long deadline = Stopwatch.GetTimestamp() + (long)(timeout.TotalSeconds * Stopwatch.Frequency);
        return ReceiveCore(ReadByte, deadline, _serialTimeoutTicks);
    }

    /// <summary>
    /// 受信パケットの解釈。シリアルの読み出しを <paramref name="read"/> に委ねているので、
    /// 実機が無くてもテストできる。
    /// </summary>
    /// <param name="read">1バイト読む関数。期限までに来なければ -1。</param>
    /// <param name="deadline">受信全体の期限（Stopwatch のタイムスタンプ）。</param>
    /// <param name="serialTimeoutTicks">開始バイトを見つけた後、フレームを読み切るまでの猶予。</param>
    /// <param name="maxScanBytes">
    /// 1回の呼び出しで読み進める上限。バッファにデータがある限り読めてしまうので、
    /// ゴミが延々と続いたときに制御周期を食い潰さないよう区切っている。
    /// </param>
    internal static CanFrame? ReceiveCore(
        ByteReader read, long deadline, long serialTimeoutTicks, int maxScanBytes = 1024)
    {
        int scanned = 0;

        while (true)
        {
            if (++scanned > maxScanBytes) return null;   // 今回は諦めて次の周期に回す

            int first = read(deadline);
            if (first < 0) return null;
            if (first != StartByte) continue;   // 同期を取り直す

            // 開始バイトを見つけた後は、フレームを読み切るまで少し待つ
            long frameDeadline = Stopwatch.GetTimestamp() + serialTimeoutTicks;

            int second = read(frameDeadline);
            if (second < 0) return null;

            if (second == EndByte)
            {
                ReadExactly(read, 18, frameDeadline);   // 状態応答。中身は使わないので捨てる
                continue;
            }

            int dlc = second & 0x0F;
            bool isExtended = (second & 0x20) != 0;
            bool isRemote = (second & 0x10) != 0;
            if (dlc > 8) continue;                      // 同期ずれ。開始バイトから探し直す

            byte[]? idBytes = ReadExactly(read, isExtended ? 4 : 2, frameDeadline);
            if (idBytes is null) return null;

            byte[]? data = ReadExactly(read, dlc, frameDeadline);
            if (data is null) return null;

            int end = read(frameDeadline);
            if (end < 0) return null;
            if (end != EndByte) continue;               // 終端が合わない。壊れたフレームとして捨てる

            uint id = isExtended
                ? BinaryPrimitives.ReadUInt32LittleEndian(idBytes)
                : BinaryPrimitives.ReadUInt16LittleEndian(idBytes);

            return new CanFrame(id, data, isExtended, isRemote);
        }
    }

    /// <summary>受信バッファに溜まった分を捨てる。</summary>
    public void FlushInput()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        _port.DiscardInBuffer();
        _receiveHead = _receiveTail = 0;
    }

    /// <summary>
    /// 1バイト読む。期限までに来なければ -1。
    /// 貯めてあるぶんが尽きたときだけ、シリアルから可能な限りまとめて読み込む。
    /// </summary>
    private int ReadByte(long deadline)
    {
        var spin = new SpinWait();
        while (true)
        {
            if (_receiveHead < _receiveTail) return _receiveBuffer[_receiveHead++];

            int available = _port.BytesToRead;
            if (available > 0)
            {
                try
                {
                    // BytesToRead で存在を確認済みなので、この Read はブロックしない
                    _receiveTail = _port.Read(_receiveBuffer, 0, Math.Min(available, _receiveBuffer.Length));
                    _receiveHead = 0;
                }
                catch (TimeoutException)
                {
                    _receiveHead = _receiveTail = 0;
                    return -1;
                }
                continue;
            }

            if (Stopwatch.GetTimestamp() >= deadline) return -1;
            spin.SpinOnce();
        }
    }

    /// <summary>指定バイト数を読み切る。期限までに揃わなければ null。</summary>
    private static byte[]? ReadExactly(ByteReader read, int count, long deadline)
    {
        var buffer = new byte[count];
        for (int i = 0; i < count; i++)
        {
            int b = read(deadline);
            if (b < 0) return null;
            buffer[i] = (byte)b;
        }
        return buffer;
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;

        try
        {
            if (_port.IsOpen) _port.Close();
        }
        catch (IOException)
        {
            // 閉じる途中のエラーは握りつぶす（既に抜かれている場合など）
        }
        _port.Dispose();
    }
}
