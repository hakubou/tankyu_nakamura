// 振子エンコーダのフレーム受信。
//
// マイコン側（8月21日分/pendulum_encoder_tim/pendulum_encoder_tim.ino）が
// 1kHzで送ってくる13バイトのバイナリフレームを読み、最新の値を保持する。
//
// フレーム形式（リトルエンディアン）:
//   sync  uint16  0xA5A5
//   seq   uint16  連番。欠落検出用
//   t_us  uint32  マイコン側のマイクロ秒（PCの時計とは同期していない）
//   count int32   累積カウント（4逓倍・符号付き・剰余なし）
//   crc8  uint8   多項式0x07・初期値0x00

using System.IO.Ports;

namespace PendulumTelemetry;

public readonly struct PendulumFrame
{
    public required ushort Seq { get; init; }
    public required uint McuMicros { get; init; }
    public required int Count { get; init; }

    /// <summary>このフレームをPCが受け取った時刻（Environment.TickCount64、ms）。</summary>
    public required long ReceivedAtMs { get; init; }
}

public sealed class PendulumEncoderReader : IDisposable
{
    private const int FrameSize = 13;
    private const ushort Sync = 0xA5A5;
    private const double DegreesPerCount = 360.0 / 2048.0;

    private readonly SerialPort _port;
    private readonly List<byte> _buffer = new(FrameSize * 4);
    private readonly Lock _lock = new();

    private PendulumFrame? _latest;
    private long _frameCount;
    private long _crcErrorCount;
    private long _dropCount;
    private ushort? _lastSeq;

    public PendulumEncoderReader(string port, int baud = 921600)
    {
        _port = new SerialPort(port, baud)
        {
            ReadTimeout = 200,
        };
        _port.Open();
        _port.DiscardInBuffer();
    }

    /// <summary>最新のフレーム。まだ1つも届いていなければ null。</summary>
    public PendulumFrame? Latest
    {
        get { lock (_lock) return _latest; }
    }

    public long FrameCount => Interlocked.Read(ref _frameCount);
    public long CrcErrorCount => Interlocked.Read(ref _crcErrorCount);
    public long DropCount => Interlocked.Read(ref _dropCount);

    /// <summary>カウント値を度に変換する（ゼロ点の校正はしていない、生の値からの変換のみ）。</summary>
    public static double CountToDegrees(int count) => count * DegreesPerCount;

    /// <summary>
    /// バックグラウンドで受信し続ける。呼び出し側で Task.Run するか、
    /// 専用スレッドで回すことを想定。cancellationToken で止められる。
    /// </summary>
    public void PumpUntilCancelled(CancellationToken cancellationToken)
    {
        byte[] chunk = new byte[256];
        while (!cancellationToken.IsCancellationRequested)
        {
            int n;
            try
            {
                n = _port.Read(chunk, 0, chunk.Length);
            }
            catch (TimeoutException)
            {
                continue;
            }
            catch (OperationCanceledException)
            {
                break;
            }

            if (n <= 0) continue;

            lock (_lock)
            {
                for (int i = 0; i < n; i++) _buffer.Add(chunk[i]);
                ConsumeBuffer();
            }
        }
    }

    // 呼び出し元で _lock を取得済みの前提。
    private void ConsumeBuffer()
    {
        int i = 0;
        while (_buffer.Count - i >= FrameSize)
        {
            ushort sync = (ushort)(_buffer[i] | (_buffer[i + 1] << 8));
            if (sync != Sync)
            {
                i++;
                continue;
            }

            byte[] frame = _buffer.GetRange(i, FrameSize).ToArray();
            byte crcCalculated = Crc8(frame, 0, FrameSize - 1);
            if (crcCalculated != frame[FrameSize - 1])
            {
                _crcErrorCount++;
                i++;
                continue;
            }

            ushort seq = (ushort)(frame[2] | (frame[3] << 8));
            uint tUs = (uint)(frame[4] | (frame[5] << 8) | (frame[6] << 16) | (frame[7] << 24));
            int count = frame[8] | (frame[9] << 8) | (frame[10] << 16) | (frame[11] << 24);

            if (_lastSeq is { } prev)
            {
                int gap = (ushort)(seq - prev);
                if (gap != 1) _dropCount += gap - 1;
            }
            _lastSeq = seq;

            _latest = new PendulumFrame
            {
                Seq = seq,
                McuMicros = tUs,
                Count = count,
                ReceivedAtMs = Environment.TickCount64,
            };
            _frameCount++;

            i += FrameSize;
        }
        if (i > 0) _buffer.RemoveRange(0, i);
    }

    private static byte Crc8(byte[] data, int offset, int length)
    {
        byte c = 0x00;
        for (int i = 0; i < length; i++)
        {
            c ^= data[offset + i];
            for (int b = 0; b < 8; b++)
                c = (byte)((c & 0x80) != 0 ? (c << 1) ^ 0x07 : c << 1);
        }
        return c;
    }

    public void Dispose()
    {
        _port.Dispose();
    }
}
