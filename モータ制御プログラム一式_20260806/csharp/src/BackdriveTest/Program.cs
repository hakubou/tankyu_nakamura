// 逆駆動テスト（Day 1 の合格判定）。
//
// kp=0, kd=0, t_ff=0 を定周期で送り続ける。指令トルクが 0 なので、モータは
// 自分では動かない。この状態で出力軸を手で回して「ほぼ無抵抗」であれば、
// MIT モードによる純粋トルク制御が効いていることが確認できる。
//
//   突っ張る／勝手に戻ろうとする  → MIT モードが効いていない（CTRL_MODE を疑う）
//   ほぼ無抵抗（コギングとギヤの引きずりのみ） → 合格
//
// あわせて制御周期のジッタを計測する。これは Day 2 でシミュレーションに入れる
// 遅延の実測値になる。
//
// このプログラムは定周期ループの原型でもある。CLAUDE.md の方針どおり、
// 周期生成に Thread.Sleep を使わない（Windows の既定タイマ分解能は約 15.6ms で、
// Thread.Sleep(5) が 15ms 眠りうるため）。
//
// 実行:
//   dotnet run --project src/BackdriveTest
//   dotnet run --project src/BackdriveTest -- --port COM3 --hz 200 --seconds 30

using System.Diagnostics;
using System.Runtime;
using DamiaoCan;

const string DefaultPort = "/dev/cu.usbserial-120";
const int DefaultMotorId = 4;

string port = DefaultPort;
int motorId = DefaultMotorId;
double hz = 200.0;
double seconds = 20.0;

for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--port" when i + 1 < args.Length: port = args[++i]; break;
        case "--id" when i + 1 < args.Length: motorId = int.Parse(args[++i]); break;
        case "--hz" when i + 1 < args.Length: hz = double.Parse(args[++i]); break;
        case "--seconds" when i + 1 < args.Length: seconds = double.Parse(args[++i]); break;
        default:
            Console.Error.WriteLine($"不明な引数: {args[i]}");
            return 1;
    }
}

Console.WriteLine("=== 逆駆動テスト ===");
Console.WriteLine($"ポート: {port}   モータCAN ID: {motorId}   周期: {hz:0} Hz   時間: {seconds:0} 秒");
Console.WriteLine();
Console.WriteLine("kp=0, kd=0, t_ff=0 を送り続けます。指令トルクは常にゼロです。");
Console.WriteLine("開始したら出力軸を手で回してください。ほぼ無抵抗なら合格です。");
Console.WriteLine("中断するときは Ctrl+C（必ず失能してから終了します）。");
Console.WriteLine();

using var cancellation = new CancellationTokenSource();
Console.CancelKeyPress += (_, e) =>
{
    e.Cancel = true;      // その場でプロセスを殺さず、失能処理を通す
    cancellation.Cancel();
};

try
{
    using var motor = new Motor(port, motorId);

    // --- 事前確認：通信とモード ---------------------------------------------
    MotorFeedback? status = motor.ReadStatus();
    if (status is null)
    {
        Console.Error.WriteLine("モータが応答しません。24V電源とケーブルを確認してください。");
        return 1;
    }
    Console.WriteLine($"通信OK  {status}");

    ControlMode? mode = motor.ReadMode();
    if (mode != ControlMode.Mit)
    {
        // 速度モードのままだと MIT フレームは受け付けられない。黙って進めない
        Console.Error.WriteLine($"制御モードが MIT ではありません（現在: {mode}）。");
        Console.Error.WriteLine("先に  dotnet run --project src/SetControlMode -- --set mit  を実行してください。");
        return 1;
    }
    Console.WriteLine($"制御モード: {(int)mode} (MIT) — OK");
    Console.WriteLine();

    // --- 定周期ループ --------------------------------------------------------
    long periodTicks = (long)(Stopwatch.Frequency / hz);
    long totalTicks = (long)(seconds * Stopwatch.Frequency);

    // GC 由来のジッタを抑える。制御ループでは停止時間の短さが平均スループットより重要
    GCSettings.LatencyMode = GCLatencyMode.SustainedLowLatency;

    // 統計用バッファは事前確保する。ループ内で List が再確保されると
    // それ自体がジッタ源になり、測定したい対象を測定行為が汚染する
    var jitter = new PeriodStatistics((int)(hz * seconds) + 16);
    int cycles = 0, feedbackReceived = 0;

    motor.Enable();
    Console.WriteLine(">> 開始。軸を手で回してください。");

    long start = Stopwatch.GetTimestamp();
    long next = start;
    long previous = 0;
    long nextLog = start + Stopwatch.Frequency;   // 1秒ごとに表示
    MotorFeedback? last = status;

    try
    {
        while (Stopwatch.GetTimestamp() - start < totalTicks && !cancellation.IsCancellationRequested)
        {
            long now = Stopwatch.GetTimestamp();
            if (previous != 0) jitter.Add((now - previous) * 1000.0 / Stopwatch.Frequency);
            previous = now;

            // 指令トルク 0。モータは自分では動かない
            motor.TorqueCommand(0.0);
            cycles++;

            // 受信待ちは周期の半分まで。取りこぼしてもループは止めない
            MotorFeedback? feedback = motor.ReadFeedback(TimeSpan.FromSeconds(0.5 / hz));
            if (feedback is not null)
            {
                feedbackReceived++;
                last = feedback;

                // エラーが出たら即座に抜ける（過温・過電流などを握りつぶさない）
                if (feedback.Error is not (0 or 1))
                {
                    Console.Error.WriteLine($"\nモータがエラーを報告しました: {DescribeError(feedback.Error)}");
                    break;
                }
            }

            if (now >= nextLog)
            {
                double elapsed = (now - start) / (double)Stopwatch.Frequency;
                Console.WriteLine(
                    $"  t={elapsed,4:0.0}s  位置={last.PositionRad,7:+0.000;-0.000} rad  " +
                    $"速度={last.VelocityRadPerSec,6:+0.00;-0.00} rad/s  " +
                    $"報告トルク={last.TorqueNm,6:+0.000;-0.000} N·m  温度 {last.DriverTemperature}/{last.RotorTemperature}℃");
                nextLog += Stopwatch.Frequency;
            }

            // 周期生成。Thread.Sleep は使わず、次の締切までスピンで待つ
            next += periodTicks;
            SpinUntil(next);
        }
    }
    finally
    {
        motor.Disable();   // 何があっても必ず失能させる
    }

    // --- 結果 ----------------------------------------------------------------
    Console.WriteLine();
    Console.WriteLine("=== 結果 ===");
    Console.WriteLine($"目標周期      : {1000.0 / hz:0.000} ms ({hz:0} Hz)");
    Console.WriteLine($"送信回数      : {cycles}");
    Console.WriteLine($"フィードバック: {feedbackReceived} 回 ({100.0 * feedbackReceived / Math.Max(cycles, 1):0.0}%)");
    Console.WriteLine(jitter.Describe());
    Console.WriteLine();
    Console.WriteLine("判定: 軸を手で回して「ほぼ無抵抗」だったなら Day 1 は合格です。");
    Console.WriteLine("      突っ張る／勝手に戻る場合は MIT モードが効いていません。");
}
catch (IOException ex)
{
    Console.Error.WriteLine($"通信エラー: {ex.Message}");
    return 1;
}
catch (UnauthorizedAccessException ex)
{
    Console.Error.WriteLine($"ポート {port} を使用できません（他のソフトが開いている可能性）: {ex.Message}");
    return 1;
}

Console.WriteLine();
Console.WriteLine("完了。モータは安全な状態（失能）です。");
return 0;

/// <summary>次の締切までスピンで待つ。Thread.Sleep より分解能が高い。</summary>
static void SpinUntil(long deadlineTicks)
{
    var spin = new SpinWait();
    while (Stopwatch.GetTimestamp() < deadlineTicks) spin.SpinOnce();
}

static string DescribeError(int code) => code switch
{
    8 => "過電圧",
    9 => "低電圧",
    0xA => "過電流",
    0xB => "MOS過温",
    0xC => "コイル過温",
    0xD => "通信喪失",
    0xE => "過負荷",
    _ => $"不明({code})",
};

/// <summary>制御周期の実測ばらつき。Day 2 でシミュレーションに入れる遅延の根拠になる。</summary>
file sealed class PeriodStatistics
{
    private readonly List<double> _samples;

    /// <param name="capacity">事前確保するサンプル数。ループ中の再確保を避けるため。</param>
    public PeriodStatistics(int capacity) => _samples = new List<double>(capacity);

    public void Add(double milliseconds) => _samples.Add(milliseconds);

    public string Describe()
    {
        if (_samples.Count == 0) return "周期の統計: サンプルなし";

        double[] sorted = [.. _samples.Order()];
        double mean = _samples.Average();
        double sd = Math.Sqrt(_samples.Sum(x => (x - mean) * (x - mean)) / _samples.Count);

        return $"実測周期      : 平均 {mean:0.000} ms / 標準偏差 {sd:0.000} ms\n" +
               $"              : 最小 {sorted[0]:0.000} / 中央 {sorted[sorted.Length / 2]:0.000} / " +
               $"95% {sorted[(int)(sorted.Length * 0.95)]:0.000} / 最大 {sorted[^1]:0.000} ms";
    }
}
