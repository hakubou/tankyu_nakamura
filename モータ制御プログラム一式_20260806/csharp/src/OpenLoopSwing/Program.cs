// 振子エンコーダ**なし**で行う開ループ加振。
//
// ■ 何ができて、何ができないか
//
// できない: 本来のスイングアップ。エネルギー整形則は振子角 α を必要とするため、
//           エンコーダが無い限り実装できない。
//
// できる  : **共振加振**。アームを振子の固有振動数で往復させると、
//           フィードバック無しでも振子にエネルギーが溜まって振れ上がる。
//           ブランコを漕ぐのと同じ原理で、センサを一切使わない。
//
// ■ もう一つの狙い：エンコーダ無しで固有振動数を測る
//
// 振子が振れると、その反力がアームに返ってくる。位置制御でアームを動かしていれば、
// 共振点では**モータ側の負荷が増える**。つまりモータ自身のフィードバックだけで
// 共振周波数を検出できる。振子エンコーダは要らない。
//
// これは機構の検証にもなる。アダプタが滑っていれば所定の振幅で動かず、
// 共振も出ない。
//
// ■ 制御方式
//
// LQR とは違い、ここでは MIT モードの位置制御（kp, kd を使う）でアームを
// 正弦波追従させる。倒立制御では kp=kd=0 の純トルクを使うが、
// この試験ではアームの軌道そのものを与えたいので位置制御が適している。
//
// 実行:
//   dotnet run --project src/OpenLoopSwing -- --mode sweep
//   dotnet run --project src/OpenLoopSwing -- --mode pump --freq 1.75

using System.Diagnostics;
using System.Runtime;
using DamiaoCan;

const string DefaultPort = "/dev/cu.usbserial-120";
const int DefaultMotorId = 4;

string port = DefaultPort;
int motorId = DefaultMotorId;
string mode = "sweep";
double amplitudeDeg = 7.0;     // アームの振幅 [deg]
double kp = 8.0;               // 位置ゲイン [N·m/rad]
double kd = 0.4;               // 速度ゲイン [N·m·s/rad]
double hz = 300.0;             // 制御周期
double freq = 1.75;            // pump モードの加振周波数 [Hz]
double freqLow = 0.8, freqHigh = 3.5;
int cyclesPerStep = 8;         // sweep で各周波数に留まる周期数
int steps = 20;
double pumpSeconds = 12.0;

// 可動範囲のハードガード。開始角からこれ以上ずれたら即座に打ち切る。
// 実機は机のヘリに置いてあり、±45° を超えると振子がヘリに当たる（2026/08/17）。
// 既定 30° は 45° に対して 15° の余裕を残した値。
double angleLimitDeg = 30.0;

// 慣性同定で使う既知の摩擦係数。FrictionTest で実測済み（2026/08/10）。
double viscous = 0.027;        // 粘性摩擦 b [N·m·s/rad]
double coulomb = 0.126;        // クーロン摩擦 τ_c [N·m]（正負の平均）

for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--port" when i + 1 < args.Length: port = args[++i]; break;
        case "--id" when i + 1 < args.Length: motorId = int.Parse(args[++i]); break;
        case "--mode" when i + 1 < args.Length: mode = args[++i].ToLowerInvariant(); break;
        case "--amp" when i + 1 < args.Length: amplitudeDeg = double.Parse(args[++i]); break;
        case "--kp" when i + 1 < args.Length: kp = double.Parse(args[++i]); break;
        case "--kd" when i + 1 < args.Length: kd = double.Parse(args[++i]); break;
        case "--freq" when i + 1 < args.Length: freq = double.Parse(args[++i]); break;
        case "--freq-low" when i + 1 < args.Length: freqLow = double.Parse(args[++i]); break;
        case "--freq-high" when i + 1 < args.Length: freqHigh = double.Parse(args[++i]); break;
        case "--steps" when i + 1 < args.Length: steps = int.Parse(args[++i]); break;
        case "--seconds" when i + 1 < args.Length: pumpSeconds = double.Parse(args[++i]); break;
        case "--angle-limit" when i + 1 < args.Length: angleLimitDeg = double.Parse(args[++i]); break;
        case "--visc" when i + 1 < args.Length: viscous = double.Parse(args[++i]); break;
        case "--coulomb" when i + 1 < args.Length: coulomb = double.Parse(args[++i]); break;
        default:
            Console.Error.WriteLine($"不明な引数: {args[i]}");
            return 1;
    }
}

if (mode is not ("sweep" or "pump" or "inertia"))
{
    Console.Error.WriteLine("--mode は sweep / pump / inertia のいずれかを指定してください");
    return 1;
}

if (angleLimitDeg is <= 0 or > 45)
{
    Console.Error.WriteLine("--angle-limit は 0 より大きく 45 度以下にしてください");
    return 1;
}

// inertia モードの既定値。高周波・小振幅で慣性トルクを摩擦から浮かび上がらせる。
if (mode == "inertia")
{
    if (!args.Contains("--freq-low")) freqLow = 4.0;
    if (!args.Contains("--freq-high")) freqHigh = 9.0;
    if (!args.Contains("--steps")) steps = 6;
    if (!args.Contains("--amp")) amplitudeDeg = 5.0;
    if (!args.Contains("--kp")) kp = 50.0;
    if (!args.Contains("--kd")) kd = 0.5;
}

if (amplitudeDeg is <= 0 or > 30)
{
    Console.Error.WriteLine("--amp は 0 より大きく 30 度以下にしてください（安全のため）");
    return 1;
}

double amp = amplitudeDeg * Math.PI / 180.0;

Console.WriteLine("=== 開ループ加振（振子エンコーダ不要）===");
Console.WriteLine();
Console.WriteLine("★★★ 実行前に必ず確認 ★★★");
Console.WriteLine();
Console.WriteLine("  1. **振子モジュールがベース板に確実に固定されているか**");
Console.WriteLine("     パッド（23×23）が窪みに嵌り、M4 で締結され、側面レールで挟まれていること。");
Console.WriteLine("     磁石とペグだけでは回り止めになりません。**滑ると모ジュールが飛びます**");
Console.WriteLine("  2. モータ本体が机に固定されているか（反力が出ます）");
Console.WriteLine("  3. 振子の回転範囲（半径 130mm 程度）に手・物・ケーブルが無いか");
Console.WriteLine("  4. 電源スイッチにすぐ手が届くか");
Console.WriteLine();
Console.WriteLine($"アームを ±{amplitudeDeg:0.0}° で正弦波駆動します（kp={kp}, kd={kd}）。");
Console.WriteLine("振子は共振点で大きく振れます。");
Console.WriteLine();
Console.Write("上記すべて確認したら Enter（中止は Ctrl+C）: ");
Console.ReadLine();
Console.WriteLine();

using var cancellation = new CancellationTokenSource();
Console.CancelKeyPress += (_, e) => { e.Cancel = true; cancellation.Cancel(); };

try
{
    using var motor = new Motor(port, motorId);

    MotorFeedback? status = motor.ReadStatus();
    if (status is null)
    {
        Console.Error.WriteLine("モータが応答しません。24V電源とケーブルを確認してください。");
        return 1;
    }
    if (motor.ReadMode() != ControlMode.Mit)
    {
        Console.Error.WriteLine("制御モードが MIT ではありません。SetControlMode で切り替えてください。");
        return 1;
    }

    IReadOnlyList<string> problems = motor.VerifyScaling();
    if (problems.Count > 0)
    {
        Console.Error.WriteLine("MotorScaling がモータの実レジスタと一致しません:");
        foreach (string x in problems) Console.Error.WriteLine($"  {x}");
        return 1;
    }

    GCSettings.LatencyMode = GCLatencyMode.SustainedLowLatency;

    // 加振の中心は**必ず実測位置**にする。0 を中心にするとモータ原点まで引き戻される。
    double center = status.PositionRad;
    double angleLimit = angleLimitDeg * Math.PI / 180.0;
    Console.WriteLine($"開始角: {center:0.000} rad（{center * 180 / Math.PI:0.0}°）");
    Console.WriteLine();

    if (mode == "pump")
    {
        RunPump(motor, center, amp, kp, kd, hz, freq, pumpSeconds, angleLimit, cancellation.Token);
    }
    else
    {
        var results = RunSweep(motor, center, amp, kp, kd, hz, freqLow, freqHigh,
                               steps, cyclesPerStep, angleLimit, cancellation.Token);
        if (mode == "inertia") ReportInertia(results, amp, viscous, coulomb);
        else ReportResonance(results);
    }

    // 終了位置を必ず報告する。次回の可動範囲の判断に要る。
    MotorFeedback? after = motor.ReadStatus();
    if (after is not null)
    {
        double moved = (after.PositionRad - center) * 180 / Math.PI;
        Console.WriteLine();
        Console.WriteLine($"終了角: {after.PositionRad:0.000} rad"
                          + $"（{after.PositionRad * 180 / Math.PI:0.0}°、開始から {moved:+0.0;-0.0}°）");
    }
}
catch (IOException ex)
{
    Console.Error.WriteLine($"通信エラー: {ex.Message}");
    return 1;
}

Console.WriteLine();
Console.WriteLine("完了。モータは安全な状態（失能）です。");
return 0;


// ============================================================================
/// <summary>
/// 指定周波数で1区間だけ正弦波駆動し、追従誤差とトルクの RMS を返す。
/// 振子が共振すると反力でアームの負荷が増えるので、この2つに山が出る。
///
/// 目標角は <paramref name="center"/> を中心に振る。<see cref="Motor.MitCommand"/> の
/// 位置はモータ内部の**絶対角**なので、これを 0 にすると開始位置から原点まで
/// 一気に引き戻す指令になる。2026/08/17 時点で実機は −2.5 rad 付近に静止しており、
/// 中心を 0 にすると 143° 動いて机のヘリに当たる。**必ず実測位置を渡すこと。**
/// </summary>
static (double ErrorRms, double TorqueRms, double PhaseSec, bool Aborted) DriveSegment(
    Motor motor, double center, double amp, double kp, double kd, double hz,
    double f, double seconds, double phase0, double angleLimit, CancellationToken ct)
{
    long periodTicks = (long)(Stopwatch.Frequency / hz);
    long start = Stopwatch.GetTimestamp();
    long next = start;
    double omega = 2 * Math.PI * f;

    double errSq = 0, torqueSq = 0;
    int n = 0;
    bool aborted = false;
    double settle = Math.Min(seconds * 0.4, 1.5);   // 過渡を捨てる

    while (!ct.IsCancellationRequested)
    {
        double t = (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;
        if (t >= seconds) break;

        double phase = phase0 + omega * t;
        double target = center + amp * Math.Sin(phase);
        double targetRate = amp * omega * Math.Cos(phase);

        motor.MitCommand(target, targetRate, kp, kd, 0.0);
        MotorFeedback? fb = motor.ReadFeedback(TimeSpan.FromSeconds(0.5 / hz));

        if (fb is not null)
        {
            if (fb.Error is not (0 or 1))
            {
                Console.Error.WriteLine($"\nモータがエラーを報告: {fb.Error}");
                aborted = true;
                break;
            }

            // 可動範囲のハードガード。ここを緩めると机のヘリに当たる。
            double excursion = Math.Abs(fb.PositionRad - center);
            if (excursion > angleLimit)
            {
                Console.Error.WriteLine(
                    $"\n★可動範囲を超えました: 開始角から {excursion * 180 / Math.PI:0.0}° "
                    + $"（上限 {angleLimit * 180 / Math.PI:0.0}°）。打ち切ります。");
                aborted = true;
                break;
            }

            if (t > settle)
            {
                double e = target - fb.PositionRad;
                errSq += e * e;
                torqueSq += fb.TorqueNm * fb.TorqueNm;
                n++;
            }
        }

        next += periodTicks;
        SpinUntil(next);
    }

    double phaseEnd = phase0 + omega * seconds;
    return (n > 0 ? Math.Sqrt(errSq / n) : 0.0,
            n > 0 ? Math.Sqrt(torqueSq / n) : 0.0,
            phaseEnd, aborted);
}


// ============================================================================
static List<(double F, double Err, double Tq)> RunSweep(
    Motor motor, double center, double amp, double kp, double kd, double hz,
    double fLow, double fHigh, int steps, int cycles, double angleLimit, CancellationToken ct)
{
    Console.WriteLine("=== 周波数掃引 ===");
    Console.WriteLine();
    Console.WriteLine($"中心角 {center:0.000} rad（{center * 180 / Math.PI:0.0}°）を基準に振ります。");
    Console.WriteLine($"開始角から ±{angleLimit * 180 / Math.PI:0.0}° を超えたら打ち切ります。");
    Console.WriteLine();
    Console.WriteLine($"{"周波数[Hz]",11} {"追従誤差RMS[deg]",17} {"トルクRMS[N·m]",16}");

    var results = new List<(double F, double Err, double Tq)>();
    double phase = 0.0;

    try
    {
        motor.Enable();
        for (int i = 0; i < steps && !ct.IsCancellationRequested; i++)
        {
            double f = fLow + (fHigh - fLow) * i / Math.Max(steps - 1, 1);
            double seconds = Math.Max(cycles / f, 3.0);
            var (err, tq, ph, aborted) =
                DriveSegment(motor, center, amp, kp, kd, hz, f, seconds, phase, angleLimit, ct);
            phase = ph;
            if (aborted) break;
            results.Add((f, err, tq));
            Console.WriteLine($"{f,11:0.00} {err * 180 / Math.PI,17:0.000} {tq,16:0.0000}");
        }
    }
    finally
    {
        Decelerate(motor, hz);
        motor.Disable();
    }

    return results;
}


// ============================================================================
static void ReportResonance(List<(double F, double Err, double Tq)> results)
{
    if (results.Count < 3) return;

    var peakErr = results.MaxBy(r => r.Err);
    var peakTq = results.MaxBy(r => r.Tq);
    Console.WriteLine();
    Console.WriteLine("=== 結果 ===");
    Console.WriteLine($"追従誤差が最大   : {peakErr.F:0.00} Hz");
    Console.WriteLine($"報告トルクが最大 : {peakTq.F:0.00} Hz");
    Console.WriteLine();
    Console.WriteLine("自由振動から予測される固有振動数は 1.75 Hz（周期 0.573 秒）。");
    Console.WriteLine("山がその近くに出れば、モデルと実機が一致していることの裏付けになります。");
    Console.WriteLine("山が出ない場合は、アダプタが滑っているか振幅が足りない可能性があります。");
    Console.WriteLine();
    Console.WriteLine($"→ 見つかった周波数で  --mode pump --freq {peakErr.F:0.00}  を実行すると振れ上がります。");
}


// ============================================================================
/// <summary>
/// 掃引結果からアームの慣性モーメント J を求める。
///
/// θ = A·sin(ωt) で駆動したときのモータトルクは
///     τ = J·θ̈ + b·θ̇ + τ_c·sign(θ̇)
///       = −J A ω² sin(ωt) + b A ω cos(ωt) + τ_c·sign(cos(ωt))
/// 慣性項は sin、粘性項とクーロン項は cos に同相なので**直交する**。したがって
///     τ_rms² = ½J²A²ω⁴ + [ ½b²A²ω² + (4/π)·b·A·ω·τ_c + τ_c² ]
/// 角括弧の中は b と τ_c が既知なら計算できる（FrictionTest で実測済み）。
/// 差し引いた残りから J が直接出る。
///
/// 低い周波数では慣性項が摩擦に埋もれて残差が負になる。そのため
/// **高周波・小振幅**で測る。振幅が小さいので可動範囲の制約とも両立する。
/// </summary>
static void ReportInertia(List<(double F, double Err, double Tq)> results,
                          double amp, double b, double tauC)
{
    if (results.Count == 0) return;

    Console.WriteLine();
    Console.WriteLine("=== 慣性モーメントの同定 ===");
    Console.WriteLine();
    Console.WriteLine($"既知の摩擦: b = {b:0.0000} N·m·s/rad,  τ_c = {tauC:0.000} N·m");
    Console.WriteLine($"振幅 A = {amp:0.0000} rad（{amp * 180 / Math.PI:0.0}°）");
    Console.WriteLine();
    Console.WriteLine($"{"周波数[Hz]",11} {"τ_rms²",12} {"摩擦分",12} {"慣性分",12} {"J[kg·m²]",13}");

    var estimates = new List<double>();
    foreach (var (f, _, tq) in results)
    {
        double w = 2 * Math.PI * f;
        double friction = 0.5 * b * b * amp * amp * w * w
                          + (4.0 / Math.PI) * b * amp * w * tauC
                          + tauC * tauC;
        double total = tq * tq;
        double inertial = total - friction;

        if (inertial <= 0)
        {
            Console.WriteLine($"{f,11:0.00} {total,12:0.00000} {friction,12:0.00000} "
                              + $"{inertial,12:0.00000} {"（信号不足）",13}");
            continue;
        }

        double j = Math.Sqrt(2 * inertial) / (amp * w * w);
        estimates.Add(j);
        Console.WriteLine($"{f,11:0.00} {total,12:0.00000} {friction,12:0.00000} "
                          + $"{inertial,12:0.00000} {j,13:0.000e+00}");
    }

    Console.WriteLine();
    if (estimates.Count < 2)
    {
        Console.WriteLine("有効な点が足りません。周波数をさらに上げるか振幅を増やしてください。");
        return;
    }

    double mean = estimates.Average();
    double sd = Math.Sqrt(estimates.Sum(x => (x - mean) * (x - mean)) / (estimates.Count - 1));
    Console.WriteLine($"J = {mean:0.000e+00} ± {sd:0.000e+00} kg·m²  "
                      + $"（有効 {estimates.Count} 点、ばらつき {sd / mean * 100:0.0}%）");
    Console.WriteLine();
    Console.WriteLine("周波数によらず一定なら信頼できます。単調に増減する場合は");
    Console.WriteLine("b か τ_c の値がずれています（残差の傾きに摩擦の誤差が乗る）。");
    Console.WriteLine();
    Console.WriteLine($"→ furuta_model.PARAMS[\"J_r\"] を {mean:0.000e+00} に更新してください。");
}


// ============================================================================
static void RunPump(Motor motor, double center, double amp, double kp, double kd, double hz,
                    double f, double seconds, double angleLimit, CancellationToken ct)
{
    Console.WriteLine($"=== 共振加振：{f:0.00} Hz で {seconds:0} 秒 ===");
    Console.WriteLine();
    Console.WriteLine("ブランコを漕ぐのと同じ原理で、振子にエネルギーを溜めます。");
    Console.WriteLine("振幅が大きくなると固有振動数が下がるので、真上までは上がりません。");
    Console.WriteLine("**上がりきらないのは正常です。** 本来のスイングアップには振子角の計測が要ります。");
    Console.WriteLine();

    try
    {
        motor.Enable();
        double phase = 0.0;
        long start = Stopwatch.GetTimestamp();
        double elapsed = 0;

        while (elapsed < seconds && !ct.IsCancellationRequested)
        {
            double chunk = Math.Min(2.0, seconds - elapsed);
            var (err, tq, ph, aborted) =
                DriveSegment(motor, center, amp, kp, kd, hz, f, chunk, phase, angleLimit, ct);
            phase = ph;
            if (aborted) break;
            elapsed = (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;
            Console.WriteLine($"  t={elapsed,5:0.0}s  追従誤差RMS {err * 180 / Math.PI,6:0.00}°  "
                              + $"トルクRMS {tq,7:0.000} N·m");
        }
    }
    finally
    {
        Decelerate(motor, hz);
        motor.Disable();
    }

    Console.WriteLine();
    Console.WriteLine("判定:");
    Console.WriteLine("  振子が大きく振れた → 機構・トルク経路ともに健全。エンコーダが届けば本番へ進めます");
    Console.WriteLine("  ほとんど振れない   → 加振周波数がずれているか、アダプタが滑っています");
    Console.WriteLine("  モジュールがずれた → **トルク経路が閉じていません**。固定を見直してください");
}


// ============================================================================
/// <summary>トルクを抜いて静止させる。急に失能すると振子の勢いで暴れる。</summary>
static void Decelerate(Motor motor, double hz)
{
    long periodTicks = (long)(Stopwatch.Frequency / hz);
    long until = Stopwatch.GetTimestamp() + (long)(2.0 * Stopwatch.Frequency);
    long next = Stopwatch.GetTimestamp();
    while (Stopwatch.GetTimestamp() < until)
    {
        motor.TorqueCommand(0.0);
        motor.ReadFeedback(TimeSpan.FromMilliseconds(2));
        next += periodTicks;
        SpinUntil(next);
    }
}

static void SpinUntil(long deadlineTicks)
{
    var spin = new SpinWait();
    while (Stopwatch.GetTimestamp() < deadlineTicks) spin.SpinOnce();
}
