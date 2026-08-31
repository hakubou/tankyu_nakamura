// トルクスケールの実測校正と、重力補償デモ。
//
// ★ このツールは「モータ軸を水平にして」使う。
//   重力を既知のトルク源として使うため、軸が鉛直だとトルクが発生せず何も測れない。
//
// 準備:
//   1. モータ軸が水平になるように固定する
//   2. 長さ r [m] の腕を出力軸に付け、先端に質量 m [kg] の錘を付ける
//   3. 腕が自由に振れる（真下にぶら下がる）ことを確認する
//
//   腕が真下を向いた状態を θ=0 とすると、重力による負荷トルクは m·g·r·sin(θ)。
//   すなわち水平（θ=90°）で最大 m·g·r になる。
//
// --mode sweep（既定）: トルクスケールの実測校正
//   トルクをゆっくり上げてから下げ、各角度での「上げ側」「下げ側」のトルクを記録する。
//   摩擦はヒステリシスとして現れるので、
//       重力トルク = (上げ側 + 下げ側) / 2      ← 摩擦が相殺される
//       摩擦       = (上げ側 - 下げ側) / 2
//   となる。得られた重力トルクの振幅を理論値 m·g·r と比べれば、
//   トルクスケールの誤差が直接わかる。
//
// --mode float: 重力補償デモ
//   τ = m·g·r·sin(θ) を毎周期送り続ける。腕をどの角度に置いても、
//   そこでふわっと止まれば、トルクスケール・符号・エンコーダ方向・遅延が
//   すべて同時に検証できたことになる。倒立振子の直前まで来た証拠。
//
// 実行:
//   dotnet run --project src/TorqueCalibration -- --mass 0.100 --radius 0.100
//   dotnet run --project src/TorqueCalibration -- --mass 0.100 --radius 0.100 --mode float

using System.Diagnostics;
using System.Runtime;
using DamiaoCan;

const string DefaultPort = "/dev/cu.usbserial-120";
const int DefaultMotorId = 4;
const double G = 9.80665;

string port = DefaultPort;
int motorId = DefaultMotorId;
string mode = "sweep";
double mass = double.NaN;      // [kg]
double radius = double.NaN;    // [m]
double hz = 300.0;             // 実測で最も揃う制御周期
double frictionFeedforward = 0.0;   // float モードでの摩擦補償量 [N·m]

// クーロン摩擦の正転・逆転それぞれの実測値 [N·m]（2026/08/12）。
// sweep の解析で「上げ側と下げ側の平均」から系統誤差を差し引くのに使う。
double frictionPositive = 0.109;
double frictionNegative = 0.143;

for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--port" when i + 1 < args.Length: port = args[++i]; break;
        case "--id" when i + 1 < args.Length: motorId = int.Parse(args[++i]); break;
        case "--mode" when i + 1 < args.Length: mode = args[++i].ToLowerInvariant(); break;
        case "--mass" when i + 1 < args.Length: mass = double.Parse(args[++i]); break;
        case "--radius" when i + 1 < args.Length: radius = double.Parse(args[++i]); break;
        case "--hz" when i + 1 < args.Length: hz = double.Parse(args[++i]); break;
        case "--friction" when i + 1 < args.Length: frictionFeedforward = double.Parse(args[++i]); break;
        case "--friction-pos" when i + 1 < args.Length: frictionPositive = double.Parse(args[++i]); break;
        case "--friction-neg" when i + 1 < args.Length: frictionNegative = double.Parse(args[++i]); break;
        default:
            Console.Error.WriteLine($"不明な引数: {args[i]}");
            return 1;
    }
}

if (mode is not ("sweep" or "float"))
{
    Console.Error.WriteLine($"--mode は sweep か float を指定してください（指定値: {mode}）");
    return 1;
}

if (double.IsNaN(mass) || double.IsNaN(radius) || mass <= 0 || radius <= 0)
{
    Console.Error.WriteLine("--mass [kg] と --radius [m] を指定してください。");
    Console.Error.WriteLine("例: --mass 0.100 --radius 0.100  （100gの錘を100mm先に付けた場合）");
    return 1;
}

double gravityTorque = mass * G * radius;   // 腕が水平のときの負荷トルク [N·m]

Console.WriteLine($"錘 {mass * 1000:0} g / 腕 {radius * 1000:0} mm");
Console.WriteLine($"→ 水平時の理論トルク m·g·r = {gravityTorque:0.0000} N·m");
Console.WriteLine();

if (gravityTorque > MotorScaling.TorqueMax * 0.8)
{
    Console.Error.WriteLine($"錘が重すぎます。T_MAX={MotorScaling.TorqueMax} N·m の8割を超えています。");
    return 1;
}

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

    // トルクを出す前に必ずスケーリングを検証する。
    // ここがズレていると指令トルクが黙って定数倍狂い、校正そのものが無意味になる。
    IReadOnlyList<string> problems = motor.VerifyScaling();
    if (problems.Count > 0)
    {
        Console.Error.WriteLine("MotorScaling がモータの実レジスタと一致しません:");
        foreach (string problem in problems) Console.Error.WriteLine($"  {problem}");
        return 1;
    }
    Console.WriteLine("スケーリング検証 OK");
    Console.WriteLine();

    GCSettings.LatencyMode = GCLatencyMode.SustainedLowLatency;

    if (mode == "sweep")
        RunSweep(motor, gravityTorque, hz, (frictionPositive - frictionNegative) / 2.0,
                 cancellation.Token);
    else RunFloat(motor, gravityTorque, frictionFeedforward, hz, cancellation.Token);
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
// sweep: トルクを上げ下げして、重力トルクと摩擦を分離する
// ============================================================================
static void RunSweep(Motor motor, double gravityTorque, double hz, double asymmetryBias,
                     CancellationToken ct)
{
    // 水平を少し超えるところまで持ち上げられるトルクを上限にする
    double peakTorque = Math.Min(gravityTorque * 1.6, MotorScaling.TorqueMax * 0.9);
    const double RampSeconds = 12.0;   // 片道。ゆっくり動かすほど動摩擦の影響が素直に出る

    Console.WriteLine("=== トルクスケールの実測校正 ===");
    Console.WriteLine();
    Console.WriteLine("★★ 腕が動きます ★★");
    Console.WriteLine();
    Console.WriteLine("  ・モータ軸は水平になっていますか（鉛直だと重力トルクが出ず測定できません）");
    Console.WriteLine("  ・腕の回転範囲に手・物・ケーブルがありませんか");
    Console.WriteLine("  ・電源スイッチにすぐ手が届きますか");
    Console.WriteLine();
    Console.WriteLine($"トルクを 0 → {peakTorque:0.000} → 0 N·m と {RampSeconds * 2:0} 秒かけて往復させます。");
    Console.WriteLine("腕は真下から持ち上がり、また戻ります。");
    Console.WriteLine();
    Console.Write("腕を真下にぶら下げた状態にして Enter（中止は Ctrl+C）: ");
    Console.ReadLine();

    long periodTicks = (long)(Stopwatch.Frequency / hz);
    var samples = new List<(double Torque, double Angle, bool Rising)>(capacity: (int)(hz * RampSeconds * 2) + 64);

    double origin;
    try
    {
        motor.Enable();

        // 真下の位置を基準にする
        MotorFeedback? rest = null;
        for (int i = 0; i < 50 && rest is null; i++)
        {
            motor.TorqueCommand(0.0);
            rest = motor.ReadFeedback(TimeSpan.FromMilliseconds(10));
        }
        if (rest is null) { Console.Error.WriteLine("フィードバックが取れません。"); return; }
        origin = rest.PositionRad;

        long start = Stopwatch.GetTimestamp();
        long next = start;
        double total = RampSeconds * 2;

        while (!ct.IsCancellationRequested)
        {
            double elapsed = (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;
            if (elapsed >= total) break;

            bool rising = elapsed < RampSeconds;
            double torque = rising
                ? peakTorque * (elapsed / RampSeconds)
                : peakTorque * (1.0 - (elapsed - RampSeconds) / RampSeconds);

            motor.TorqueCommand(torque);
            MotorFeedback? feedback = motor.ReadFeedback(TimeSpan.FromSeconds(0.5 / hz));

            if (feedback is not null)
            {
                if (feedback.Error is not (0 or 1))
                {
                    Console.Error.WriteLine($"モータがエラーを報告: {feedback.Error}");
                    break;
                }
                samples.Add((torque, feedback.PositionRad - origin, rising));
            }

            next += periodTicks;
            SpinUntil(next);
        }
    }
    finally
    {
        // ゆっくりトルクを抜く。急に0にすると腕が落ちる
        for (int i = 0; i < (int)hz; i++)
        {
            motor.TorqueCommand(0.0);
            motor.ReadFeedback(TimeSpan.FromMilliseconds(2));
        }
        motor.Disable();
    }

    Analyze(samples, gravityTorque, asymmetryBias);
}

/// <summary>
/// 角度で区切って、上げ側・下げ側の平均トルクを比べる。
///
/// 上げ側では摩擦が負荷に加わり、下げ側では逆向きに働くので
///     重力トルク = (上げ + 下げ)/2、摩擦 = (上げ − 下げ)/2
/// で分離できる……のは摩擦が正逆対称な場合だけ。
///
/// 実測では正転 0.109 / 逆転 0.143 と 30% 非対称で、平均には
///     (τ_c⁺ − τ_c⁻)/2 ≒ −0.017 N·m
/// が系統誤差として残る。実測値が分かっているのでこれを差し引く。
///
/// この補正は循環論法にならない。トルクスケールが k 倍ずれていれば
/// 摩擦の実測値も同じく k 倍ずれるため、補正後の比はきれいに 1/k になる。
/// 補正を入れることで、必要な錘が 340g 級から 100〜150g 級まで下がり、
/// 3Dプリント製の錘で校正できるようになる。
/// </summary>
/// <param name="asymmetryBias">(τ_c⁺ − τ_c⁻)/2 [N·m]。上げ下げ平均から差し引く。</param>
static void Analyze(List<(double Torque, double Angle, bool Rising)> samples, double gravityTorque,
                    double asymmetryBias)
{
    Console.WriteLine();
    Console.WriteLine("=== 結果 ===");

    if (samples.Count < 100)
    {
        Console.WriteLine($"サンプルが {samples.Count} 個しかなく、解析できません。");
        return;
    }

    double maxAngle = samples.Max(s => Math.Abs(s.Angle));
    if (maxAngle < 0.2)
    {
        Console.WriteLine($"腕がほとんど動いていません（最大 {maxAngle:0.000} rad）。");
        Console.WriteLine("錘が重すぎるか、軸が鉛直になっている可能性があります。");
        return;
    }

    Console.WriteLine($"摩擦の非対称による系統誤差 {asymmetryBias:+0.0000;-0.0000} N·m を差し引きます");
    Console.WriteLine();
    Console.WriteLine($"{"角度[deg]",10} {"上げ[N·m]",11} {"下げ[N·m]",11} {"重力[N·m]",11} {"理論[N·m]",11} {"比",7}");

    var ratios = new List<double>();
    var frictions = new List<double>();

    // 15°刻みで、上げ側と下げ側の両方にデータがある区間だけ比較する
    for (double degrees = 15; degrees <= 90; degrees += 15)
    {
        double center = degrees * Math.PI / 180.0;
        const double HalfWidth = 0.06;   // rad

        var rising = samples.Where(s => s.Rising && Math.Abs(Math.Abs(s.Angle) - center) < HalfWidth).ToList();
        var falling = samples.Where(s => !s.Rising && Math.Abs(Math.Abs(s.Angle) - center) < HalfWidth).ToList();
        if (rising.Count < 3 || falling.Count < 3) continue;

        double up = rising.Average(s => s.Torque);
        double down = falling.Average(s => s.Torque);
        // 摩擦の対称成分はここで相殺され、非対称成分は実測値で差し引く
        double measured = (up + down) / 2.0 - asymmetryBias;
        double friction = (up - down) / 2.0;
        double theory = gravityTorque * Math.Sin(center);  // 真下からの角度なので sin

        ratios.Add(measured / theory);
        frictions.Add(friction);

        Console.WriteLine($"{degrees,10:0} {up,11:0.0000} {down,11:0.0000} {measured,11:0.0000} {theory,11:0.0000} {measured / theory,7:0.000}");
    }

    if (ratios.Count == 0)
    {
        Console.WriteLine("上げ側・下げ側の両方にデータがある角度がありませんでした。");
        Console.WriteLine("錘を軽くするか、可動範囲を広げてください。");
        return;
    }

    double meanRatio = ratios.Average();
    Console.WriteLine();
    Console.WriteLine($"トルクスケール比（実測 / 理論） = {meanRatio:0.000}");
    Console.WriteLine($"摩擦（ヒステリシス半幅）        = {frictions.Average():0.000} N·m");
    Console.WriteLine();

    if (Math.Abs(meanRatio - 1.0) < 0.15)
    {
        Console.WriteLine("→ 誤差 15% 以内。トルクスケールは信頼できます。");
        Console.WriteLine("  残差はギヤ効率とトルク定数のばらつきによるもので、正常な範囲です。");
    }
    else
    {
        Console.WriteLine("→ ★ 15% を超えてズレています ★");
        Console.WriteLine("  T_MAX レジスタと MotorScaling.TorqueMax の一致、");
        Console.WriteLine("  錘の質量・腕の長さの実測値、モータ軸が水平かを確認してください。");
        Console.WriteLine($"  一定倍率のズレなら、LQR 出力に {1.0 / meanRatio:0.000} を掛けて補正する手もあります。");
    }
}


// ============================================================================
// float: 重力補償デモ
// ============================================================================
static void RunFloat(Motor motor, double gravityTorque, double frictionFeedforward, double hz, CancellationToken ct)
{
    Console.WriteLine("=== 重力補償デモ ===");
    Console.WriteLine();
    Console.WriteLine("★★ 腕が動きます ★★");
    Console.WriteLine();
    Console.WriteLine("τ = m·g·r·sin(θ) を送り続けます。腕をどの角度に置いても");
    Console.WriteLine("そこでふわっと止まれば合格です。");
    Console.WriteLine();
    if (frictionFeedforward > 0)
        Console.WriteLine($"摩擦補償 {frictionFeedforward:0.000} N·m を tanh で加えます。");
    else
        Console.WriteLine("摩擦補償なし（--friction 0.13 のように指定すると加えられます）。");
    Console.WriteLine();
    Console.Write("腕を真下にぶら下げた状態にして Enter（中止は Ctrl+C）: ");
    Console.ReadLine();

    long periodTicks = (long)(Stopwatch.Frequency / hz);
    const double VelocityEpsilon = 0.15;   // tanh の平滑化幅 [rad/s]。速度ノイズより十分大きく

    try
    {
        motor.Enable();

        MotorFeedback? rest = null;
        for (int i = 0; i < 50 && rest is null; i++)
        {
            motor.TorqueCommand(0.0);
            rest = motor.ReadFeedback(TimeSpan.FromMilliseconds(10));
        }
        if (rest is null) { Console.Error.WriteLine("フィードバックが取れません。"); return; }

        double origin = rest.PositionRad;
        long start = Stopwatch.GetTimestamp();
        long next = start;
        long nextLog = start + Stopwatch.Frequency;
        MotorFeedback last = rest;

        Console.WriteLine(">> 開始。腕を手で動かしてみてください。Ctrl+C で終了。");

        while (!ct.IsCancellationRequested)
        {
            double angle = last.PositionRad - origin;

            // 真下を 0 としたので、重力による負荷は sin(θ)。これを打ち消す
            double torque = gravityTorque * Math.Sin(angle);

            // クーロン摩擦の補償。sign() を直接使うと速度ゼロ近傍でチャタリングするため tanh で平滑化する
            if (frictionFeedforward > 0)
                torque += frictionFeedforward * Math.Tanh(last.VelocityRadPerSec / VelocityEpsilon);

            motor.TorqueCommand(torque);
            MotorFeedback? feedback = motor.ReadFeedback(TimeSpan.FromSeconds(0.5 / hz));

            if (feedback is not null)
            {
                if (feedback.Error is not (0 or 1))
                {
                    Console.Error.WriteLine($"モータがエラーを報告: {feedback.Error}");
                    break;
                }
                last = feedback;
            }

            long now = Stopwatch.GetTimestamp();
            if (now >= nextLog)
            {
                Console.WriteLine($"  角度={angle * 180.0 / Math.PI,7:+0.0;-0.0}°  " +
                                  $"指令トルク={torque,7:+0.000;-0.000} N·m  " +
                                  $"速度={last.VelocityRadPerSec,6:+0.00;-0.00} rad/s  " +
                                  $"温度 {last.DriverTemperature}/{last.RotorTemperature}℃");
                nextLog += Stopwatch.Frequency;
            }

            next += periodTicks;
            SpinUntil(next);
        }
    }
    finally
    {
        for (int i = 0; i < (int)hz; i++)
        {
            motor.TorqueCommand(0.0);
            motor.ReadFeedback(TimeSpan.FromMilliseconds(2));
        }
        motor.Disable();
    }

    Console.WriteLine();
    Console.WriteLine("判定: どの角度でも腕がふわっと止まったなら合格です。");
    Console.WriteLine("      トルクスケール・符号・エンコーダ方向・遅延がすべて同時に検証できています。");
}

/// <summary>次の締切までスピンで待つ。Thread.Sleep より分解能が高い。</summary>
static void SpinUntil(long deadlineTicks)
{
    var spin = new SpinWait();
    while (Stopwatch.GetTimestamp() < deadlineTicks) spin.SpinOnce();
}
