// 摩擦の切り分けと定量化。
//
// 「手で回したときの抵抗は、ギヤの摩擦なのか、モータが力を出しているのか」を判定する。
//
// --mode ab（既定・モータは動かない）
//     「失能」と「使能＋t_ff=0」を数秒ごとに切り替える。
//     BLDC はドライバ失能時は相が開放され、電気的な制動が無い状態になる。
//     したがって両者の感触が同じなら、抵抗はすべて機械的なもの（ギヤ摩擦＋コギング）で、
//     モータは力を出していない＝MITモードの純粋トルク制御が正しく効いている。
//     感触が違えば、使能時に何かトルクが出ている。
//
// --mode breakaway（★モータが回転する）
//     トルクをゼロからゆっくり増やし、軸が動き出した瞬間のトルクを記録する。
//     これが静止摩擦（スティクション）の実測値。正転・逆転を複数回繰り返す。
//
//     この数値は摩擦補償にそのまま使うほか、振子の設計にも効く。
//     平衡制御に必要なトルクが静止摩擦と同程度だと、リミットサイクル（微小振動）が
//     消えなくなるため、比が小さければ振子を重く／長くする判断材料になる。
//
// --mode kinetic（★モータが回転する）
//     一定速度で回したときに必要なトルクを、複数の速度で測る。
//     PI速度制御の積分項が定常状態で摩擦トルクそのものに収束することを利用する。
//
//     得られた (ω, τ) に τ = τ_coulomb + b·ω を直線当てはめし、
//     クーロン摩擦 τ_coulomb と粘性摩擦係数 b を同時に求める。
//
//     動摩擦は Stribeck 効果により静止摩擦より小さい。平衡制御中はアームが
//     動いているため、実際に効くのはこちら。b はモデルの粘性項にそのまま入る。
//
// 実行:
//   dotnet run --project src/FrictionTest                      ← A/B 判定（安全）
//   dotnet run --project src/FrictionTest -- --mode breakaway  ← 静止摩擦の実測（回転する）
//   dotnet run --project src/FrictionTest -- --mode kinetic    ← 動摩擦・粘性の実測（回転する）

using System.Diagnostics;
using DamiaoCan;

const string DefaultPort = "/dev/cu.usbserial-120";
const int DefaultMotorId = 4;

string port = DefaultPort;
int motorId = DefaultMotorId;
string mode = "ab";
double maxTorque = 1.0;      // 安全のための上限 [N·m]。定格 3 N·m に対して十分小さく
double rampRate = 0.10;      // トルクの増やし方 [N·m/s]
int trials = 3;              // 各方向の試行回数

// kinetic モードで回す速度 [rad/s]。
// 平衡制御中のアーム速度は 2 rad/s 程度までなので、低速域のデータの方が実際に効く。
// 振子を付けた状態では高速で回すと振り回されるため、既定を低めにしてある。
double[] speeds = [0.5, 1.0, 2.0, 3.0];

for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--port" when i + 1 < args.Length: port = args[++i]; break;
        case "--id" when i + 1 < args.Length: motorId = int.Parse(args[++i]); break;
        case "--mode" when i + 1 < args.Length: mode = args[++i].ToLowerInvariant(); break;
        case "--max-torque" when i + 1 < args.Length: maxTorque = double.Parse(args[++i]); break;
        case "--ramp" when i + 1 < args.Length: rampRate = double.Parse(args[++i]); break;
        case "--trials" when i + 1 < args.Length: trials = int.Parse(args[++i]); break;
        case "--speeds" when i + 1 < args.Length:
            speeds = [.. args[++i].Split(',').Select(double.Parse)];
            break;
        default:
            Console.Error.WriteLine($"不明な引数: {args[i]}");
            return 1;
    }
}

if (mode is not ("ab" or "breakaway" or "kinetic" or "inertia"))
{
    Console.Error.WriteLine($"--mode は ab / breakaway / kinetic / inertia のいずれかを指定してください（指定値: {mode}）");
    return 1;
}

if (maxTorque is <= 0 or > 3.0)
{
    // 定格 3 N·m を超える指定は事故のもと
    Console.Error.WriteLine($"--max-torque は 0 より大きく 3.0 以下にしてください（指定値: {maxTorque}）");
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
        Console.Error.WriteLine("制御モードが MIT ではありません。");
        Console.Error.WriteLine("先に  dotnet run --project src/SetControlMode -- --set mit  を実行してください。");
        return 1;
    }

    Console.WriteLine($"通信OK  {status}");
    Console.WriteLine();

    switch (mode)
    {
        case "ab": RunAbTest(motor, cancellation.Token); break;
        case "breakaway": RunBreakawayTest(motor, maxTorque, rampRate, trials, cancellation.Token); break;
        case "kinetic": RunKineticTest(motor, maxTorque, speeds, cancellation.Token); break;
        case "inertia": RunInertiaTest(motor, cancellation.Token); break;
    }
}
catch (IOException ex)
{
    Console.Error.WriteLine($"通信エラー: {ex.Message}");
    return 1;
}
catch (UnauthorizedAccessException ex)
{
    Console.Error.WriteLine($"ポート {port} を使用できません: {ex.Message}");
    return 1;
}

Console.WriteLine();
Console.WriteLine("完了。モータは安全な状態（失能）です。");
return 0;


// ============================================================================
// A/B 判定：失能 ⇔ 使能＋t_ff=0 を交互に切り替える
// ============================================================================
static void RunAbTest(Motor motor, CancellationToken ct)
{
    const double PhaseSeconds = 5.0;
    const double Hz = 200.0;
    const int Rounds = 3;

    Console.WriteLine("=== A/B 判定 ===");
    Console.WriteLine("5秒ごとに「失能」と「使能＋トルク0」を切り替えます。");
    Console.WriteLine("どちらの状態でも軸を手で回し続け、感触が変わるかを見てください。");
    Console.WriteLine();
    Console.WriteLine("  感触が同じ   → 抵抗はすべて機械的（ギヤ摩擦＋コギング）。合格");
    Console.WriteLine("  使能側が重い → 使能時に何かトルクが出ている。要調査");
    Console.WriteLine();
    Console.WriteLine("中断は Ctrl+C。");
    Console.WriteLine();

    long periodTicks = (long)(Stopwatch.Frequency / Hz);

    try
    {
        for (int round = 1; round <= Rounds && !ct.IsCancellationRequested; round++)
        {
            foreach (bool enabled in (bool[])[false, true])
            {
                if (ct.IsCancellationRequested) break;

                Console.WriteLine(enabled
                    ? $"[{round}/{Rounds}] ★ 使能＋トルク0 ★ … 回してください"
                    : $"[{round}/{Rounds}] ── 失能（無通電）── … 回してください");

                if (enabled) motor.Enable(); else motor.Disable();

                long start = Stopwatch.GetTimestamp();
                long next = start;
                long limit = start + (long)(PhaseSeconds * Stopwatch.Frequency);

                while (Stopwatch.GetTimestamp() < limit && !ct.IsCancellationRequested)
                {
                    // 使能中だけ指令を送る。失能中は何も送らない（通電していない）
                    if (enabled)
                    {
                        motor.TorqueCommand(0.0);
                        motor.ReadFeedback(TimeSpan.FromSeconds(0.5 / Hz));
                    }

                    next += periodTicks;
                    SpinUntil(next);
                }
            }
        }
    }
    finally
    {
        motor.Disable();
    }

    Console.WriteLine();
    Console.WriteLine("=== 判定 ===");
    Console.WriteLine("感触に差が無ければ、モータはトルクを出していません。Day 1 は合格です。");
    Console.WriteLine("残る抵抗は 10:1 遊星ギヤの摩擦とコギングで、これは正常です。");
    Console.WriteLine("その大きさは  --mode breakaway  で数値化できます。");
}


// ============================================================================
// 静止摩擦の実測：トルクをゆっくり上げ、動き出した瞬間の値を記録する
// ============================================================================
static void RunBreakawayTest(Motor motor, double maxTorque, double rampRate, int trials, CancellationToken ct)
{
    const double Hz = 200.0;
    const double MotionThresholdRad = 0.02;   // これだけ動いたら「動き出した」と判定
    const double SettleSeconds = 1.0;         // 各試行の間に止まるのを待つ時間

    Console.WriteLine("=== 静止摩擦（ブレークアウェイトルク）の実測 ===");
    Console.WriteLine();
    Console.WriteLine("★★ このテストではモータが回転します ★★");
    Console.WriteLine();
    Console.WriteLine("  ・モータが固定されているか（固定されていないと本体側が回ります）");
    Console.WriteLine("  ・出力軸の回転範囲に手・物・ケーブルが無いか");
    Console.WriteLine("  ・電源スイッチにすぐ手が届くか");
    Console.WriteLine();
    Console.WriteLine($"トルクを 0 から {rampRate:0.00} N·m/s で増やし、上限 {maxTorque:0.00} N·m で打ち切ります。");
    Console.WriteLine($"正転・逆転を各 {trials} 回。中断は Ctrl+C。");
    Console.WriteLine();
    Console.Write("開始してよければ Enter を押してください（中止する場合は Ctrl+C）: ");
    Console.ReadLine();
    Console.WriteLine();

    long periodTicks = (long)(Stopwatch.Frequency / Hz);
    List<double> positive = [], negative = [];

    try
    {
        motor.Enable();

        foreach (double direction in (double[])[+1.0, -1.0])
        {
            for (int trial = 1; trial <= trials && !ct.IsCancellationRequested; trial++)
            {
                // 基準位置を取る
                MotorFeedback? origin = null;
                for (int i = 0; i < 20 && origin is null; i++)
                {
                    motor.TorqueCommand(0.0);
                    origin = motor.ReadFeedback(TimeSpan.FromMilliseconds(10));
                }
                if (origin is null)
                {
                    Console.Error.WriteLine("フィードバックが取得できません。中止します。");
                    return;
                }

                double startPosition = origin.PositionRad;
                double torque = 0.0;
                double? breakaway = null;

                long start = Stopwatch.GetTimestamp();
                long next = start;

                while (!ct.IsCancellationRequested)
                {
                    double elapsed = (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;
                    torque = Math.Min(rampRate * elapsed, maxTorque);

                    motor.TorqueCommand(direction * torque);
                    MotorFeedback? feedback = motor.ReadFeedback(TimeSpan.FromSeconds(0.5 / Hz));

                    if (feedback is not null)
                    {
                        if (feedback.Error is not (0 or 1))
                        {
                            Console.Error.WriteLine($"モータがエラーを報告: {DescribeError(feedback.Error)}");
                            return;
                        }

                        // 16bit エンコーダの巻き戻りを跨いでも正しく差を取る
                        double moved = Math.Abs(WrapToPi(feedback.PositionRad - startPosition));
                        if (moved > MotionThresholdRad)
                        {
                            breakaway = torque;
                            break;
                        }
                    }

                    if (torque >= maxTorque)
                    {
                        // 上限まで上げても動かなかった
                        break;
                    }

                    next += periodTicks;
                    SpinUntil(next);
                }

                // すぐトルクを抜いて止める
                long stopUntil = Stopwatch.GetTimestamp() + (long)(SettleSeconds * Stopwatch.Frequency);
                while (Stopwatch.GetTimestamp() < stopUntil)
                {
                    motor.TorqueCommand(0.0);
                    motor.ReadFeedback(TimeSpan.FromMilliseconds(2));
                }

                string label = direction > 0 ? "正転" : "逆転";
                if (breakaway is { } value)
                {
                    Console.WriteLine($"  {label} 試行{trial}: {value:0.000} N·m で動き出した");
                    (direction > 0 ? positive : negative).Add(value);
                }
                else
                {
                    Console.WriteLine($"  {label} 試行{trial}: 上限 {maxTorque:0.00} N·m まで動かなかった");
                }
            }
        }
    }
    finally
    {
        motor.Disable();
    }

    // --- 結果 ---------------------------------------------------------------
    Console.WriteLine();
    Console.WriteLine("=== 結果 ===");
    Report("正転", positive);
    Report("逆転", negative);

    List<double> all = [.. positive, .. negative];
    if (all.Count > 0)
    {
        double mean = all.Average();
        Console.WriteLine();
        Console.WriteLine($"静止摩擦トルク τ_c ≒ {mean:0.000} N·m");
        Console.WriteLine();
        Console.WriteLine("この値の使い道:");
        Console.WriteLine("  ・クーロン摩擦補償  τ += τ_c * tanh(θ̇/ε)  にそのまま入れる");
        Console.WriteLine("  ・振子の設計判断: 平衡制御に必要なトルクがこの値と同程度だと");
        Console.WriteLine("    リミットサイクルが消えない。必要トルクが τ_c の 5〜10 倍あるのが望ましい");
        Console.WriteLine("  ・QUBE-Servo 2 の素の寸法での必要制御トルクは約 0.014 N·m しかない。");
        Console.WriteLine("    アーム延長と振子への錘追加で必要トルクを引き上げる必要がある");
    }

    static void Report(string label, List<double> samples)
    {
        if (samples.Count == 0)
        {
            Console.WriteLine($"{label}: 測定できたデータなし");
            return;
        }
        double mean = samples.Average();
        Console.WriteLine($"{label}: 平均 {mean:0.000} N·m  " +
                          $"(最小 {samples.Min():0.000} / 最大 {samples.Max():0.000}、{samples.Count}回)");
    }
}


// ============================================================================
// 動摩擦・粘性摩擦の実測：一定速度で回し、定常トルクを速度ごとに測る
// ============================================================================
static void RunKineticTest(Motor motor, double maxTorque, double[] targets, CancellationToken ct)
{
    const double Hz = 200.0;
    const double SettleSeconds = 3.5;    // 速度が定常に達するのを待つ。振子の揺れも収まるよう長めに
    const double MeasureSeconds = 3.0;   // 平均を取る区間。振子の振動を平均化できる長さにする
    const double Kp = 0.08;              // 速度誤差 [rad/s] → トルク [N·m]
    const double Ki = 1.0;               // 積分項。定常状態でこれが摩擦トルクに収束する

    Console.WriteLine("=== 動摩擦・粘性摩擦の実測 ===");
    Console.WriteLine();
    Console.WriteLine("★★ このテストではモータが連続回転します ★★");
    Console.WriteLine();
    Console.WriteLine("  ・モータが固定されているか");
    Console.WriteLine("  ・出力軸の回転範囲に手・物・ケーブルが無いか（連続で回り続けます）");
    Console.WriteLine("  ・電源スイッチにすぐ手が届くか");
    Console.WriteLine();
    Console.WriteLine($"速度 {string.Join(", ", targets.Select(v => $"±{v:0.#}"))} rad/s で順に回し、");
    Console.WriteLine($"定常状態の必要トルクを測ります（トルク上限 {maxTorque:0.00} N·m）。");
    Console.WriteLine($"1点あたり約 {SettleSeconds + MeasureSeconds:0.0} 秒、全 {targets.Length * 2} 点。中断は Ctrl+C。");
    Console.WriteLine();
    Console.Write("開始してよければ Enter を押してください（中止する場合は Ctrl+C）: ");
    Console.ReadLine();
    Console.WriteLine();

    long periodTicks = (long)(Stopwatch.Frequency / Hz);
    List<(double Velocity, double Torque)> positive = [], negative = [];

    try
    {
        motor.Enable();

        foreach (double direction in (double[])[+1.0, -1.0])
        {
            foreach (double target in targets)
            {
                if (ct.IsCancellationRequested) break;

                double commanded = direction * target;
                double integral = 0.0;
                double velocitySum = 0.0, torqueSum = 0.0;
                int samples = 0;

                long start = Stopwatch.GetTimestamp();
                long settleUntil = start + (long)(SettleSeconds * Stopwatch.Frequency);
                long finishAt = settleUntil + (long)(MeasureSeconds * Stopwatch.Frequency);
                long next = start;
                bool failed = false;

                while (Stopwatch.GetTimestamp() < finishAt && !ct.IsCancellationRequested)
                {
                    MotorFeedback? feedback = motor.ReadFeedback(TimeSpan.FromSeconds(0.5 / Hz));

                    if (feedback is not null)
                    {
                        if (feedback.Error is not (0 or 1))
                        {
                            Console.Error.WriteLine($"モータがエラーを報告: {DescribeError(feedback.Error)}");
                            failed = true;
                            break;
                        }

                        double error = commanded - feedback.VelocityRadPerSec;
                        integral += error / Hz;

                        // アンチワインドアップ。積分項だけで上限を超えないよう抑える
                        integral = Math.Clamp(integral, -maxTorque / Ki, maxTorque / Ki);

                        double torque = Math.Clamp(Kp * error + Ki * integral, -maxTorque, maxTorque);
                        motor.TorqueCommand(torque);

                        // 整定を待ってから平均を取る
                        if (Stopwatch.GetTimestamp() >= settleUntil)
                        {
                            velocitySum += feedback.VelocityRadPerSec;
                            torqueSum += torque;
                            samples++;
                        }
                    }

                    next += periodTicks;
                    SpinUntil(next);
                }

                if (failed) return;

                if (samples > 0)
                {
                    double meanVelocity = velocitySum / samples;
                    double meanTorque = torqueSum / samples;
                    Console.WriteLine($"  指令 {commanded,5:+0.0;-0.0} rad/s → " +
                                      $"実測 {meanVelocity,6:+0.00;-0.00} rad/s で " +
                                      $"トルク {meanTorque,6:+0.000;-0.000} N·m");

                    // 目標に届いていない点は摩擦の定常値になっていないので捨てる
                    if (Math.Abs(meanVelocity - commanded) < 0.3 * target)
                        (direction > 0 ? positive : negative).Add((meanVelocity, meanTorque));
                    else
                        Console.WriteLine("      （目標速度に届いていないため、当てはめから除外）");
                }
            }

            // 方向を変える前に一度止める
            Decelerate(motor, Hz);
        }
    }
    finally
    {
        Decelerate(motor, Hz);
        motor.Disable();
    }

    // --- 直線当てはめ τ = τ_coulomb + b·ω ------------------------------------
    Console.WriteLine();
    Console.WriteLine("=== 結果 ===");

    double? coulombPositive = Fit("正転", positive);
    double? coulombNegative = Fit("逆転", negative);

    if (coulombPositive is { } cp && coulombNegative is { } cn)
    {
        double coulomb = (cp + Math.Abs(cn)) / 2.0;
        Console.WriteLine();
        Console.WriteLine($"動摩擦（クーロン摩擦） τ_c ≒ {coulomb:0.000} N·m");
        Console.WriteLine();
        Console.WriteLine("使い道:");
        Console.WriteLine($"  ・摩擦補償  τ += {coulomb:0.000} * tanh(θ̇/ε)  の係数");
        Console.WriteLine("  ・粘性係数 b はモデルのアーム側減衰項にそのまま入れる");
        Console.WriteLine("  ・静止摩擦（breakaway）より小さいはず。平衡制御中はアームが");
        Console.WriteLine("    動いているので、実際に効くのはこの動摩擦の方");
    }

    // 最小二乗で τ = a + b·ω に当てはめ、切片 a（クーロン摩擦）を返す
    static double? Fit(string label, List<(double Velocity, double Torque)> points)
    {
        if (points.Count < 2)
        {
            Console.WriteLine($"{label}: 有効なデータ点が {points.Count} 個しかなく、当てはめできません");
            return null;
        }

        double n = points.Count;
        double sumX = points.Sum(p => p.Velocity);
        double sumY = points.Sum(p => p.Torque);
        double sumXy = points.Sum(p => p.Velocity * p.Torque);
        double sumXx = points.Sum(p => p.Velocity * p.Velocity);

        double denominator = n * sumXx - sumX * sumX;
        if (Math.Abs(denominator) < 1e-12)
        {
            Console.WriteLine($"{label}: 速度が散らばっておらず、当てはめできません");
            return null;
        }

        double slope = (n * sumXy - sumX * sumY) / denominator;
        double intercept = (sumY - slope * sumX) / n;

        Console.WriteLine($"{label}: τ = {intercept:+0.000;-0.000} {(slope >= 0 ? "+" : "-")} " +
                          $"{Math.Abs(slope):0.0000}·ω   " +
                          $"(クーロン {Math.Abs(intercept):0.000} N·m / 粘性 {Math.Abs(slope):0.0000} N·m·s/rad、{points.Count}点)");
        return intercept;
    }
}

/// <summary>トルクを抜いて惰性で止まるのを待つ。</summary>
static void Decelerate(Motor motor, double hz)
{
    long periodTicks = (long)(Stopwatch.Frequency / hz);
    long until = Stopwatch.GetTimestamp() + (long)(1.5 * Stopwatch.Frequency);
    long next = Stopwatch.GetTimestamp();

    while (Stopwatch.GetTimestamp() < until)
    {
        motor.TorqueCommand(0.0);
        motor.ReadFeedback(TimeSpan.FromMilliseconds(2));
        next += periodTicks;
        SpinUntil(next);
    }
}


// ============================================================================
// 慣性の実測：トルク段差から角加速度を測り、差分で摩擦を消す
// ============================================================================
static void RunInertiaTest(Motor motor, CancellationToken ct)
{
    const double Hz = 300.0;
    const double SpeedLimit = 12.0;   // これを超えたら打ち切る [rad/s]
    const double MaxBurst = 0.35;     // 1回の加速の上限時間 [s]
    double[] levels = [0.25, 0.35, 0.45];
    const int Repeats = 3;

    Console.WriteLine("=== モータ軸まわりの全慣性 J の実測 ===");
    Console.WriteLine();
    Console.WriteLine("★★ アームが短時間ずつ加速します ★★");
    Console.WriteLine();
    Console.WriteLine("  ・モータが固定されているか、回転範囲に手・物が無いか");
    Console.WriteLine();
    Console.WriteLine("原理: 一定トルク τ を与えると  J·θ̈ = τ − τ_friction。");
    Console.WriteLine("      2つのトルクで測って差を取ると **摩擦が完全に消える**:");
    Console.WriteLine("          J = (τ₂ − τ₁) / (θ̈₂ − θ̈₁)");
    Console.WriteLine("      周波数掃引と違い、摩擦の値を知らなくても求まる。");
    Console.WriteLine();
    Console.Write("開始してよければ Enter（中止は Ctrl+C）: ");
    Console.ReadLine();
    Console.WriteLine();

    long periodTicks = (long)(Stopwatch.Frequency / Hz);
    var accel = new Dictionary<double, List<double>>();
    foreach (double L in levels) accel[L] = [];

    try
    {
        motor.Enable();
        for (int rep = 0; rep < Repeats && !ct.IsCancellationRequested; rep++)
        {
            foreach (double level in levels)
            {
                foreach (double dir in (double[])[+1.0, -1.0])
                {
                    // 静止させてから
                    Settle(motor, Hz, 1.2);

                    var ts = new List<double>(128);
                    var ws = new List<double>(128);
                    long start = Stopwatch.GetTimestamp();
                    long next = start;

                    while (!ct.IsCancellationRequested)
                    {
                        double t = (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;
                        if (t > MaxBurst) break;

                        motor.TorqueCommand(dir * level);
                        MotorFeedback? fb = motor.ReadFeedback(TimeSpan.FromSeconds(0.5 / Hz));
                        if (fb is not null)
                        {
                            if (fb.Error is not (0 or 1))
                            {
                                Console.Error.WriteLine($"モータがエラーを報告: {DescribeError(fb.Error)}");
                                return;
                            }
                            double w = dir * fb.VelocityRadPerSec;
                            if (Math.Abs(w) > SpeedLimit) break;
                            // 離脱直後の過渡を避け、確実に滑っている領域だけ使う
                            if (w > 1.0) { ts.Add(t); ws.Add(w); }
                        }

                        next += periodTicks;
                        SpinUntil(next);
                    }

                    if (ts.Count >= 8)
                    {
                        double slope = Slope(ts, ws);
                        if (slope > 0) accel[level].Add(slope);
                    }
                }
            }
            Console.WriteLine($"  試行 {rep + 1}/{Repeats} 完了");
        }
    }
    finally
    {
        Settle(motor, Hz, 1.5);
        motor.Disable();
    }

    // --- 結果 ---------------------------------------------------------------
    Console.WriteLine();
    Console.WriteLine("=== 結果 ===");
    Console.WriteLine($"{"トルク[N·m]",12} {"角加速度[rad/s²]",18} {"回数",6}");
    var pts = new List<(double Tau, double Acc)>();
    foreach (double level in levels)
    {
        if (accel[level].Count == 0) { Console.WriteLine($"{level,12:0.00}  データなし"); continue; }
        double mean = accel[level].Average();
        pts.Add((level, mean));
        Console.WriteLine($"{level,12:0.00} {mean,18:0.0} {accel[level].Count,6}");
    }

    if (pts.Count < 2)
    {
        Console.WriteLine("\n有効な点が2つ未満です。トルクを上げるか、速度上限を緩めてください。");
        return;
    }

    // τ = J·θ̈ + τ_friction を最小二乗。傾きが J、切片が摩擦
    double n = pts.Count;
    double sx = pts.Sum(p => p.Acc), sy = pts.Sum(p => p.Tau);
    double sxy = pts.Sum(p => p.Acc * p.Tau), sxx = pts.Sum(p => p.Acc * p.Acc);
    double J = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    double tf = (sy - J * sx) / n;

    Console.WriteLine();
    Console.WriteLine($"**J = {J * 1e3:0.000} ×10⁻³ kg·m²**");
    Console.WriteLine($"  切片から求まる摩擦 = {tf:0.000} N·m（独立実測 0.126 と比べて妥当か確認）");
    Console.WriteLine();
    Console.WriteLine("この J には以下がすべて含まれる:");
    Console.WriteLine("  ・モータのロータ慣性 × 100（減速比 10:1 の二乗。おそらく支配的）");
    Console.WriteLine("  ・アダプタ板・スペーサ");
    Console.WriteLine("  ・QUBE モジュール本体");
    Console.WriteLine("  ・振子の m_p·L_r²");
    Console.WriteLine();
    Console.WriteLine("→ furuta_model.PARAMS の J_r には、この値から m_p·L_r² を引いた分を入れる。");
}

/// <summary>最小二乗の傾き。</summary>
static double Slope(List<double> x, List<double> y)
{
    double n = x.Count, sx = x.Sum(), sy = y.Sum();
    double sxy = 0, sxx = 0;
    for (int i = 0; i < x.Count; i++) { sxy += x[i] * y[i]; sxx += x[i] * x[i]; }
    return (n * sxy - sx * sy) / (n * sxx - sx * sx);
}

/// <summary>トルクを抜いて止まるのを待つ。</summary>
static void Settle(Motor motor, double hz, double seconds)
{
    long until = Stopwatch.GetTimestamp() + (long)(seconds * Stopwatch.Frequency);
    while (Stopwatch.GetTimestamp() < until)
    {
        motor.TorqueCommand(0.0);
        motor.ReadFeedback(TimeSpan.FromMilliseconds(2));
    }
}


// ============================================================================
// 補助
// ============================================================================

/// <summary>次の締切までスピンで待つ。Thread.Sleep より分解能が高い。</summary>
static void SpinUntil(long deadlineTicks)
{
    var spin = new SpinWait();
    while (Stopwatch.GetTimestamp() < deadlineTicks) spin.SpinOnce();
}

/// <summary>±π に畳む。エンコーダが値域端で巻き戻っても差分を正しく取るため。</summary>
static double WrapToPi(double radians)
{
    double range = 2.0 * MotorScaling.PositionMax;
    double wrapped = (radians + MotorScaling.PositionMax) % range;
    if (wrapped < 0) wrapped += range;
    return wrapped - MotorScaling.PositionMax;
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
