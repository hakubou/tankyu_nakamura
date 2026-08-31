// 8/26 作成: 振子の倒立保持（バランス制御）に初めて挑戦するプログラム。
//
// ★★★ PendulumTelemetry と違い、これは実際にモータへトルクを送る。 ★★★
//      座標系の符号は8/24（s_theta=+1）・8/26（s_alpha=+1）に実測で確定済み
//      （SignCalibration、いずれも反転なし）。PendulumTelemetryの角度計算ロジックは
//      そのまま正しいので、このプログラムでも同じ計算式を流用している。
//
// ゲイン倍率（第3引数、既定1.0）で FurutaGains.StateFeedback の効き方を弱めて試せる。
// 「低いゲインから慎重に」というメンバーBの警告（ゲイン感度が高く、Brysonの重みを2倍にする
// だけでリミットサイクルが20倍以上悪化した実績あり）を踏まえた、実機での最初のつまみ。
//
// 実行:  dotnet run --project src/BalanceControl -- COM4 COM5          ← ゲイン1.0
//        dotnet run --project src/BalanceControl -- COM4 COM5 0.3     ← ゲイン0.3から様子見
//                                                    振子側 モータ側  ゲイン倍率
//
// 安全:
//   ・アーム角が開始角から ArmAngleLimitDeg を超えたら即座にトルク0＋失能して終了
//     （机のヘリは±45°。この機構では1回転させない）
//   ・振子角が上向きから PendulumAngleAbortDeg を超えたら「倒立に失敗した」とみなし
//     即座にトルク0＋失能して終了（スイングアップは未実装。線形化ゲインは上向き近傍でしか有効でない）
//   ・Ctrl+C／電源スイッチでいつでも止められる状態で実行すること

using System.Diagnostics;
using System.Text;
using DamiaoCan;
using PendulumTelemetry;

// ★8/28追加: --log <path> を付けたときだけ時系列（t, θ, α, u）をCSVに残す。
// 外乱テストで「どれだけ乱れて、何秒で戻ったか」を後から数値で見るため。
// 既定（--logなし）では今までと完全に同じ動作・同じ負荷にする
// （制御ループが時間に敏感なため、頼まれていないときに余計な仕事をしない）。
string? logPath = null;
{
    var argList = args.ToList();
    int logIdx = argList.IndexOf("--log");
    if (logIdx >= 0 && logIdx + 1 < argList.Count)
    {
        logPath = argList[logIdx + 1];
        argList.RemoveRange(logIdx, 2);
    }
    args = argList.ToArray();
}

string pendulumPort = args.Length > 0 ? args[0] : "COM4";
string motorPort = args.Length > 1 ? args[1] : "COM5";
double gainScale = args.Length > 2 ? double.Parse(args[2]) : 1.0;
double frictionScale = args.Length > 3 ? double.Parse(args[3]) : 1.0;
double thetaGainBoost = args.Length > 4 ? double.Parse(args[4]) : 1.0;
const int MotorId = 4;

// ★8/26: 40°では制御の実力を見る前に打ち切ってしまうため、140°へ拡大（メンバーA）。
// 机の縁は新しい固定治具でクリアランスを再確認済み。ただし「1回転はさせない
// （配線が引っ掛かる）」は別の制約として引き続き有効。±140°は振れ幅280°で
// 1回転(360°)には達しないが、ケーブルのたるみで追従できる範囲かは別途要確認。
const double ArmAngleLimitDeg = 140.0;
const double PendulumAngleAbortDeg = 60.0;   // 線形化が意味を持つ範囲の外に出たら打ち切り

Console.WriteLine("=== 倒立保持（バランス制御） ===");
Console.WriteLine();
Console.WriteLine($"ゲイン倍率: {gainScale:0.00} ／ 摩擦補償倍率: {frictionScale:0.00} ／ θ復元力の追加倍率: {thetaGainBoost:0.00}");
Console.WriteLine(logPath is not null
    ? $"時系列ログ: 有効（終了時に {logPath} へ書き出します）"
    : "時系列ログ: 無効（--log <path> を付けると記録します）");
Console.WriteLine();
Console.WriteLine("★★★ 実行前に必ず確認 ★★★");
Console.WriteLine();
Console.WriteLine("  1. 振子モジュールがベース板に確実に固定されているか（M4・M3・側面レールすべて）");
Console.WriteLine("  2. モータ本体が土台に固定されているか（反力が出ます）");
Console.WriteLine("  3. アームが自由に±40°動ける状態か（ケーブル・手・物が可動範囲に無いか）");
Console.WriteLine("  4. 電源スイッチにすぐ手が届くか（Ctrl+Cでも停止できます）");
Console.WriteLine($"  5. アーム角が開始角から±{ArmAngleLimitDeg:0}°、振子角が上向きから±{PendulumAngleAbortDeg:0}°を");
Console.WriteLine("     超えたら自動でトルク0＋失能します（それでも手を離さないこと）");
Console.WriteLine();
Console.Write("上記すべて確認したら Enter（中止は Ctrl+C）: ");
Console.ReadLine();

using var timerResolution = WindowsTimerResolution.Begin(1);

Console.WriteLine($">> 振子エンコーダ: {pendulumPort} (921600 baud) / モータ: {motorPort}");

using var encoder = new PendulumEncoderReader(pendulumPort);
using var motor = new Motor(motorPort, MotorId);

using var cancellation = new CancellationTokenSource();
Console.CancelKeyPress += (_, e) =>
{
    e.Cancel = true;
    cancellation.Cancel();
};

var pumpTask = Task.Run(() => encoder.PumpUntilCancelled(cancellation.Token), cancellation.Token);

MotorFeedback? initialStatus = motor.ReadStatus();
if (initialStatus is null)
{
    Console.Error.WriteLine("モータが応答しません。24V電源とケーブルを確認してください。");
    cancellation.Cancel();
    return 1;
}
Console.WriteLine($">> モータ接続OK。現在位置 {initialStatus.PositionRad * 180.0 / Math.PI:0.0} 度。");

Console.WriteLine(">> 振子エンコーダからのフレーム待機中...");
var waitStart = DateTime.UtcNow;
while (encoder.Latest is null)
{
    if ((DateTime.UtcNow - waitStart).TotalSeconds > 5)
    {
        Console.Error.WriteLine("振子エンコーダからフレームが届きません。マイコンの電源・COMポート・ボーレート(921600)を確認してください。");
        cancellation.Cancel();
        return 1;
    }
    Thread.Sleep(50);
}
Console.WriteLine(">> フレーム受信OK。");

// --- ゼロ点校正（振子: 先端から見て真上が0度／アーム: 現在位置を0度）---
// PendulumTelemetry と同じ校正手順・同じ符号（s_theta=+1, s_alpha=+1で反転なしのため式は同一）。
Console.WriteLine();
Console.WriteLine(">> 振子を真上（0度の位置）に持っていき、静止させてから Enter を押してください。");
Console.ReadLine();
int zeroOffsetCount = encoder.Latest!.Value.Count;
double armZeroRad = motor.ReadStatus()?.PositionRad ?? initialStatus.PositionRad;
Console.WriteLine($">> ゼロ点を校正しました（生カウント {zeroOffsetCount}、アーム {armZeroRad * 180.0 / Math.PI:0.0}度 をそれぞれ0度とします）。");

Console.WriteLine();
Console.WriteLine(">> このまま振子を真上付近で支えていてください。3秒後に制御を開始します。");
for (int i = 3; i >= 1; i--)
{
    Console.WriteLine($"   {i}...");
    Thread.Sleep(1000);
}
Console.WriteLine(">> 制御開始。手を離してください。Ctrl+Cでいつでも停止できます。");
Console.WriteLine();
Console.WriteLine(" 振子角    アーム角  | 推定α̇     推定θ̇   | u_fb    u_fric   u_total | obs更新Hz  最大dt[ms]");

double armAngleLimitRad = ArmAngleLimitDeg * Math.PI / 180.0;

object stateLock = new();
double sharedArmDeg = 0.0;
double sharedPendulumDeg = 0.0;
FurutaObserver observer = new();
long observerUpdateCount = 0;
string? abortReason = null;

double maxDtMs = 0.0;
object dtLock = new();

// ★8/26追加: 複数回の試行を数字で比較できるよう、保持時間と最大の乱れを記録する。
double maxAbsArmDeg = 0.0;
double maxAbsPendulumDeg = 0.0;
object extremaLock = new();

// ★8/28追加: --logが指定されたときだけ使う時系列バッファ。
// ループ中はList.Addのみ（ファイルI/Oはループの外、終了後に一括で行う）。
// 10分・600Hz相当を確保しておき、実行中の再確保（GCジッタの原因）をほぼ避ける。
List<(double T, double ThetaDeg, double AlphaDeg, double AlphaDotHat, double ThetaDotHat, double UFb, double UFric, double UTotal, double DtMs)>? timeSeriesLog =
    logPath is not null ? new(capacity: 10 * 60 * 600) : null;

motor.Enable();
var controlStartSw = Stopwatch.StartNew();

var controlTask = Task.Run(() =>
{
    try
    {
        var sw = Stopwatch.StartNew();
        double lastT = sw.Elapsed.TotalSeconds;
        double appliedTorque = 0.0;   // 直前のループで実際に送ったトルク（Step()に渡すのはこれ）

        while (!cancellation.IsCancellationRequested)
        {
            motor.TorqueCommand(appliedTorque);
            MotorFeedback? arm = motor.ReadFeedback(TimeSpan.FromMilliseconds(5));
            PendulumFrame? frame = encoder.Latest;
            if (arm is null || frame is not { } f)
            {
                continue;
            }

            double now = sw.Elapsed.TotalSeconds;
            double dt = now - lastT;
            lastT = now;

            double dtMs = dt * 1000.0;
            lock (dtLock) { if (dtMs > maxDtMs) maxDtMs = dtMs; }

            double thetaRad = arm.PositionRad - armZeroRad;
            double alphaRad = PendulumEncoderReader.CountToDegrees(f.Count - zeroOffsetCount) * Math.PI / 180.0;

            // ★安全ガード。ここで打ち切ったら以降トルクは常に0（appliedTorqueをそのまま更新しない）。
            if (Math.Abs(thetaRad) > armAngleLimitRad)
            {
                abortReason = $"アーム角が開始角から{thetaRad * 180.0 / Math.PI:+0.0;-0.0}°（上限±{ArmAngleLimitDeg:0}°）";
                appliedTorque = 0.0;
                motor.TorqueCommand(0.0);
                break;
            }
            double alphaWrapped = Wrap180(alphaRad * 180.0 / Math.PI);
            if (Math.Abs(alphaWrapped) > PendulumAngleAbortDeg)
            {
                abortReason = $"振子角が上向きから{alphaWrapped:+0.0;-0.0}°（上限±{PendulumAngleAbortDeg:0}°）";
                appliedTorque = 0.0;
                motor.TorqueCommand(0.0);
                break;
            }

            double uCmd = observer.Step(thetaRad, alphaRad, dt, appliedTorqueNm: appliedTorque, gainScale: gainScale, frictionScale: frictionScale, thetaGainBoost: thetaGainBoost);
            appliedTorque = uCmd;   // 次のループの先頭でこれを送る
            Interlocked.Increment(ref observerUpdateCount);

            // ★8/28追加: --log指定時のみ。List.Addは事前確保した容量内なら再確保しない。
            timeSeriesLog?.Add((
                controlStartSw.Elapsed.TotalSeconds,
                thetaRad * 180.0 / Math.PI,
                Wrap180(alphaRad * 180.0 / Math.PI),
                observer.AlphaDotHat,
                observer.ThetaDotHat,
                observer.LastUFeedback,
                observer.LastUFriction,
                observer.LastUTotal,
                dtMs));

            lock (stateLock)
            {
                sharedArmDeg = thetaRad * 180.0 / Math.PI;
                sharedPendulumDeg = alphaRad * 180.0 / Math.PI;
            }
            lock (extremaLock)
            {
                double armDegAbs = Math.Abs(thetaRad * 180.0 / Math.PI);
                double pendulumDegAbs = Math.Abs(Wrap180(alphaRad * 180.0 / Math.PI));
                if (armDegAbs > maxAbsArmDeg) maxAbsArmDeg = armDegAbs;
                if (pendulumDegAbs > maxAbsPendulumDeg) maxAbsPendulumDeg = pendulumDegAbs;
            }
        }
    }
    finally
    {
        // Ctrl+C・例外・安全ガード・正常終了のいずれでも必ずトルク0にしてから失能する。
        try { motor.TorqueCommand(0.0); motor.ReadFeedback(TimeSpan.FromMilliseconds(5)); } catch { /* 終了処理中の例外は無視 */ }
        try { motor.Disable(); } catch { /* 終了処理中の例外は無視 */ }
    }
}, cancellation.Token);

var displaySw = Stopwatch.StartNew();
long lastCount = 0;
double lastDisplayT = 0;

while (!cancellation.IsCancellationRequested && abortReason is null)
{
    double armDeg, pendulumDeg;
    lock (stateLock)
    {
        armDeg = sharedArmDeg;
        pendulumDeg = sharedPendulumDeg;
    }

    long count = Interlocked.Read(ref observerUpdateCount);
    double nowT = displaySw.Elapsed.TotalSeconds;
    double obsHz = (nowT - lastDisplayT) > 0 ? (count - lastCount) / (nowT - lastDisplayT) : 0;
    lastCount = count;
    lastDisplayT = nowT;

    double maxDtSnapshot;
    lock (dtLock) { maxDtSnapshot = maxDtMs; }

    Console.WriteLine(
        $" {Wrap180(pendulumDeg),7:+0.0;-0.0}   {Wrap180(armDeg),7:+0.0;-0.0}  | " +
        $"{observer.AlphaDotHat,7:+0.00;-0.00}  {observer.ThetaDotHat,7:+0.00;-0.00}  | " +
        $"{observer.LastUFeedback,6:+0.00;-0.00}  {observer.LastUFriction,6:+0.00;-0.00}   {observer.LastUTotal,6:+0.00;-0.00}  | " +
        $"{obsHz,6:0}  {maxDtSnapshot,9:0.0}");

    Thread.Sleep(100);
}

cancellation.Cancel();
try { Task.WaitAll(new[] { pumpTask, controlTask }, 500); } catch { /* 中断時の例外は無視 */ }

Console.WriteLine();
if (abortReason is not null)
{
    Console.WriteLine($">> ★安全ガードにより自動停止しました: {abortReason}");
}
double heldSeconds;
double maxArmSnapshot, maxPendulumSnapshot;
lock (extremaLock) { maxArmSnapshot = maxAbsArmDeg; maxPendulumSnapshot = maxAbsPendulumDeg; }
heldSeconds = controlStartSw.Elapsed.TotalSeconds;
Console.WriteLine($">> 保持時間 {heldSeconds:0.0}秒（Ctrl+C・安全ガードいずれかで終了するまで） / " +
                   $"最大アーム角 {maxArmSnapshot:0.0}° / 最大振子角 {maxPendulumSnapshot:0.0}°");
Console.WriteLine(">> モータは安全な状態（トルク0→失能）です。終了しました。");

// ★8/28追加: --log指定時のみ、ループ終了後にまとめて書き出す（ループ中はI/Oしない）。
if (logPath is not null && timeSeriesLog is not null)
{
    try
    {
        string? dir = Path.GetDirectoryName(logPath);
        if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
        var sb = new StringBuilder();
        sb.Append("t_s,theta_deg,alpha_deg,alpha_dot_hat,theta_dot_hat,u_fb,u_fric,u_total,dt_ms\n");
        foreach (var row in timeSeriesLog)
        {
            sb.Append(row.T.ToString("0.000")).Append(',')
              .Append(row.ThetaDeg.ToString("0.00")).Append(',')
              .Append(row.AlphaDeg.ToString("0.00")).Append(',')
              .Append(row.AlphaDotHat.ToString("0.000")).Append(',')
              .Append(row.ThetaDotHat.ToString("0.000")).Append(',')
              .Append(row.UFb.ToString("0.000")).Append(',')
              .Append(row.UFric.ToString("0.000")).Append(',')
              .Append(row.UTotal.ToString("0.000")).Append(',')
              .Append(row.DtMs.ToString("0.0")).Append('\n');
        }
        File.WriteAllText(logPath, sb.ToString());
        Console.WriteLine($">> 時系列ログを記録しました（{timeSeriesLog.Count}行）: {logPath}");
    }
    catch (Exception ex)
    {
        Console.WriteLine($">> 警告: 時系列ログの書き出しに失敗しました（{ex.Message}）。実験自体は正常に終了しています。");
    }
}

return 0;

static double Wrap180(double deg)
{
    deg %= 360.0;
    if (deg > 180.0) deg -= 360.0;
    if (deg < -180.0) deg += 360.0;
    return deg;
}
