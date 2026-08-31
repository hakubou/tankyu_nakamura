// 8/24 作成: 座標系の符号（s_theta, s_alpha）を実測で決める。
//
// ■ 背景（メンバーBからの回答、8/24）
//
// furuta_model.py のθ・αの向きの定義と、8/19に実機で決めた向きが一致するかは
// 言葉だけでは確定できない（「先端から見て」がどちら向きに顔を向けた状態かで
// 結論が反転するため）。一方で、モデルには (θ, α, u) を全部同時に反転しても
// A・Bが一致するという対称性があり（メンバーBが数値確認済み、差の最大値0.0）、
// 危険なのは「どちら回りか」ではなく **u・θ・αの3者が互いに整合しているか**。
//
// そこで、倒立させずに済む2回の実測で符号定数を直接決める。
//   s_theta = ±1    θ_meas = s_theta × (モータ報告位置)
//   s_alpha = ±1    α_meas = s_alpha × (カウント × 2π/2048) + オフセット
// これが決まれば、FurutaGains.cs の K・L・A・B は生成したままの値で使える
// （メンバーBの対称性の議論より、部分的な符号ずれさえ避ければよい）。
//
// ■ 測定1：s_theta
// ぶら下げたまま t_ff=+0.2N·m（静止摩擦0.147をわずかに超える）を短時間入れ、
// 生のモータ位置がどちらに動くかを見る。θ_measが増えるようにs_thetaを選ぶ。
//
// ■ 測定2：s_alpha
// 同じくぶら下げたまま、t_ff=+0.3N·m×50msのパルスを入れる。
// モデルの予測（u=+1N·mのとき θ̈=+454, α̈=+557 rad/s²）より、
// 正しい符号ならα_measは増加するはず。生カウントの変化方向からs_alphaを選ぶ。
//
// ■ 安全
// 実際にモータへ非ゼロトルクを送る、このプロジェクトで初めての操作。
// 机のヘリの制約（2026/08/17時点で±45°）はまだ有効。角度が超えたら即座に打ち切る。
//
// 実行:
//   dotnet run --project src/SignCalibration -- COM4 COM5
//                                                振子側 モータ側
//
// ■ 8/24追記：モジュールをアームに固定できない状況での代替手順
//
// 測定2はアームとモジュールが機械的に繋がっていないと測れない（8/24に実機で確認済み。
// 未固定でΔ0カウントだった）。結束バンドを切ってしまい仮固定も難しい状況向けに、
// **モータへトルクを送らず、手動で符号を決める**ための監視専用モードを追加した。
//
//   dotnet run --project src/SignCalibration -- --watch-theta COM5
//     モータの生位置を読み続けるだけ（トルクは送らない）。
//     手でアームを回し、数字が増える方向＝「+θ方向」＝「+e_t方向」の基準になる
//     （s_theta=+1が8/24に確定済みなので、この数字がそのままθ_measの増減と一致する）。
//
//   dotnet run --project src/SignCalibration -- --watch-alpha COM4
//     振子カウントを読み続けるだけ（モータ接続すら不要）。
//     モジュールを最終的な向き（窪み面下向き・エンコーダ側外向き）で手に持ち、
//     上のwatch-thetaで確認した「+θ方向（＝+e_t方向）」へ振子を手で振って、
//     カウントが増えるかどうかを見る。増えればs_alpha=+1、減れば-1。
//
// どちらもCtrl+Cで終了。

using System.Diagnostics;
using DamiaoCan;
using SignCalibration;

if (args.Length > 0 && args[0] is "--watch-theta" or "--watch-alpha")
{
    return RunWatchMode(args);
}

// ★8/26追加: BalanceControlで制御周期が突然600Hz→130Hz台に落ち、u_totalが飽和し続ける
// 振動が発生した事象を受けて、続行前にモータの温度・エラーコードを確認するための
// 読み取り専用モード（トルクは一切送らない）。
if (args.Length > 0 && args[0] == "--status")
{
    return RunStatusCheck(args);
}

string pendulumPort = args.Length > 0 ? args[0] : "COM4";
string motorPort = args.Length > 1 ? args[1] : "COM5";
const int MotorId = 4;

const double ThetaTorqueNm = 0.2;      // 静止摩擦0.147をわずかに超える最小限
const double ThetaPulseMs = 150.0;     // 測定1の印加時間（8/26: 組立直後は摩擦が減り300msだと20°超過。半分に短縮）
const double AlphaTorqueNm = 0.3;
const double AlphaPulseMs = 50.0;      // 測定2の印加時間（メンバーB指定）
const double SettleMs = 300.0;         // 各測定後、トルク0で減速させる時間
const double AngleLimitDeg = 20.0;     // 開始角からの可動範囲ハードガード（机のヘリ対策）
const double ControlHz = 300.0;

Console.WriteLine("=== 座標系符号の実測（測定1: s_theta、測定2: s_alpha）===");
Console.WriteLine();
Console.WriteLine("★★★ 実行前に必ず確認 ★★★");
Console.WriteLine();
Console.WriteLine("  1. 振子モジュールがベース板に確実に固定されているか");
Console.WriteLine("     （パッドが窪みに嵌り、M4で締結され、側面レールで挟まれていること）");
Console.WriteLine("  2. モータ本体が机に固定されているか（反力が出ます）");
Console.WriteLine("  3. 振子が自由にぶら下がった状態（真下、静止）になっているか");
Console.WriteLine("  4. 振子の可動範囲に手・物・ケーブルが無いか");
Console.WriteLine("  5. 電源スイッチにすぐ手が届くか");
Console.WriteLine();
Console.WriteLine($"測定1: t_ff=+{ThetaTorqueNm}N·m を{ThetaPulseMs:0}ms、測定2: t_ff=+{AlphaTorqueNm}N·m を{AlphaPulseMs:0}msだけ流します。");
Console.WriteLine($"開始角から±{AngleLimitDeg:0}°を超えたら即座に打ち切ります（机のヘリは±45°）。");
Console.WriteLine();
Console.Write("上記すべて確認したら Enter（中止は Ctrl+C）: ");
Console.ReadLine();
Console.WriteLine();

using var cancellation = new CancellationTokenSource();
Console.CancelKeyPress += (_, e) => { e.Cancel = true; cancellation.Cancel(); };

using var encoder = new PendulumEncoderReader(pendulumPort);
using var motor = new Motor(motorPort, MotorId);

var pumpTask = Task.Run(() => encoder.PumpUntilCancelled(cancellation.Token), cancellation.Token);

MotorFeedback? status = motor.ReadStatus();
if (status is null)
{
    Console.Error.WriteLine("モータが応答しません。24V電源とケーブルを確認してください。");
    cancellation.Cancel();
    return 1;
}
if (motor.ReadMode() != ControlMode.Mit)
{
    Console.Error.WriteLine("制御モードが MIT ではありません。SetControlMode で切り替えてください。");
    cancellation.Cancel();
    return 1;
}
IReadOnlyList<string> problems = motor.VerifyScaling();
if (problems.Count > 0)
{
    Console.Error.WriteLine("MotorScaling がモータの実レジスタと一致しません:");
    foreach (string x in problems) Console.Error.WriteLine($"  {x}");
    cancellation.Cancel();
    return 1;
}

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
Console.WriteLine();

double centerRad = status.PositionRad;
double angleLimit = AngleLimitDeg * Math.PI / 180.0;

Console.WriteLine($"開始角（生のモータ位置）: {centerRad:0.000} rad（{centerRad * 180 / Math.PI:0.0}°）");
Console.WriteLine($"開始時の生カウント: {encoder.Latest!.Value.Count}");
Console.WriteLine();

// ============================================================================
Console.Write("測定1（s_theta）を開始します。Enterで実行: ");
Console.ReadLine();

double thetaBefore = motor.ReadStatus()!.PositionRad;
double thetaAfter = thetaBefore;
bool aborted1 = false;

motor.Enable();
try
{
    RunPulse(motor, ThetaTorqueNm, ThetaPulseMs, ControlHz, centerRad, angleLimit,
             onSample: p => thetaAfter = p, aborted: () => aborted1 = true, ct: cancellation.Token);
}
finally
{
    Settle(motor, SettleMs, ControlHz);
    motor.Disable();
}

double thetaDeltaDeg = (thetaAfter - thetaBefore) * 180 / Math.PI;
Console.WriteLine($"  θ（生）: {thetaBefore * 180 / Math.PI:0.00}° → {thetaAfter * 180 / Math.PI:0.00}°  (Δ{thetaDeltaDeg:+0.00;-0.00}°)");

int sTheta;
if (aborted1)
{
    Console.WriteLine("  ★可動範囲を超えたため打ち切りました。s_thetaは判定できません。");
    sTheta = 0;
}
else if (Math.Abs(thetaDeltaDeg) < 0.3)
{
    Console.WriteLine("  ★動きが小さすぎて判定できません（0.3°未満）。ThetaTorqueNmを上げて再試行してください。");
    sTheta = 0;
}
else
{
    sTheta = thetaDeltaDeg > 0 ? +1 : -1;
    Console.WriteLine($"  → s_theta = {sTheta:+0;-0}");
}
Console.WriteLine();

// ============================================================================
Console.Write("測定2（s_alpha）を開始します。Enterで実行: ");
Console.ReadLine();

int alphaCountBefore = encoder.Latest!.Value.Count;
int alphaCountAfter = alphaCountBefore;
double thetaAfter2 = motor.ReadStatus()!.PositionRad;
bool aborted2 = false;

motor.Enable();
try
{
    RunPulse(motor, AlphaTorqueNm, AlphaPulseMs, ControlHz, centerRad, angleLimit,
             onSample: p =>
             {
                 thetaAfter2 = p;
                 if (encoder.Latest is { } f) alphaCountAfter = f.Count;
             },
             aborted: () => aborted2 = true, ct: cancellation.Token);
}
finally
{
    Settle(motor, SettleMs, ControlHz);
    motor.Disable();
}

int alphaCountDelta = alphaCountAfter - alphaCountBefore;
double alphaDeltaDeg = alphaCountDelta * 360.0 / 2048.0;
Console.WriteLine($"  カウント: {alphaCountBefore} → {alphaCountAfter}  (Δ{alphaCountDelta:+0;-0}カウント ≈ {alphaDeltaDeg:+0.0;-0.0}°)");

int sAlpha;
if (aborted2)
{
    Console.WriteLine("  ★可動範囲を超えたため打ち切りました。s_alphaは判定できません。");
    sAlpha = 0;
}
else if (Math.Abs(alphaCountDelta) < 5)
{
    Console.WriteLine("  ★動きが小さすぎて判定できません（5カウント未満）。AlphaTorqueNmを上げて再試行してください。");
    sAlpha = 0;
}
else
{
    sAlpha = alphaCountDelta > 0 ? +1 : -1;
    Console.WriteLine($"  → s_alpha = {sAlpha:+0;-0}");
}
Console.WriteLine();

// ============================================================================
MotorFeedback? after = motor.ReadStatus();
if (after is not null)
{
    double moved = (after.PositionRad - centerRad) * 180 / Math.PI;
    Console.WriteLine($"終了角: {after.PositionRad * 180 / Math.PI:0.0}°（開始から {moved:+0.0;-0.0}°）");
}
Console.WriteLine();

if (sTheta != 0 && sAlpha != 0)
{
    Console.WriteLine("=== 結果。以下をコードに反映してください ===");
    Console.WriteLine();
    Console.WriteLine("// 2026/08/24 実測により決定（SignCalibrationツール、メンバーBの手順に基づく）");
    Console.WriteLine($"private const int SThetaSign = {sTheta};  // θ_meas = SThetaSign * (モータ報告位置)");
    Console.WriteLine($"private const int SAlphaSign = {sAlpha};  // α_meas = SAlphaSign * (カウント×2π/2048) + オフセット");
}
else
{
    Console.WriteLine("=== 判定できなかった測定があります。上記の対処をしてから再実行してください ===");
}

cancellation.Cancel();
try { pumpTask.Wait(500); } catch { /* 終了処理中の例外は無視 */ }
Console.WriteLine();
Console.WriteLine("完了。モータは安全な状態（失能）です。");
return sTheta != 0 && sAlpha != 0 ? 0 : 1;


// ============================================================================
/// <summary>
/// t_ff=torqueNmを durationMs だけ送り続け、tickごとにonSampleへ現在のθ（生位置）を渡す。
/// 開始角からangleLimitを超えたら即座に打ち切り、abortedを呼ぶ。
/// </summary>
static void RunPulse(Motor motor, double torqueNm, double durationMs, double hz,
                     double centerRad, double angleLimit,
                     Action<double> onSample, Action aborted, CancellationToken ct)
{
    long periodTicks = (long)(Stopwatch.Frequency / hz);
    long start = Stopwatch.GetTimestamp();
    long next = start;
    double seconds = durationMs / 1000.0;

    while (!ct.IsCancellationRequested)
    {
        double t = (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;
        if (t >= seconds) break;

        motor.TorqueCommand(torqueNm);
        MotorFeedback? fb = motor.ReadFeedback(TimeSpan.FromMilliseconds(5));

        if (fb is not null)
        {
            if (fb.Error is not (0 or 1))
            {
                Console.Error.WriteLine($"\nモータがエラーを報告: {fb.Error}");
                aborted();
                return;
            }

            double excursion = Math.Abs(fb.PositionRad - centerRad);
            if (excursion > angleLimit)
            {
                Console.Error.WriteLine(
                    $"\n★可動範囲を超えました: 開始角から {excursion * 180 / Math.PI:0.0}° "
                    + $"（上限 {angleLimit * 180 / Math.PI:0.0}°）。打ち切ります。");
                aborted();
                return;
            }

            onSample(fb.PositionRad);
        }

        next += periodTicks;
        SpinUntil(next);
    }
}

/// <summary>トルクを抜いて静止させる。急に失能すると振子の勢いで暴れる。</summary>
static void Settle(Motor motor, double durationMs, double hz)
{
    long periodTicks = (long)(Stopwatch.Frequency / hz);
    long until = Stopwatch.GetTimestamp() + (long)(durationMs / 1000.0 * Stopwatch.Frequency);
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

// ============================================================================
// 8/24追記: モータへトルクを一切送らない監視専用モード。
// --watch-theta はモータの生位置を、--watch-alpha は振子カウントを表示し続けるだけ。
// アームやモジュールを手で動かして、数字がどちらに増えるかを目で確認するために使う。
static int RunWatchMode(string[] args)
{
    string mode = args[0];
    string port = args.Length > 1 ? args[1] : (mode == "--watch-theta" ? "COM5" : "COM4");

    if (mode == "--watch-theta")
    {
        Console.WriteLine("=== θ監視モード（モータの生位置を表示するだけ。トルクは送りません）===");
        Console.WriteLine("アームを手で回して、数字が増える方向を確認してください。Ctrl+Cで終了。");
        Console.WriteLine();

        using var cancellation = new CancellationTokenSource();
        Console.CancelKeyPress += (_, e) => { e.Cancel = true; cancellation.Cancel(); };

        using var motor = new Motor(port, 4);
        MotorFeedback? status = motor.ReadStatus();
        if (status is null)
        {
            Console.Error.WriteLine("モータが応答しません。24V電源とケーブルを確認してください。");
            return 1;
        }

        while (!cancellation.IsCancellationRequested)
        {
            MotorFeedback? fb = motor.ReadStatus(tries: 5);
            if (fb is not null)
            {
                Console.Write($"\rθ（生）: {fb.PositionRad * 180 / Math.PI,7:0.00}°   ");
            }
            Thread.Sleep(150);
        }
        Console.WriteLine();
        return 0;
    }
    else
    {
        Console.WriteLine("=== α監視モード（振子カウントを表示するだけ。モータは使いません）===");
        Console.WriteLine("モジュールを最終的な向き（窪み面下向き・エンコーダ側外向き）で持ち、");
        Console.WriteLine("振子を手で振って、数字が増える方向を確認してください。Ctrl+Cで終了。");
        Console.WriteLine();

        using var cancellation = new CancellationTokenSource();
        Console.CancelKeyPress += (_, e) => { e.Cancel = true; cancellation.Cancel(); };

        using var encoder = new PendulumEncoderReader(port);
        var pumpTask = Task.Run(() => encoder.PumpUntilCancelled(cancellation.Token), cancellation.Token);

        Console.WriteLine(">> フレーム待機中...");
        var waitStart = DateTime.UtcNow;
        while (encoder.Latest is null)
        {
            if ((DateTime.UtcNow - waitStart).TotalSeconds > 5)
            {
                Console.Error.WriteLine("フレームが届きません。COMポートを確認してください。");
                cancellation.Cancel();
                return 1;
            }
            Thread.Sleep(50);
        }
        Console.WriteLine(">> 受信OK。");
        Console.WriteLine();

        while (!cancellation.IsCancellationRequested)
        {
            if (encoder.Latest is { } f)
            {
                double deg = f.Count * 360.0 / 2048.0;
                Console.Write($"\rカウント: {f.Count,6}   角度: {deg,7:0.0}°   ");
            }
            Thread.Sleep(100);
        }
        Console.WriteLine();

        cancellation.Cancel();
        try { pumpTask.Wait(500); } catch { /* 終了処理中の例外は無視 */ }
        return 0;
    }
}

// --status: モータの温度・エラーコードを1回読んで表示するだけ。トルクは送らない。
// 実行:  dotnet run --project src/SignCalibration -- --status COM5
static int RunStatusCheck(string[] args)
{
    string port = args.Length > 1 ? args[1] : "COM5";

    using var motor = new Motor(port, 4);
    MotorFeedback? fb = motor.ReadStatus(tries: 5);
    if (fb is null)
    {
        Console.Error.WriteLine("モータが応答しません。24V電源とケーブルを確認してください。");
        return 1;
    }

    Console.WriteLine("=== モータ状態 ===");
    Console.WriteLine($"位置: {fb.PositionRad * 180 / Math.PI,7:0.00}°");
    Console.WriteLine($"速度: {fb.VelocityRadPerSec,7:0.00} rad/s");
    Console.WriteLine($"トルク: {fb.TorqueNm,7:0.00} N·m");
    Console.WriteLine($"ドライバ温度: {fb.DriverTemperature,3} ℃");
    Console.WriteLine($"モータ内部温度: {fb.RotorTemperature,3} ℃");

    string errorText = fb.Error switch
    {
        0 => "正常",
        1 => "使能中（正常）",
        8 => "★過電圧",
        9 => "★低電圧",
        0xA => "★過電流",
        0xB => "★MOS過温",
        0xC => "★コイル過温",
        0xD => "★通信喪失",
        0xE => "★過負荷",
        _ => $"★不明なコード {fb.Error}",
    };
    Console.WriteLine($"エラーコード: {fb.Error}（{errorText}）");
    return 0;
}
