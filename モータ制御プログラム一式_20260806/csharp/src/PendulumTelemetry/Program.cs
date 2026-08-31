// 8/21 作成: 振子角（マイコン経由）とアーム角（モータのCANフィードバック）を
//            同時にPCへ取り込み、FurutaGains のオブザーバ・LQRを計算して表示する。
//
// ★★★ トルク指令は常に 0.0（motor.TorqueCommand(0.0)）。モータを動かす指令は一切出さない。★★★
//      ただし ReadFeedback() を高速に呼ぶために Enable() はしている。
//      CLAUDE.md より「kp=0, kd=0 かつ t_ff=0 なら指令トルクが0なので動かないのは当然の挙動」
//      「失能時と使能時（トルク0）で感触に差がなく、寄生トルクなし」（8/6逆駆動テストで確認済み）。
//      当初は motor.Enable() すら呼ばない方針だったが、ReadStatus() は1回あたり最大100ms超と
//      重く、300Hzを前提にしたオブザーバゲインでは発散したため、8/21にこの方式へ切り替えた。
// ★★★ 表示専用。u_total はあくまで「今の状態ならこう指令するはず」という計算結果。★★★
// ★★★ 座標系の符号を実機と照合していないため、実際の制御にはまだ使えない。 ★★★
//      （詳細は FurutaObserver.cs 冒頭のコメント）
//
// 実行:  dotnet run --project src/PendulumTelemetry
//        dotnet run --project src/PendulumTelemetry -- COM4 COM5   ← ポートを指定する場合
//                                                        振子側 モータ側

using System.Diagnostics;
using DamiaoCan;
using PendulumTelemetry;

string pendulumPort = args.Length > 0 ? args[0] : "COM4";
string motorPort = args.Length > 1 ? args[1] : "COM5";
const int MotorId = 4;

// ★Windowsの既定タイマ分解能(約15.6ms)のままだと、SpinWait内部のThread.Sleep(1)が
//   最大15.6msかかり、オブザーバの安定条件(dt<13ms)を破る。必ず最初に上げておく。
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

// エンコーダの受信をバックグラウンドで回し続ける
var pumpTask = Task.Run(() => encoder.PumpUntilCancelled(cancellation.Token), cancellation.Token);

// --- 通信確認 ---
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
Console.WriteLine();
Console.WriteLine(">> 振子を真上（0度の位置）に持っていき、静止させてから Enter を押してください。");
Console.ReadLine();
int zeroOffsetCount = encoder.Latest!.Value.Count;
double armZeroRad = motor.ReadStatus()?.PositionRad ?? initialStatus.PositionRad;
Console.WriteLine($">> ゼロ点を校正しました（生カウント {zeroOffsetCount}、アーム {armZeroRad * 180.0 / Math.PI:0.0}度 をそれぞれ0度とします）。");
Console.WriteLine(">> ※アーム角の正式な原点合わせ（先生・メンバーBとの確認）は別途必要。");

// --- オブザーバ用の共有状態（バックグラウンドスレッドと表示ループの橋渡し）---
object stateLock = new();
double sharedArmDeg = 0.0;
double sharedPendulumDeg = 0.0;
FurutaObserver observer = new();
long observerUpdateCount = 0;

// dt の統計。タイマ分解能の設定が効いているかを数値で確認するため。
// オブザーバの安定条件は dt < 13ms（A-LC の固有値から算出、8/21）。
double maxDtMs = 0.0;
long slowStepCount = 0;          // dt が 13ms を超えた回数
const double DtLimitMs = 13.0;
object dtLock = new();

// --- オブザーバ更新ループ（CAN通信の速度で回る。専用スレッドでMotorを叩く）---
// Motor/SerialPort はスレッドセーフではない想定なので、CANを触るのはこのスレッドだけにする。
//
// トルク0を送り続けて ReadFeedback() で読む方式（8/21変更、詳細は冒頭コメント）。
// 「送信→即座に読む」を1組にすることで、ReadStatus()の失能→100ms待ちを回避し、
// 300Hz設計に対して現実的なレートに近づける。
Console.WriteLine(">> モータへトルク0を送り続けます（動きません）。ReadFeedbackで高速読み取りします。");
motor.Enable();

var observerTask = Task.Run(() =>
{
    try
    {
        var sw = Stopwatch.StartNew();
        double lastT = sw.Elapsed.TotalSeconds;

        while (!cancellation.IsCancellationRequested)
        {
            motor.TorqueCommand(0.0);   // ★常にトルク0。これ以外の指令は送らない
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
            lock (dtLock)
            {
                if (dtMs > maxDtMs) maxDtMs = dtMs;
                if (dtMs > DtLimitMs) slowStepCount++;
            }

            double thetaRad = arm.PositionRad - armZeroRad;
            double alphaRad = PendulumEncoderReader.CountToDegrees(f.Count - zeroOffsetCount) * Math.PI / 180.0;

            observer.Step(thetaRad, alphaRad, dt, appliedTorqueNm: 0.0);   // ★実際に送ったのは常に0
            Interlocked.Increment(ref observerUpdateCount);

            lock (stateLock)
            {
                sharedArmDeg = thetaRad * 180.0 / Math.PI;
                sharedPendulumDeg = alphaRad * 180.0 / Math.PI;
            }
        }
    }
    finally
    {
        // Ctrl+C・例外・正常終了のいずれでも必ず失能させる（CLAUDE.mdの方針）
        try { motor.Disable(); } catch { /* 終了処理中の例外は無視 */ }
    }
}, cancellation.Token);

Console.WriteLine();
Console.WriteLine(">> 表示を開始します。Ctrl+Cで終了。");
Console.WriteLine(">> u_total は表示のみ。モータへは送信していません。");
Console.WriteLine();
Console.WriteLine(" 振子角    アーム角  | 推定α̇     推定θ̇   | u_fb    u_fric   u_total | obs更新Hz  最大dt[ms]  13ms超");

var displaySw = Stopwatch.StartNew();
long lastCount = 0;
double lastDisplayT = 0;

while (!cancellation.IsCancellationRequested)
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
    long slowSnapshot;
    lock (dtLock) { maxDtSnapshot = maxDtMs; slowSnapshot = slowStepCount; }

    Console.WriteLine(
        $" {Wrap180(pendulumDeg),7:+0.0;-0.0}   {Wrap180(armDeg),7:+0.0;-0.0}  | " +
        $"{observer.AlphaDotHat,7:+0.00;-0.00}  {observer.ThetaDotHat,7:+0.00;-0.00}  | " +
        $"{observer.LastUFeedback,6:+0.00;-0.00}  {observer.LastUFriction,6:+0.00;-0.00}   {observer.LastUTotal,6:+0.00;-0.00}  | " +
        $"{obsHz,6:0}  {maxDtSnapshot,9:0.0}  {slowSnapshot,6}");

    Thread.Sleep(100);   // 表示は10Hzで十分。オブザーバ自体は裏でCANの速度のまま更新中
}

cancellation.Cancel();
try { Task.WaitAll(new[] { pumpTask, observerTask }, 500); } catch { /* 中断時の例外は無視 */ }

lock (dtLock)
{
    Console.WriteLine();
    Console.WriteLine($">> dt統計: 最大 {maxDtMs:0.0} ms / 13ms超が {slowStepCount} 回 / 総更新 {observerUpdateCount} 回");
    if (observerUpdateCount > 0)
        Console.WriteLine($">> 13ms超の割合: {100.0 * slowStepCount / observerUpdateCount:0.000} %");
}

Console.WriteLine(">> 終了しました。");
return 0;

static double Wrap180(double deg)
{
    deg %= 360.0;
    if (deg > 180.0) deg -= 360.0;
    if (deg < -180.0) deg += 360.0;
    return deg;
}
