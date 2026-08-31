// 2026/8/6 に実際に成功した動作: 速度モードで 0.5 rad/s、12.6秒でちょうど1回転
// 実測結果: 359.5〜359.6度（1.00回転）、速度のばらつきはほぼ無し
//
// 実行前の確認:
//   1. 24V電源が入っていること（これを忘れて動かず、原因究明に時間を使った）
//   2. モータが固定されていること
//   3. 出力軸の回転範囲に手や物が無いこと
//   4. 電源スイッチにすぐ手が届くこと
//
// 実行:  dotnet run --project src/DemoOneTurn
//        dotnet run --project src/DemoOneTurn -- COM5        ← ポートを指定する場合

using DamiaoCan;

const string DefaultPort = "COM3";   // デバイスマネージャーの「ポート(COMとLPT)」で確認できる
const int MotorId = 4;               // 本機のCAN ID
const double Speed = 0.5;            // rad/s（約3rpm）ゆっくり
const double Turns = 1.0;            // 回転数

string port = args.Length > 0 ? args[0] : DefaultPort;

// Ctrl+C を押されても、その場でプロセスを殺さずに減速停止→失能を通す
using var cancellation = new CancellationTokenSource();
Console.CancelKeyPress += (_, e) =>
{
    e.Cancel = true;
    cancellation.Cancel();
};

try
{
    using var motor = new Motor(port, MotorId);

    // --- 1. 通信できるか確認する（この時点ではモータは動かない）-------------
    MotorFeedback? status = motor.ReadStatus();
    if (status is null)
    {
        Console.Error.WriteLine("モータが応答しません。24V電源とケーブルを確認してください。");
        return 1;
    }

    Console.WriteLine($"通信OK  CAN ID={status.Id}  位置={status.PositionRad:+0.00;-0.00} rad  " +
                      $"エラー={status.Error}  温度 {status.DriverTemperature}/{status.RotorTemperature}℃");

    // --- 2. 制御モードを確認し、速度モードでなければ変更する ---------------
    ControlMode? mode = motor.ReadMode();
    Console.WriteLine(mode is { } current
        ? $"制御モード: {(int)current} ({current.ToDisplayName()})"
        : "制御モード: 読み取れませんでした");

    if (mode != ControlMode.Velocity)
    {
        Console.WriteLine("速度モードへ変更します...");
        if (motor.SetMode(ControlMode.Velocity))
        {
            Console.WriteLine("変更してフラッシュに保存しました");
        }
        else
        {
            Console.Error.WriteLine("モード変更に失敗しました");
            return 1;
        }
    }

    // --- 3. 回す ------------------------------------------------------------
    Console.WriteLine();
    Console.WriteLine($"=== {Turns:0.0} 回転させます（約 {Turns * 2 * Math.PI / Speed:0.0} 秒）===");
    double moved = motor.Spin(speed: Speed, turns: Turns, cancellationToken: cancellation.Token);

    Console.WriteLine();
    Console.WriteLine($"結果: {moved:0.0} 度（{moved / 360.0:0.00} 回転）動きました");
}
catch (IOException ex)
{
    // ポートが開けない、途中でUSBが抜けた、など
    Console.Error.WriteLine($"通信エラー: {ex.Message}");
    return 1;
}
catch (UnauthorizedAccessException ex)
{
    Console.Error.WriteLine($"ポート {port} を使用できません（他のソフトが開いている可能性）: {ex.Message}");
    return 1;
}

// using を抜けると自動的に失能され、通信も閉じられる
Console.WriteLine("完了。モータは安全な状態（失能）です。");
return 0;
