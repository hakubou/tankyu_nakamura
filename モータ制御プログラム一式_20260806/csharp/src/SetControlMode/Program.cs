// 制御モードを確認・変更するツール。
//
// 倒立振子では MIT モード(1) を使う。トルクを t_ff で直接与えるため。
// 速度モード(3) のままでは MIT フレーム(0x000+ID) を送っても受け付けられない。
//
// このツールはモータを回転させない。SetMode が内部で必ず失能してから書き込む。
// ただし通信には 24V 電源が必要（電源が入っていないと応答が返らない）。
//
// 実行:
//   dotnet run --project src/SetControlMode                          ← 現在のモードを表示するだけ
//   dotnet run --project src/SetControlMode -- --port /dev/cu.usbserial-120
//   dotnet run --project src/SetControlMode -- --set mit             ← MIT モードへ変更して保存
//   dotnet run --project src/SetControlMode -- --set velocity        ← 速度モードへ戻す

using DamiaoCan;

// 既定ポート。Windows なら "COM3"、macOS なら /dev/cu.usbserial-*
const string DefaultPort = "/dev/cu.usbserial-120";
const int DefaultMotorId = 4;

string port = DefaultPort;
int motorId = DefaultMotorId;
ControlMode? requested = null;
float? newTorqueMax = null;

for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--port" when i + 1 < args.Length:
            port = args[++i];
            break;

        case "--id" when i + 1 < args.Length:
            if (!int.TryParse(args[++i], out motorId))
            {
                Console.Error.WriteLine("--id には数値を指定してください");
                return 1;
            }
            break;

        case "--set" when i + 1 < args.Length:
            string name = args[++i].ToLowerInvariant();
            requested = name switch
            {
                "mit" or "1" => ControlMode.Mit,
                "posvel" or "2" => ControlMode.PositionVelocity,
                "velocity" or "vel" or "3" => ControlMode.Velocity,
                _ => null,
            };
            if (requested is null)
            {
                Console.Error.WriteLine($"未知のモードです: {name}（mit / posvel / velocity）");
                return 1;
            }
            break;

        case "--set-tmax" when i + 1 < args.Length:
            if (!float.TryParse(args[++i], out float tmax) || tmax is <= 0 or > 10.0f)
            {
                Console.Error.WriteLine("--set-tmax は 0 より大きく 10.0 以下で指定してください");
                return 1;
            }
            newTorqueMax = tmax;
            break;

        default:
            Console.Error.WriteLine($"不明な引数: {args[i]}");
            return 1;
    }
}

Console.WriteLine($"ポート: {port}   モータCAN ID: {motorId}");
Console.WriteLine();

try
{
    using var motor = new Motor(port, motorId);

    // --- 1. 通信確認（この時点ではモータは動かない）------------------------
    MotorFeedback? status = motor.ReadStatus();
    if (status is null)
    {
        Console.Error.WriteLine("モータが応答しません。");
        Console.Error.WriteLine("  1. 24V電源は入っていますか？（LEDが赤＝失能で点灯していれば通電済み）");
        Console.Error.WriteLine("  2. CAN_H / CAN_L の結線とGND共通化を確認してください");
        Console.Error.WriteLine($"  3. モータのCAN IDは本当に {motorId} ですか？");
        return 1;
    }

    Console.WriteLine($"通信OK  {status}");

    // --- 2. 現在の制御モード ------------------------------------------------
    ControlMode? current = motor.ReadMode();
    Console.WriteLine(current is { } mode
        ? $"現在の制御モード: {(int)mode} ({mode.ToDisplayName()})"
        : "現在の制御モード: 読み取れませんでした");
    Console.WriteLine();

    // --- 3. 変更 ------------------------------------------------------------
    if (requested is not { } target)
    {
        Console.WriteLine("（--set を指定していないので変更しません）");
    }
    else if (current == target)
    {
        Console.WriteLine($"すでに {target.ToDisplayName()} モードです。変更は不要です。");
    }
    else
    {
        Console.WriteLine($"{target.ToDisplayName()} モードへ変更し、フラッシュに保存します...");

        if (!motor.SetMode(target))
        {
            Console.Error.WriteLine("モード変更に失敗しました。書き込みが反映されていません。");
            return 1;
        }

        // 読み返して定着を確認する。フラッシュ保存の成否は電源再投入まで確定しない
        ControlMode? after = motor.ReadMode();
        Console.WriteLine(after == target
            ? $"変更しました: {(int)target} ({target.ToDisplayName()})"
            : $"警告: 読み返した値が一致しません（{after}）");

        Console.WriteLine();
        Console.WriteLine("★ 電源を入れ直してから、もう一度このツールを --set 無しで実行し、");
        Console.WriteLine("  フラッシュ保存が効いているか必ず確認してください。");
    }

    // --- 3.5 T_MAX の変更 ---------------------------------------------------
    //
    // T_MAX=10 N·m を 12bit で割ると 1 LSB = 4.9 mN·m で、この振子の制御トルクに対して粗すぎる。
    // 下げると分解能が上がるが、指令できる最大トルクも同時に下がる。
    //
    // 安全上の順序: 先にモータ側を書き換え、あとで MotorScaling を合わせる。
    // 逆順にすると「コードは小さい T_MAX、モータは大きい T_MAX」となり、
    // 同じ指令値がモータ側で数倍のトルクとして解釈されて危険。
    if (newTorqueMax is { } targetTorqueMax)
    {
        Console.WriteLine($"T_MAX を {targetTorqueMax} N·m に変更し、フラッシュに保存します...");

        if (!motor.WriteParameterSingle(RegisterId.TorqueMax, targetTorqueMax))
        {
            Console.Error.WriteLine("T_MAX の書き込みに失敗しました。値は変わっていません。");
            return 1;
        }

        Console.WriteLine($"モータ側の T_MAX を {targetTorqueMax} N·m に変更しました。");
        Console.WriteLine();
        Console.WriteLine($"★ 次に MotorScaling.TorqueMax を {targetTorqueMax} に書き換えてビルドし直してください。");
        Console.WriteLine("  それまでは指令トルクが実際より小さく出ます（安全側にズレるだけなので危険はありません）。");
        Console.WriteLine("★ 書き換え後は必ずトルク校正をやり直してください。");
        Console.WriteLine();
    }

    // --- 4. スケーリング定数をモータの実レジスタ値と照合する ----------------
    //
    // MotorScaling の定数はデータシート由来の推定値。実レジスタとズレていると
    // トルク指令が黙って定数倍狂うので、必ず実機の値と突き合わせる。
    Console.WriteLine();
    Console.WriteLine("スケーリング定数の照合（MotorScaling ⇔ モータの実レジスタ）:");

    bool allMatch = true;
    allMatch &= Compare("P_MAX", RegisterId.PositionMax, MotorScaling.PositionMax, "rad");
    allMatch &= Compare("V_MAX", RegisterId.VelocityMax, MotorScaling.VelocityMax, "rad/s");
    allMatch &= Compare("T_MAX", RegisterId.TorqueMax, MotorScaling.TorqueMax, "N·m");

    // 減速比も読めるなら表示しておく（10:1 のはず）
    if (motor.ReadParameterSingle(RegisterId.GearRatio) is { } gearRatio)
        Console.WriteLine($"  減速比 = {gearRatio:0.###}:1");

    Console.WriteLine();
    Console.WriteLine(allMatch
        ? "→ 一致しました。MotorScaling はこのまま使えます。"
        : "→ 不一致があります。MotorScaling.cs の定数を実レジスタ値に書き換えてください。");

    Console.WriteLine();
    Console.WriteLine($"  KP_MAX = {MotorScaling.KpMax}（レジスタからは読めないため未検証）");
    Console.WriteLine($"  KD_MAX = {MotorScaling.KdMax}（同上）");

    // 実レジスタ値と定数を比べて表示する。読めなければ警告して不一致扱いにする
    bool Compare(string label, byte rid, double constant, string unit)
    {
        float? actual = motor.ReadParameterSingle(rid);
        if (actual is not { } value)
        {
            Console.WriteLine($"  {label,-6} 定数={constant,-6} 実値=読み取り失敗（RID={rid}）");
            return false;
        }

        bool match = Math.Abs(value - constant) < 1e-3;
        Console.WriteLine($"  {label,-6} 定数={constant,-6} 実値={value,-8:0.###} {unit,-5} {(match ? "OK" : "★不一致★")}");
        return match;
    }
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
