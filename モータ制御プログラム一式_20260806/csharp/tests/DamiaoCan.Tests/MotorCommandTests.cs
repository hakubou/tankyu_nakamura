namespace DamiaoCan.Tests;

/// <summary>
/// Motor が送るCANフレームの順序と内容の検証。実機の代わりに <see cref="FakeCanBus"/> を使う。
/// </summary>
public class MotorCommandTests
{
    private const int MotorId = 4;

    private static (Motor motor, FakeCanBus bus) CreateMotor()
    {
        var bus = new FakeCanBus();
        return (new Motor(bus, MotorId), bus);
    }

    /// <summary>モータのフィードバックフレームを組み立てる。</summary>
    private static CanFrame Feedback(int positionRaw, int error = 0, int driverTemp = 30, int rotorTemp = 30)
        => new((uint)MotorId,
        [
            (byte)((error << 4) | MotorId),
            (byte)((positionRaw >> 8) & 0xFF),
            (byte)(positionRaw & 0xFF),
            0, 0, 0,
            (byte)driverTemp, (byte)rotorTemp,
        ]);

    /// <summary>パラメータ読み出しへの応答を組み立てる。</summary>
    private static CanFrame ModeReply(ControlMode mode)
        => new(0x7FF, [MotorId, 0x00, 0x33, 10, (byte)mode, 0x00, 0x00, 0x00]);

    // ---- 使能／失能 --------------------------------------------------------

    [Fact]
    public void 使能はモータIDあてに送る()
    {
        var (motor, bus) = CreateMotor();
        using (motor) motor.Enable();

        Assert.Equal("004:FFFFFFFFFFFFFFFC", FakeCanBus.Describe(bus.Sent[0]));
    }

    [Fact]
    public void 失能はモータIDあてに送る()
    {
        var (motor, bus) = CreateMotor();
        using (motor) motor.Disable();

        Assert.Equal("004:FFFFFFFFFFFFFFFD", FakeCanBus.Describe(bus.Sent[0]));
    }

    [Fact]
    public void 破棄するときは必ず失能させる()
    {
        var (motor, bus) = CreateMotor();
        motor.Enable();
        motor.Dispose();

        Assert.Equal("004:FFFFFFFFFFFFFFFD", FakeCanBus.Describe(bus.Sent[^1]));
    }

    [Fact]
    public void 外から渡されたバスは破棄しない()
    {
        var bus = new FakeCanBus();
        using (var motor = new Motor(bus, MotorId)) motor.Enable();

        Assert.False(bus.Disposed);
    }

    [Fact]
    public void バスの所有を渡した場合は一緒に閉じる()
    {
        var bus = new FakeCanBus();
        using (var motor = new Motor(bus, MotorId, ownsBus: true)) motor.Enable();

        Assert.True(bus.Disposed);
    }

    // ---- 速度指令 ----------------------------------------------------------

    [Theory]
    [InlineData(0.5, "204:0000003F")]
    [InlineData(-0.5, "204:000000BF")]
    [InlineData(0.0, "204:00000000")]
    public void 速度指令は0x200プラスIDあてに単精度で送る(double speed, string expected)
    {
        var (motor, bus) = CreateMotor();
        using (motor) motor.SetVelocity(speed);

        Assert.Equal(expected, FakeCanBus.Describe(bus.Sent[0]));
    }

    // ---- 状態の取得 --------------------------------------------------------

    [Fact]
    public void 状態取得は失能コマンドを送って応答を読む()
    {
        var bus = new FakeCanBus { RespondWith = _ => Feedback(positionRaw: 40000, driverTemp: 45, rotorTemp: 32) };
        using var motor = new Motor(bus, MotorId);

        MotorFeedback? status = motor.ReadStatus();

        Assert.NotNull(status);
        Assert.Equal(40000, status!.PositionRaw);
        Assert.Equal(45, status.DriverTemperature);
        // 送ったのは失能（動かさない安全なコマンド）だけであること
        Assert.All(bus.Sent, frame => Assert.Equal("004:FFFFFFFFFFFFFFFD", FakeCanBus.Describe(frame)));
    }

    [Fact]
    public void 応答が無ければ状態取得はnullを返す()
    {
        var (motor, _) = CreateMotor();
        using (motor)
        {
            Assert.Null(motor.ReadStatus(tries: 2));
        }
    }

    [Fact]
    public void スキャンは応答したIDだけを返す()
    {
        var bus = new FakeCanBus
        {
            // 3番と6番だけが応答する
            RespondWith = frame => frame.Id is 3 or 6
                ? new CanFrame(frame.Id, [(byte)frame.Id, 0, 0, 0, 0, 0, 30, 30])
                : null,
        };
        using var motor = new Motor(bus, MotorId);

        Dictionary<int, MotorFeedback?> found = motor.Scan(1, 8);

        Assert.Equal([3, 6], found.Keys.Order());
    }

    // ---- 制御モード --------------------------------------------------------

    [Fact]
    public void モード読み出しは0x7FFあてに送り応答を解釈する()
    {
        var bus = new FakeCanBus { RespondWith = _ => ModeReply(ControlMode.Velocity) };
        using var motor = new Motor(bus, MotorId);

        ControlMode? mode = motor.ReadMode();

        Assert.Equal(ControlMode.Velocity, mode);
        Assert.Equal("7FF:0400330A00000000", FakeCanBus.Describe(bus.Sent[0]));
    }

    [Fact]
    public void モード読み出しは無関係な応答を無視する()
    {
        var bus = new FakeCanBus();
        bus.EnqueueIncoming(Feedback(positionRaw: 100));         // ただのフィードバック
        bus.EnqueueIncoming(ModeReply(ControlMode.Mit));         // 本命
        using var motor = new Motor(bus, MotorId);

        Assert.Equal(ControlMode.Mit, motor.ReadMode());
    }

    [Fact]
    public void モード変更は失能から書き込み読み返し保存の順に送る()
    {
        var bus = new FakeCanBus { RespondWith = _ => ModeReply(ControlMode.Velocity) };
        using var motor = new Motor(bus, MotorId);

        bool ok = motor.SetMode(ControlMode.Velocity);

        Assert.True(ok);
        Assert.Equal(
        [
            "004:FFFFFFFFFFFFFFFD",   // 念のため失能
            "7FF:0400550A03000000",   // モード書き込み（速度モード=3）
            "7FF:0400330A00000000",   // 読み返して確認
            "7FF:0400AA0100000000",   // フラッシュ保存
        ], bus.Sent.Select(FakeCanBus.Describe));
    }

    [Fact]
    public void 読み返しが一致しなければモード変更は失敗を返す()
    {
        // 書き込んでも MIT のままを返してくるモータ
        var bus = new FakeCanBus { RespondWith = _ => ModeReply(ControlMode.Mit) };
        using var motor = new Motor(bus, MotorId);

        Assert.False(motor.SetMode(ControlMode.Velocity));
        // 失敗したのでフラッシュ保存は送らない
        Assert.DoesNotContain(bus.Sent, f => f.Data.Length > 2 && f.Data[2] == 0xAA);
    }

    [Fact]
    public void 保存しない指定なら保存コマンドを送らない()
    {
        var bus = new FakeCanBus { RespondWith = _ => ModeReply(ControlMode.Velocity) };
        using var motor = new Motor(bus, MotorId);

        Assert.True(motor.SetMode(ControlMode.Velocity, save: false));
        Assert.DoesNotContain(bus.Sent, f => f.Data.Length > 2 && f.Data[2] == 0xAA);
    }

    // ---- 回転 --------------------------------------------------------------

    [Fact]
    public void 回転は使能してから速度を送り続け最後に停止して失能する()
    {
        var (motor, bus) = CreateMotor();
        using (motor) motor.Spin(speed: 0.5, durationSeconds: 0.05, verbose: false);

        List<string> sent = [.. bus.Sent.Select(FakeCanBus.Describe)];

        Assert.Equal("004:FFFFFFFFFFFFFFFC", sent[0]);                    // 使能
        Assert.Contains("204:0000003F", sent);                            // 速度 0.5
        Assert.Equal(10, sent.Count(s => s == "204:00000000"));           // 停止指令を10回
        Assert.Equal("004:FFFFFFFFFFFFFFFD", sent[^1]);                   // 失能
        // 停止指令はすべて速度指令より後
        Assert.True(sent.LastIndexOf("204:0000003F") < sent.IndexOf("204:00000000"));
    }

    [Fact]
    public void 中断されたら残り時間を待たずに停止して失能する()
    {
        var (motor, bus) = CreateMotor();
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();

        using (motor)
        {
            // 60秒指定でも、中断済みなので即座に停止処理へ進む
            motor.Spin(speed: 0.5, durationSeconds: 60.0, verbose: false,
                       cancellationToken: cancellation.Token);
        }

        List<string> sent = [.. bus.Sent.Select(FakeCanBus.Describe)];
        Assert.DoesNotContain("204:0000003F", sent);                      // 回転指令は出ていない
        Assert.Equal(10, sent.Count(s => s == "204:00000000"));           // 停止指令は出す
        Assert.Equal("004:FFFFFFFFFFFFFFFD", sent[^1]);                   // 失能して終わる
    }

    [Fact]
    public void 回転量はフィードバックの累積から求める()
    {
        int position = 0;
        var bus = new FakeCanBus
        {
            // 速度指令のたびに 1000 ずつ進み、65536 を跨いで巻き戻る
            RespondWith = frame => frame.Id == 0x204
                ? Feedback(positionRaw: (position += 1000) & 0xFFFF)
                : null,
        };
        using var motor = new Motor(bus, MotorId);

        double degrees = motor.Spin(speed: 0.5, durationSeconds: 0.3, verbose: false);

        // 何回送れたかは実時間次第なので、向きと桁だけ確認する
        Assert.True(degrees > 0, $"正方向に進んだはずが {degrees} 度だった");
        int velocityCommands = bus.Sent.Count(f => f.Id == 0x204 && f.Data[3] != 0x00);
        Assert.Equal(Motor.RawTotalToDegrees((velocityCommands - 1) * 1000.0), degrees, 6);
    }

    [Fact]
    public void 例外が起きても必ず失能する()
    {
        var bus = new FakeCanBus();
        var motor = new Motor(bus, MotorId);

        // 使能の直後に通信が切れた状況を作る
        bus.RespondWith = _ => throw new IOException("USBが抜けた");

        Assert.Throws<IOException>(() => motor.Spin(speed: 0.5, durationSeconds: 1.0, verbose: false));
        Assert.Equal("004:FFFFFFFFFFFFFFFD", FakeCanBus.Describe(bus.Sent[^1]));
    }

    [Fact]
    public void 速度0や指定漏れは弾く()
    {
        var (motor, _) = CreateMotor();
        using (motor)
        {
            Assert.Throws<ArgumentException>(() => motor.Spin(speed: 0.0, turns: 1.0));
            Assert.Throws<ArgumentException>(() => motor.Spin(speed: 0.5));
        }
    }

    // ---- MITモード ---------------------------------------------------------

    [Fact]
    public void MIT指令はモータIDあてに送る()
    {
        var (motor, bus) = CreateMotor();
        using (motor) motor.MitCommand(position: 1.57, velocity: 0.5, kp: 20.0, kd: 2.0);

        Assert.Equal("004:90138210A36667FF", FakeCanBus.Describe(bus.Sent[0]));
    }

    // ---- 純粋トルク指令（倒立振子で使う）-----------------------------------

    [Fact]
    public void トルク指令はkpとkdのビットが必ずゼロになる()
    {
        var (motor, bus) = CreateMotor();
        using (motor) motor.TorqueCommand(0.0);

        byte[] data = bus.Sent[0].Data;

        // kp は data[3] の下位4bit と data[4]、kd は data[5] と data[6] の上位4bit
        int kp = ((data[3] & 0x0F) << 8) | data[4];
        int kd = (data[5] << 4) | (data[6] >> 4);

        Assert.Equal(0, kp);
        Assert.Equal(0, kd);
    }

    [Theory]
    // 位置・速度・kp・kd はすべて0。トルクだけが変化する。
    // 期待値は MotorScaling.TorqueMax = 2.0 のときのもの
    // （1 LSB = 4/4095 = 0.98 mN·m）。T_MAX を変えたらここも変わる。
    [InlineData(0.0, "004:7FFF7FF0000007FF")]
    [InlineData(1.0, "004:7FFF7FF000000BFF")]
    [InlineData(-1.0, "004:7FFF7FF0000003FF")]
    public void トルク指令はモータIDあてに送りトルクだけを載せる(double torque, string expected)
    {
        var (motor, bus) = CreateMotor();
        using (motor) motor.TorqueCommand(torque);

        Assert.Equal(expected, FakeCanBus.Describe(bus.Sent[0]));
    }

    [Fact]
    public void トルク指令は値域を超えても飽和して壊れない()
    {
        var (motor, bus) = CreateMotor();
        using (motor)
        {
            motor.TorqueCommand(1000.0);    // T_MAX を大きく超える
            motor.TorqueCommand(-1000.0);
        }

        // 12bit の上限・下限に張り付くこと（オーバーフローして符号が反転しないこと）
        Assert.Equal("004:7FFF7FF000000FFF", FakeCanBus.Describe(bus.Sent[0]));
        Assert.Equal("004:7FFF7FF000000000", FakeCanBus.Describe(bus.Sent[1]));
    }

    [Fact]
    public void フィードバック読み出しはパラメータ応答を読み飛ばす()
    {
        var bus = new FakeCanBus();
        bus.EnqueueIncoming(ModeReply(ControlMode.Mit));          // 0x7FF。これは無視されるべき
        bus.EnqueueIncoming(Feedback(positionRaw: 12345));        // 本命
        using var motor = new Motor(bus, MotorId);

        MotorFeedback? feedback = motor.ReadFeedback(TimeSpan.FromMilliseconds(200));

        Assert.NotNull(feedback);
        Assert.Equal(12345, feedback!.PositionRaw);
    }

    [Fact]
    public void フィードバックが来なければnullを返す()
    {
        var (motor, _) = CreateMotor();
        using (motor)
        {
            Assert.Null(motor.ReadFeedback(TimeSpan.FromMilliseconds(20)));
        }
    }

    [Fact]
    public void MIT移動は応答が無ければ電源を疑うよう促す()
    {
        var (motor, _) = CreateMotor();
        using (motor)
        {
            var ex = Assert.Throws<InvalidOperationException>(() => motor.MitMove(deltaRad: 1.57));
            Assert.Contains("24V電源", ex.Message);
        }
    }

    [Fact]
    public void MIT移動は目標へ向かって少しずつ指令を出し最後に失能する()
    {
        var bus = new FakeCanBus { RespondWith = frame => frame.Data[7] == 0xFD ? Feedback(32767) : null };
        using var motor = new Motor(bus, MotorId);

        motor.MitMove(deltaRad: 0.05, speed: 0.5, verbose: false);

        List<CanFrame> mit = [.. bus.Sent.Where(f => f.Id == MotorId && f.Data[0] != 0xFF)];
        Assert.NotEmpty(mit);
        // 目標位置（上位2バイト）が単調に増えていること
        int[] positions = [.. mit.Select(f => (f.Data[0] << 8) | f.Data[1])];
        Assert.Equal(positions.Order(), positions);
        Assert.Equal("004:FFFFFFFFFFFFFFFD", FakeCanBus.Describe(bus.Sent[^1]));
    }
}
