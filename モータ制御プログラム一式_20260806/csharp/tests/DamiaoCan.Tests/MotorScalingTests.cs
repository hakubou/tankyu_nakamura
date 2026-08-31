namespace DamiaoCan.Tests;

/// <summary>
/// 値域の換算と、MITフレームの詰め方の検証。
/// 期待値は Python版 (damiao_can.py) の _float_to_uint / _uint_to_float / _mit_frame の実出力。
/// </summary>
public class MotorScalingTests
{
    private const double P = MotorScaling.PositionMax;

    [Theory]
    [InlineData(0.0, 32767)]
    [InlineData(1.57, 36883)]
    [InlineData(-1.57, 28651)]
    [InlineData(12.5, 65535)]
    [InlineData(-12.5, 0)]
    [InlineData(3.3333, 41505)]
    public void 実数から整数への換算がPython版と一致する(double value, int expected)
    {
        Assert.Equal(expected, MotorScaling.FloatToUInt(value, -P, P, 16));
    }

    [Theory]
    [InlineData(0, -12.5)]
    [InlineData(1, -12.499618524452583)]
    [InlineData(32767, -0.0001907377737087046)]
    [InlineData(32768, 0.0001907377737087046)]
    [InlineData(65535, 12.5)]
    [InlineData(40000, 2.7590218966964226)]
    public void 整数から実数への換算がPython版と一致する(int raw, double expected)
    {
        Assert.Equal(expected, MotorScaling.UIntToFloat(raw, -P, P, 16), 12);
    }

    [Theory]
    [InlineData(99.0, 65535)]    // 上限で頭打ち
    [InlineData(-99.0, 0)]       // 下限で頭打ち
    public void 値域外は端で丸める(double value, int expected)
    {
        Assert.Equal(expected, MotorScaling.FloatToUInt(value, -P, P, 16));
    }

    // ---- MITフレーム -------------------------------------------------------

    public static TheoryData<string, double, double, double, double, double, string> MITフレームの例 => new()
    {
        // 名前, 位置, 速度, kp, kd, トルク, Python版が生成した8バイト
        { "既定値",       0.0,      0.0,   20.0,   2.0, 0.0,  "7FFF7FF0A36667FF" },
        { "90度・低速",   1.57,     0.5,   20.0,   2.0, 0.0,  "90138210A36667FF" },
        { "逆回転・高kp", -3.14159, -1.25, 100.0,  4.5, -2.5, "5FD47AA333E655FF" },
        { "全て上限超え", 99.0,     99.0,  9999.0, 99.0, 99.0, "FFFFFFFFFFFFFFFF" },
        { "全て下限超え", -99.0,    -99.0, -5.0,   -5.0, -99.0, "0000000000000000" },
    };

    // Python版 (damiao_can.py) が使っていた値域。当時 T_MAX は 10.0 だった。
    // 現在の MotorScaling.TorqueMax は 2.0 に変更されているが、
    // 「詰め方が Python版と一致すること」は値域と独立に保証されるべきなので、
    // ここでは当時の値域を明示して固定する。
    private const double PythonPositionMax = 12.5;
    private const double PythonVelocityMax = 30.0;
    private const double PythonTorqueMax = 10.0;

    [Theory]
    [MemberData(nameof(MITフレームの例))]
    public void MITフレームがPython版と一致する(
        string name, double position, double velocity, double kp, double kd, double torque, string expected)
    {
        _ = name;
        byte[] frame = Motor.BuildMitFrame(position, velocity, kp, kd, torque,
                                           PythonPositionMax, PythonVelocityMax, PythonTorqueMax);

        Assert.Equal(8, frame.Length);
        Assert.Equal(expected, Convert.ToHexString(frame));
    }

    [Fact]
    public void 値域を省略すると現在のMotorScalingが使われる()
    {
        // T_MAX を変更したときに、既定の呼び出し口が追随していることを保証する。
        // 1 LSB = 2*TorqueMax/4095 なので、TorqueMax が小さいほど分解能が上がる。
        byte[] frame = Motor.BuildMitFrame(0.0, 0.0, 0.0, 0.0, MotorScaling.TorqueMax);

        int torqueRaw = ((frame[6] & 0x0F) << 8) | frame[7];
        Assert.Equal(4095, torqueRaw);   // 上限いっぱい
    }

    // ---- 回転量の累積 ------------------------------------------------------

    [Theory]
    [InlineData(100, 50, 50)]        // 普通に増えた
    [InlineData(50, 100, -50)]       // 普通に減った
    [InlineData(5, 65530, 11)]       // 一周して 65530 → 5（+11 であって -65525 ではない）
    [InlineData(65530, 5, -11)]      // 逆回転で一周
    public void エンコーダの一周をまたいだ差分を補正する(int current, int previous, int expected)
    {
        Assert.Equal(expected, Motor.UnwrapDelta(current, previous));
    }

    [Theory]
    [InlineData(65535, 1432.395)]
    [InlineData(32768, 716.2084284733348)]
    [InlineData(-65535, -1432.395)]
    [InlineData(12345, 269.82400663767453)]
    public void 累積エンコーダ値を角度に換算する(double totalRaw, double expectedDegrees)
    {
        // Python版は rad→deg に 57.2958 を、C#版は 180/π を使っている。
        // 差は 4回転ぶん回しても 0.001度未満（エンコーダの分解能 0.0004度より小さい）。
        Assert.Equal(expectedDegrees, Motor.RawTotalToDegrees(totalRaw), tolerance: 0.001);
    }
}
