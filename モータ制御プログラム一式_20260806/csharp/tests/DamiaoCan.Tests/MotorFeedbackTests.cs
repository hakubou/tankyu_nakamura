namespace DamiaoCan.Tests;

/// <summary>
/// フィードバックフレームの解釈。期待値は Python版 (_decode) の実出力。
/// </summary>
public class MotorFeedbackTests
{
    [Theory]
    // hex, ID, エラー, 位置raw, 速度raw, トルクraw, ドライバ温度, ロータ温度
    [InlineData("147FFF8005002D20", 4, 1, 32767, 2048, 1280, 45, 32)]
    [InlineData("040000000000191B", 4, 0, 0, 0, 0, 25, 27)]
    [InlineData("84C3507AB1233C41", 4, 8, 50000, 1963, 291, 60, 65)]
    public void フィードバックの解釈がPython版と一致する(
        string hex, int id, int error, int positionRaw, int velocityRaw, int torqueRaw,
        int driverTemp, int rotorTemp)
    {
        MotorFeedback? feedback = MotorFeedback.Decode(Convert.FromHexString(hex));

        Assert.NotNull(feedback);
        Assert.Equal(id, feedback!.Id);
        Assert.Equal(error, feedback.Error);
        Assert.Equal(positionRaw, feedback.PositionRaw);
        Assert.Equal(velocityRaw, feedback.VelocityRaw);
        Assert.Equal(torqueRaw, feedback.TorqueRaw);
        Assert.Equal(driverTemp, feedback.DriverTemperature);
        Assert.Equal(rotorTemp, feedback.RotorTemperature);
    }

    [Theory]
    [InlineData("147FFF8005002D20", -0.0001907377737087046)]
    [InlineData("040000000000191B", -12.5)]
    [InlineData("84C3507AB1233C41", 6.573777370870527)]
    public void 位置のrad換算がPython版と一致する(string hex, double expectedRad)
    {
        MotorFeedback feedback = MotorFeedback.Decode(Convert.FromHexString(hex))!;

        Assert.Equal(expectedRad, feedback.PositionRad, 12);
    }

    [Fact]
    public void 長さが8バイト未満ならnullを返す()
    {
        Assert.Null(MotorFeedback.Decode(Convert.FromHexString("147FFF80")));
        Assert.Null(MotorFeedback.Decode([]));
    }
}
