namespace DamiaoCan;

/// <summary>
/// モータから返るフィードバックフレーム（8バイト）を解釈したもの。
///
/// バイト割り当て:
///   [0]      上位4bit=エラー、下位4bit=CAN ID
///   [1][2]   位置 16bit
///   [3][4]   速度 12bit（[4]の上位4bitまで）
///   [4][5]   トルク 12bit（[4]の下位4bitから）
///   [6]      ドライバ温度 [℃]
///   [7]      モータ内部温度 [℃]
/// </summary>
public sealed class MotorFeedback
{
    private MotorFeedback(byte[] data)
    {
        Id = data[0] & 0x0F;
        Error = (data[0] >> 4) & 0x0F;
        PositionRaw = (data[1] << 8) | data[2];
        VelocityRaw = (data[3] << 4) | (data[4] >> 4);
        TorqueRaw = ((data[4] & 0x0F) << 8) | data[5];
        DriverTemperature = data[6];
        RotorTemperature = data[7];
    }

    /// <summary>フィードバックフレームを解釈する。8バイト未満なら null。</summary>
    public static MotorFeedback? Decode(byte[] data)
        => data.Length < 8 ? null : new MotorFeedback(data);

    /// <summary>応答したモータのCAN ID。</summary>
    public int Id { get; }

    /// <summary>エラーコード。0=正常、1=使能中、8〜14(0x8〜0xE)=故障。</summary>
    public int Error { get; }

    public int PositionRaw { get; }

    public int VelocityRaw { get; }

    public int TorqueRaw { get; }

    /// <summary>ドライバ（MOSFET）温度 [℃]。</summary>
    public int DriverTemperature { get; }

    /// <summary>モータ内部（ロータ）温度 [℃]。</summary>
    public int RotorTemperature { get; }

    /// <summary>位置 [rad]。±12.5 rad の範囲を16bitで表現している。</summary>
    public double PositionRad
        => MotorScaling.UIntToFloat(PositionRaw, -MotorScaling.PositionMax, MotorScaling.PositionMax, 16);

    /// <summary>速度 [rad/s]。</summary>
    public double VelocityRadPerSec
        => MotorScaling.UIntToFloat(VelocityRaw, -MotorScaling.VelocityMax, MotorScaling.VelocityMax, 12);

    /// <summary>トルク [N·m]。</summary>
    public double TorqueNm
        => MotorScaling.UIntToFloat(TorqueRaw, -MotorScaling.TorqueMax, MotorScaling.TorqueMax, 12);

    public override string ToString()
        => $"CAN ID={Id} 位置={PositionRad:+0.00;-0.00} rad エラー={Error} " +
           $"温度 {DriverTemperature}/{RotorTemperature}℃";
}
