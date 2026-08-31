namespace DamiaoCan;

/// <summary>
/// パラメータ番号（RID）。0x7FF あてのパラメータ読み書きで使う。
///
/// 注意: この対応表は DM のレジスタマップに基づく。公式の上位機（達妙調試助手）が
/// 入手できていないため、番号が正しいことは「読めた値が物理的に妥当か」で検証すること。
/// 例えば <see cref="TorqueMax"/> を読んで 10.0 前後の値が返れば番号は合っている。
/// 桁外れの値や NaN が返る場合は番号違いを疑う。
/// </summary>
public static class RegisterId
{
    /// <summary>マスタ（PC側）のCAN ID。uint32。</summary>
    public const byte MasterId = 7;

    /// <summary>モータ自身のCAN ID。uint32。</summary>
    public const byte CanId = 8;

    /// <summary>指令が途切れてからモータが自動で失能するまでの時間。uint32。</summary>
    public const byte Timeout = 9;

    /// <summary>制御モード（1=MIT, 2=位置速度, 3=速度）。uint32。</summary>
    public const byte ControlMode = 10;

    /// <summary>減速比。float32。DM-J4310 は 10:1。</summary>
    public const byte GearRatio = 20;

    /// <summary>位置の値域 ±[rad]。float32。<see cref="MotorScaling.PositionMax"/> と一致すべき。</summary>
    public const byte PositionMax = 21;

    /// <summary>速度の値域 ±[rad/s]。float32。<see cref="MotorScaling.VelocityMax"/> と一致すべき。</summary>
    public const byte VelocityMax = 22;

    /// <summary>トルクの値域 ±[N·m]。float32。<see cref="MotorScaling.TorqueMax"/> と一致すべき。</summary>
    public const byte TorqueMax = 23;
}
