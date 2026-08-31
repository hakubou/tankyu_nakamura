namespace DamiaoCan;

/// <summary>
/// MITモードのフレームに値を詰めるときの値域と、実数⇔整数の線形変換。
/// 値域は DM-J4310-2EC V1.1 の仕様に合わせてある。
/// </summary>
public static class MotorScaling
{
    /// <summary>位置の値域 ±[rad]。</summary>
    public const double PositionMax = 12.5;

    /// <summary>速度の値域 ±[rad/s]。</summary>
    public const double VelocityMax = 30.0;

    /// <summary>
    /// トルクの値域 ±[N·m]。
    ///
    /// 2026/08/12 に既定の 10.0 から 2.0 へ変更した（モータの T_MAX レジスタ RID=23 も同時に変更済み）。
    /// 理由: 12bit なので 1 LSB = 2*TorqueMax/4095。10.0 のときは 4.9 mN·m あり、
    /// 倒立振子の制御トルク（平衡近傍では 10〜30 mN·m 程度）に対して粗すぎた。
    /// 2.0 なら 0.98 mN·m となり分解能が5倍になる。
    /// 一方でスイングアップのピーク（摩擦 0.147 + 加速分で 0.65 N·m 程度）に対して
    /// 3倍の余裕が残るため、飽和の心配もない。
    ///
    /// この値はモータのレジスタと必ず一致していなければならない。
    /// ズレていると指令トルクが黙って定数倍狂う。<see cref="Motor.VerifyScaling"/> で検証できる。
    /// </summary>
    public const double TorqueMax = 2.0;

    /// <summary>位置ゲイン kp の上限。</summary>
    public const double KpMax = 500.0;

    /// <summary>速度ゲイン kd の上限。</summary>
    public const double KdMax = 5.0;

    /// <summary>実数を、指定ビット数の整数に線形変換する。範囲外は端で丸める。</summary>
    public static int FloatToUInt(double value, double lo, double hi, int bits)
    {
        value = Math.Clamp(value, lo, hi);
        return (int)((value - lo) * ((1 << bits) - 1) / (hi - lo));
    }

    /// <summary>上の逆変換。</summary>
    public static double UIntToFloat(int value, double lo, double hi, int bits)
        => (double)value / ((1 << bits) - 1) * (hi - lo) + lo;
}
