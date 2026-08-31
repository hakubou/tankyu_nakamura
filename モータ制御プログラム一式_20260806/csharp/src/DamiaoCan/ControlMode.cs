namespace DamiaoCan;

/// <summary>モータの制御モード。パラメータ番号10に書き込む値。</summary>
public enum ControlMode
{
    /// <summary>MIT（位置・速度・kp・kd・トルクの5要素を指定する）。</summary>
    Mit = 1,

    /// <summary>位置速度モード。</summary>
    PositionVelocity = 2,

    /// <summary>速度モード（速度[rad/s]だけを指定する。一定速で回すならこれが簡単）。</summary>
    Velocity = 3,
}

public static class ControlModeExtensions
{
    /// <summary>表示用の名前。未知の値なら "不明"。</summary>
    public static string ToDisplayName(this ControlMode mode) => mode switch
    {
        ControlMode.Mit => "MIT",
        ControlMode.PositionVelocity => "POS-VEL",
        ControlMode.Velocity => "VELOCITY",
        _ => "不明",
    };
}
