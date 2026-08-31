namespace DamiaoCan;

/// <summary>
/// CANバス。<see cref="SeeedCanBus"/> が USB-CAN Analyzer 向けの実装。
/// 別のアダプタを使う場合や、テストで通信を差し替える場合はこれを実装する。
/// </summary>
public interface ICanBus : IDisposable
{
    /// <summary>CANフレームを1本送る。</summary>
    void Send(CanFrame frame);

    /// <summary>CANフレームを1本受け取る。指定時間内に受け取れなければ null。</summary>
    CanFrame? Receive(TimeSpan timeout);
}
