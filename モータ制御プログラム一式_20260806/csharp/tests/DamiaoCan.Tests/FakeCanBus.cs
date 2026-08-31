namespace DamiaoCan.Tests;

/// <summary>
/// 実機の代わりに使うCANバス。送ったフレームを記録し、
/// <see cref="RespondWith"/> に設定した規則でモータの応答を返す。
/// </summary>
internal sealed class FakeCanBus : ICanBus
{
    private readonly Queue<CanFrame> _incoming = new();

    /// <summary>これまでに送られたフレーム。</summary>
    public List<CanFrame> Sent { get; } = [];

    /// <summary>送信フレームに対する応答を決める。null を返せば無応答。</summary>
    public Func<CanFrame, CanFrame?>? RespondWith { get; set; }

    public bool Disposed { get; private set; }

    public void Send(CanFrame frame)
    {
        Sent.Add(frame);
        if (RespondWith?.Invoke(frame) is { } reply) _incoming.Enqueue(reply);
    }

    public CanFrame? Receive(TimeSpan timeout)
        => _incoming.Count > 0 ? _incoming.Dequeue() : null;

    /// <summary>次に Receive で返すフレームを直接積む。</summary>
    public void EnqueueIncoming(CanFrame frame) => _incoming.Enqueue(frame);

    public void Dispose() => Disposed = true;

    /// <summary>送信フレームを "ID:データ16進" の形にして比較しやすくする。</summary>
    public static string Describe(CanFrame frame)
        => $"{frame.Id:X3}:{Convert.ToHexString(frame.Data)}";
}
