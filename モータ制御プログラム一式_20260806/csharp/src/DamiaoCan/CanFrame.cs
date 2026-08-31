namespace DamiaoCan;

/// <summary>1本のCANフレーム。</summary>
public readonly struct CanFrame
{
    public CanFrame(uint id, byte[] data, bool isExtendedId = false, bool isRemoteFrame = false)
    {
        if (data.Length > 8)
            throw new ArgumentException("CANのデータ長は8バイトまでです", nameof(data));

        Id = id;
        Data = data;
        IsExtendedId = isExtendedId;
        IsRemoteFrame = isRemoteFrame;
    }

    /// <summary>アービトレーションID（標準フレームなら11bit）。</summary>
    public uint Id { get; }

    /// <summary>データ部（0〜8バイト）。</summary>
    public byte[] Data { get; }

    public bool IsExtendedId { get; }

    public bool IsRemoteFrame { get; }

    /// <summary>データ長（DLC）。</summary>
    public int Dlc => Data.Length;

    public override string ToString() => $"ID=0x{Id:X3} DLC={Dlc} DATA={Convert.ToHexString(Data)}";
}
