using System.Diagnostics;

namespace DamiaoCan.Tests;

/// <summary>
/// USB-CAN Analyzer のシリアルプロトコルの検証。
///
/// 期待値は、Python版が実際に生成したバイト列をそのまま使っている
/// （python-can 4.6.1 の seeedstudio バックエンドを実行して採取）。
/// ここが一致していれば、アダプタに届くバイト列は Python 版と同一。
/// </summary>
public class SeeedProtocolTests
{
    private static string Hex(byte[] bytes) => Convert.ToHexString(bytes);

    // ---- 初期化フレーム ----------------------------------------------------

    [Theory]
    [InlineData(1_000_000, "AA55120101000000000000000000010000000015")]
    [InlineData(500_000, "AA55120301000000000000000000010000000017")]
    public void 初期化フレームがPython版と一致する(int bitrate, string expected)
    {
        byte[] frame = SeeedCanBus.BuildInitFrame(bitrate, CanFrameType.Standard, CanOperationMode.Normal);

        Assert.Equal(20, frame.Length);
        Assert.Equal(expected, Hex(frame));
    }

    [Fact]
    public void 初期化フレームのCRCは先頭2バイトを除く総和の下位8bit()
    {
        byte[] frame = SeeedCanBus.BuildInitFrame(1_000_000, CanFrameType.Standard, CanOperationMode.Normal);

        byte sum = 0;
        for (int i = 2; i < 19; i++) sum += frame[i];

        Assert.Equal(sum, frame[19]);
    }

    [Fact]
    public void 対応していないビットレートは弾く()
    {
        Assert.Throws<ArgumentException>(
            () => SeeedCanBus.BuildInitFrame(123_456, CanFrameType.Standard, CanOperationMode.Normal));
    }

    // ---- 送信パケット ------------------------------------------------------

    public static TheoryData<string, uint, string, string> 送信パケットの例 => new()
    {
        // 名前, CAN ID, データ, Python版が生成したシリアルパケット
        { "使能",           0x004, "FFFFFFFFFFFFFFFC", "AAC80400FFFFFFFFFFFFFFFC55" },
        { "失能",           0x004, "FFFFFFFFFFFFFFFD", "AAC80400FFFFFFFFFFFFFFFD55" },
        { "速度+0.5",       0x204, "0000003F",         "AAC404020000003F55" },
        { "速度-0.5",       0x204, "000000BF",         "AAC40402000000BF55" },
        { "速度0",          0x204, "00000000",         "AAC404020000000055" },
        { "モード読み出し", 0x7FF, "0400330A00000000", "AAC8FF070400330A0000000055" },
        { "モード書き込み", 0x7FF, "0400550A03000000", "AAC8FF070400550A0300000055" },
        { "フラッシュ保存", 0x7FF, "0400AA0100000000", "AAC8FF070400AA010000000055" },
    };

    [Theory]
    [MemberData(nameof(送信パケットの例))]
    public void 送信パケットがPython版と一致する(string name, uint canId, string dataHex, string expected)
    {
        _ = name;
        var frame = new CanFrame(canId, Convert.FromHexString(dataHex));

        Assert.Equal(expected, Hex(SeeedCanBus.BuildSerialPacket(frame)));
    }

    [Fact]
    public void 拡張フレームはIDを4バイトで送る()
    {
        var frame = new CanFrame(0x12345678, [0x01, 0x02], isExtendedId: true);

        // AA [C0|拡張(0x20)|DLC2 = E2] [ID 4バイトLE] [データ] 55
        Assert.Equal("AAE2785634120102" + "55", Hex(SeeedCanBus.BuildSerialPacket(frame)));
    }

    [Fact]
    public void リモートフレームはビットが立つ()
    {
        var frame = new CanFrame(0x004, [], isRemoteFrame: true);

        Assert.Equal("AAD0040055", Hex(SeeedCanBus.BuildSerialPacket(frame)));
    }

    [Fact]
    public void データが8バイトを超えると弾く()
    {
        Assert.Throws<ArgumentException>(() => new CanFrame(0x004, new byte[9]));
    }

    // ---- 受信パース --------------------------------------------------------

    /// <summary>与えたバイト列を順に返す読み出し関数。尽きたら -1（タイムアウト扱い）。</summary>
    private static SeeedCanBus.ByteReader Feed(string hex)
    {
        byte[] bytes = Convert.FromHexString(hex);
        int index = 0;
        return _ => index < bytes.Length ? bytes[index++] : -1;
    }

    private static CanFrame? Parse(string hex)
        => SeeedCanBus.ReceiveCore(Feed(hex), long.MaxValue, Stopwatch.Frequency);

    [Fact]
    public void 標準フレームを受信できる()
    {
        // モータ4番からのフィードバック
        CanFrame? frame = Parse("AAC80400147FFF8005002D2055");

        Assert.NotNull(frame);
        Assert.Equal(0x004u, frame!.Value.Id);
        Assert.False(frame.Value.IsExtendedId);
        Assert.Equal("147FFF8005002D20", Hex(frame.Value.Data));
    }

    [Fact]
    public void 拡張フレームを受信できる()
    {
        CanFrame? frame = Parse("AAE278563412010255");

        Assert.NotNull(frame);
        Assert.Equal(0x12345678u, frame!.Value.Id);
        Assert.True(frame.Value.IsExtendedId);
        Assert.Equal("0102", Hex(frame.Value.Data));
    }

    [Fact]
    public void 状態応答パケットは読み飛ばして次のフレームを返す()
    {
        // AA 55 + 18バイト（状態応答）→ 続く実フレームが取れること
        string status = "AA55" + new string('0', 36);
        CanFrame? frame = Parse(status + "AAC404020000003F55");

        Assert.NotNull(frame);
        Assert.Equal(0x204u, frame!.Value.Id);
        Assert.Equal("0000003F", Hex(frame.Value.Data));
    }

    [Fact]
    public void 先頭のゴミバイトは読み飛ばして同期する()
    {
        CanFrame? frame = Parse("001122" + "AAC404020000003F55");

        Assert.NotNull(frame);
        Assert.Equal(0x204u, frame!.Value.Id);
    }

    [Fact]
    public void 終端バイトが違うフレームは捨てて次を探す()
    {
        // 1本目の終端が 0x00 で壊れている。2本目が返るべき
        CanFrame? frame = Parse("AAC4040200000000" + "00" + "AAC404020000003F55");

        Assert.NotNull(frame);
        Assert.Equal("0000003F", Hex(frame!.Value.Data));
    }

    [Fact]
    public void 途中で切れたフレームはnullを返す()
    {
        Assert.Null(Parse("AAC80400147FFF"));
    }

    [Fact]
    public void 何も来なければnullを返す()
    {
        Assert.Null(Parse(""));
    }
}
