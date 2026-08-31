# DM-J4310-2EC V1.1 を C# から動かす

研究室の方針に合わせ、Python 版（`../damiao_can.py` / `../demo_1回転.py`）を C# へ移植したものです。
ハードウェアの構成・配線・注意点は Python 版の [`../README.md`](../README.md) と同じなので、
**初めて動かす人は先にそちらを読んでください。**

---

## 必要なもの

### ハードウェア

Python 版とまったく同じです（DM-J4310-2EC V1.1 / 24V電源 / USB-CAN Analyzer V8.00 / XT30(2+2)）。

### ソフトウェア

- .NET SDK 10.0 以降
- CH340 のドライバ（USB-CAN を挿すと Windows 11 なら自動で入ることが多い）

Python も python-can も不要です。CAN アダプタのプロトコルは自前で実装してあります（後述）。

---

## 動かす手順

### 1. COMポート番号を調べる

デバイスマネージャー →「ポート (COM と LPT)」に `USB-SERIAL CH340 (COM3)` のように表示されます。

### 2. 安全確認（必ず）

- 24V電源が入っているか（**動かないときの原因はたいていこれ**）
- モータが固定されているか
- 出力軸の回転範囲に手・物・ケーブルが無いか
- 電源スイッチにすぐ手が届くか

### 3. 実行

```bash
dotnet run --project src/DemoOneTurn -- COM3
```

引数を省略すると `COM3` を使います。macOS / Linux なら `/dev/tty.usbserial-1140` のように指定してください。

通信確認 → モード確認 → 1回転、まで自動で行います。実行中に **Ctrl+C** を押すと、
その場で減速停止してから失能します（プロセスを即座に殺さないようにしてあります）。

### 4. 単体の exe にして配る場合

.NET が入っていないPCでも動く形にまとめられます。

```bash
dotnet publish src/DemoOneTurn -c Release -r win-x64 --self-contained -p:PublishSingleFile=true
```

---

## 簡単な使い方

```csharp
using DamiaoCan;

using var motor = new Motor("COM3", motorId: 4);

Console.WriteLine(motor.ReadStatus());        // 状態を読む（動かない）
motor.Spin(speed: 0.5, turns: 1.0);           // 0.5 rad/s で1回転
```

`using` を抜けると **必ず失能** されます。途中で例外が出てもモータは安全な状態（非通電）に戻ります。

### よく使う操作

| C# | Python 版 |
|---|---|
| `motor.ReadStatus()` | `m.read_status()` |
| `motor.Scan()` | `m.scan()` |
| `motor.ReadMode()` | `m.read_mode()` |
| `motor.SetMode(ControlMode.Velocity)` | `m.set_mode(3)` |
| `motor.Spin(speed: 0.5, turns: 2.0)` | `m.spin(speed=0.5, turns=2.0)` |
| `motor.Spin(speed: -0.5, turns: 1.0)` | `m.spin(speed=-0.5, turns=1.0)` |
| `motor.MitMove(deltaRad: 1.57)` | `m.mit_move(delta_rad=1.57)` |

---

## ファイル構成

| ファイル | 内容 |
|---|---|
| `src/DamiaoCan/Motor.cs` | 本体。使能/失能・状態取得・モード読み書き・速度制御・MIT制御 |
| `src/DamiaoCan/SeeedCanBus.cs` | USB-CAN Analyzer のシリアルプロトコル |
| `src/DamiaoCan/ICanBus.cs` | CANバスの抽象。別アダプタやテスト用の差し替え口 |
| `src/DamiaoCan/MotorFeedback.cs` | モータからの8バイトフィードバックの解釈 |
| `src/DamiaoCan/MotorScaling.cs` | MITフレームの値域と実数⇔整数の換算 |
| `src/DamiaoCan/ControlMode.cs` | 制御モード (1=MIT, 2=位置速度, 3=速度) |
| `src/DemoOneTurn/Program.cs` | 実行用。1回転させるデモ |
| `tests/DamiaoCan.Tests/` | 実機なしで動く検証（後述） |

---

## Python 版からの移植で変えたところ

### python-can に相当するものを自前で実装した

Python 版は `python-can` の `seeedstudio` バックエンドが USB-CAN Analyzer を扱っていましたが、
.NET には同等のライブラリがありません。そこで `SeeedCanBus.cs` に同じシリアルプロトコルを実装しています。

```
初期化   AA 55 12 [速度] [フレーム種別] [フィルタ4B] [マスク4B] [動作モード] 01 00×4 [CRC]
送受信   AA [0xC0|拡張<<5|RTR<<4|DLC] [ID 2B(標準) リトルエンディアン] [データ] 55
状態応答 AA 55 [18バイト]
```

依存は NuGet の `System.IO.Ports` だけです。

### 受信は必ず「まとめ読み」にしてある（重要）

`SerialPort.ReadByte()` は **1 回あたり約 1.16ms** かかります（macOS で実測）。
CAN フレーム 1 本はシリアル 13 バイトなので、Python 版と同じ「1 バイトずつ読む」構造で書くと
**1 本の受信に 15ms** も取られ、200Hz どころか 62Hz しか出ません。

`Read(buffer, 0, n)` でまとめ読みすれば同じ 13 バイトが 0.29ms で取れるので、
`SeeedCanBus.ReadByte()` は内部バッファに一括で貯めてから 1 バイトずつ払い出す作りにしてあります。
**ここを 1 バイト読みに戻すと制御周期が壊れます。**

実測値: 受信 15.08ms → 1.16ms、200Hz ループの実周期 16.3ms → 5.000ms。

### Ctrl+C で安全に止まるようにした

C# は既定だと Ctrl+C でプロセスが即座に終了し、`finally` の失能処理が走りません。
Python 版と同じ安全性を保つため、デモ側で Ctrl+C を捕まえて停止処理へ回しています。
`Spin()` と `MitMove()` は `CancellationToken` を受け取ります。

### 危険な入力を弾くようにした

- `Spin(speed: 0)` に回転数を指定すると、Python では所要時間がゼロ除算になります。
  C# では無限大になって**回り続けてしまう**ため、明示的に例外にしています。
- `MitMove` の速度が 0 以下の場合も同様です。

### CANバスをインターフェースにした

`Motor` は `ICanBus` に依存します。実機なしでテストできるほか、
別のCANアダプタを使う場合や1本のバスに複数台ぶら下げる場合にも差し替えられます。

### rad→deg の換算定数

Python 版は `57.2958`、C# 版は `180/π` を使っています。
差は4回転ぶん回しても 0.001度未満（エンコーダ分解能 0.0004度より小さい）で、実測に影響しません。

---

## 検証

実機が無くても、**Python 版とバイト単位で同じフレームを出すか**を自動で確認できます。

```bash
dotnet test
```

期待値は Python 版（`damiao_can.py` と python-can 4.6.1 の seeedstudio バックエンド）を
実際に走らせて採取したバイト列そのものです。80件すべて合格しています。

検証している範囲:

- アダプタの初期化フレーム（CRC 含む）
- 使能 / 失能 / 速度指令 / モード読み書き / フラッシュ保存の各パケット
- MITフレームの詰め方（値域外の頭打ちを含む）
- フィードバックの解釈と位置の rad 換算
- 受信パースの復帰（ゴミバイト・状態応答パケット・壊れたフレーム）
- エンコーダが一周したときの差分補正
- 例外や中断が起きても最後に必ず失能を送ること

**実機での動作確認はまだ行っていません。** 上記はあくまで「Python 版と同じバイト列を出す」ことの
確認までです。初回はモータを固定し、電源スイッチに手を届く状態で試してください。
