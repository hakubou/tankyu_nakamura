# DM-J4310-2EC V1.1 — 電源(24V)とCAN-USB変換 調査メモ

対象モーター: DAMIAO（达妙）**DM-J4310-2EC V1.1**（QDDサーボ関節モーター / 24V版）
作成日: 2026-07-06

---

## 1. まとめ（結論）

| 項目 | 推奨・目安 |
|------|-----------|
| 電源電圧 | **24V**（動作範囲 15〜32V） |
| 電源電流（1個・実用） | **最低 3A、余裕をもって 5〜10A** を推奨 |
| 電源電流（1個・ピーク吸収） | 瞬間 7.5A まで流れうる → **10A級電源**が安心 |
| CAN-USB変換 | **DAMIAO純正 USB-to-CAN（USB2CAN）モジュール**が最も確実 |
| CANボーレート | **1 Mbps 固定**（標準フレーム） |
| 終端抵抗 | CANバス両端に **120Ω** |

---

## 2. 24V電源に必要な電流

### 2.1 モーター単体の消費電流

公式・代理店資料の数値（24V版）:

| 電流の種類 | 値 | 備考 |
|-----------|-----|------|
| **定格電流（電源側 / バス電流）** | **2.5 A** | 定格トルク3N·m時の目安 |
| **ピーク電流（電源側）** | **7.5 A** | 瞬間・加速/急停止時 |
| 定格相電流（Rated Phase Current） | 3.7 A | モーター巻線に流れる電流 |
| ピーク相電流（Peak Phase Current） | 7.2 A | 巻線ピーク |

> ⚠️ 「相電流（phase current）」と「電源電流（bus current）」は別物です。
> 電源（ACアダプタ/安定化電源）の容量選定には、**電源側の電流（定格2.5A / ピーク7.5A）** を使います。

### 2.2 電力に換算

- 定格: 24V × 2.5A ≒ **60 W**
- ピーク: 24V × 7.5A ≒ **180 W**

### 2.3 電源容量の選び方（推奨）

**モーター1個の場合**
- 動作確認・デバッグ用途: **24V / 3〜5A（72〜120W）** で可
- 負荷をかけた運用: 突入・加速でピークが出るため **24V / 10A（240W）** クラスが安心
  - 安価な安定化電源やベンチ電源なら「24V 10A」表記のものが定番

**複数個（ロボットアーム等）の場合**
- 最悪ケース = ピーク電流の合計: `モーター数 × 7.5A`
  - 例）4軸アーム → 4 × 7.5A = 30A（＝ 24V 30A ≒ 720W）
  - 例）6軸アーム → 6 × 7.5A = 45A
- 実際は全軸が同時に最大ピークになることは稀なので、
  現実的には **「定格合計 ＋ 余裕」** で設計することが多い:
  - 例）6軸 → 定格 6 × 2.5A = 15A → **24V 20A（480W）** 程度を選定し、大電流の瞬間はコンデンサ/電源のマージンで吸収
- **重要（公式ガイダンス）**: 複数モーターでは**専用の電源分配基板（Power Board）を用意し、制御PC/Jetson等の電源とモーター電源を分離**すること。

### 2.4 その他の電気的注意

- 過電圧保護: 推奨 **32V以下**
- 低電圧保護: 推奨 **15V以上**
- 過電流保護: 推奨 **9.8A以下**
- 温度保護: ドライバ 120℃ / コイル 推奨100℃以下
- 48V版も存在（動作範囲15〜52V）。24V版とは別品番なので購入時に要確認。

---

## 3. CAN-USB変換（上位機との接続）

### 3.1 通信仕様

- 制御インターフェース: **CAN @ 1 Mbps（標準フレーム、ボーレート固定）**
- パラメータ設定インターフェース: **UART @ 921600 bps**
- 制御モード: MITモード / 速度モード / 位置モード（台形加減速対応）

### 3.2 変換機の選択肢

#### (A) DAMIAO純正 USB-to-CAN（USB2CAN）モジュール ★推奨
- モーターと同時購入できる純正デバッグツール
- 超小型（約 **39 × 18 mm**）、64KB オンボードバッファ、CANアナライザ機能内蔵
- ボーレート 1000 kbps〜（可変）、タイマ送信対応
- Windows用「**DAMIAO デバッグアシスタント（上位机 / Debugging Assistant）**」で
  ID設定・モード切替・モニタリングが可能
- USB↔UART変換も兼ねるため、パラメータ設定（921600bps）にも使える

#### (B) 汎用USB-CANアダプタ
- **CANable 2.0 / candleLight系**（Linuxのsocketcanで `can0` として使える）
  - `slcan` またはネイティブ candleLight ファームで動作
- **CAN FD対応アダプタ**（LeRobot/OpenArms構成では CAN FD を使用）
- 中華系「USB-CAN / USB2CAN アナライザ」も可（ドライバ・ボーレート設定に注意）

### 3.3 物理接続（純正モジュールの例）

DAMIAOのデバッグツールを使う典型的な配線:

1. **電源＋CAN通信端子**: モーター側の **XT30(2+2)-F** コネクタケーブルで
   USB-to-CANツールへ接続（電源とCAN-H/CAN-Lが一体化したコネクタ）
2. **デバッグ用シリアル（UART）**: **GH1.25 3ピン**ケーブルでシリアルポートを接続
3. **24V電源**: モーターへ別途供給（XT30側の電源ラインへ）
4. USBをPCへ接続 → デバッグアシスタントを起動 → 該当シリアルポートを選択 → 通電

> ⚠️ **H/L反転の注意**: 一部ホスト基板（例: Seeed reComputer Mini）では
> CAN-H / CAN-L のピン配置がモーター側と逆になっている。接続不良時は H/L の入替を確認。

### 3.4 CANバス配線の基本

- **CAN-H / CAN-L / GND** の3線を各モーターへデイジーチェーン接続
- バスの**両端に120Ωの終端抵抗**（片側はアダプタ内蔵の場合あり）
- 複数モーターは **CAN IDを重複しないよう個別設定**（純正ツールで設定）
  - OpenArms/LeRobotの慣例: 送信ID `0x0N` / 受信ID `0x1N`（N=関節番号）

### 3.5 Linux（socketcan）での設定例（LeRobot/OpenArms）

```bash
# can-utils をインストール
sudo apt-get install can-utils

# CAN FD（推奨 / OpenArms）
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

# 通常CAN（FDなし）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 疎通確認
candump can0
```

- Windowsは純正「デバッグアシスタント（上位机）」でGUI操作が容易
- LeRobotのDamiao対応は**現状Linuxのみ**（OpenArms用CANアダプタのドライバがLinux専用）

---

## 4. 購入時のチェックリスト

- [ ] モーターの電圧版（**24V** or 48V）を確認
- [ ] 24V電源: 1個なら **24V 5〜10A**、多軸なら電流を積算して選定
- [ ] **電源分配基板**（多軸時）と制御系電源の分離
- [ ] **USB-to-CANモジュール**（純正 or CANable等）
- [ ] **XT30(2+2)** 電源+CANケーブル、**GH1.25 3ピン** UARTケーブル
- [ ] **120Ω終端抵抗**（バス両端）
- [ ] Windows: デバッグアシスタント / Linux: can-utils・socketcan

---

## 5. 参考文献

- [DAMIAO 公式製品ページ DM-J4310-2EC V1.1](https://damiao.enactic.ai/products/hardware/dm-j4310-2ec-v1.1/)
- [DM-J4310-2EC V1.1 減速モーター ユーザーマニュアル V1.0 (PDF, Seeed)](https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf)
- [DM-J4310-2EC V1.1 Gear Motor User Manual V1.0 (PDF, sharingwin)](https://sharingwin.com/wp-content/uploads/2025/09/DM-J4310-2EC-V1.1-Gear-Motor-User-Manual-V1.0.pdf)
- [Damiao Series Motors — Seeed Studio Wiki](https://wiki.seeedstudio.com/damiao_series/)
- [Damiao Motors and CAN Bus — Hugging Face / LeRobot](https://huggingface.co/docs/lerobot/damiao)
- [DAMIAO Brushless Servo Joint Motor Debugger USB to CAN MIT Module — OpenELAB](https://openelab.io/products/damiao-brushless-servo-usb-can)
- [DAMIAO CAN Debugger USB-to-CAN — Foxtech](https://store.foxtech.com/can-debugger-usb-to-can-communication-debug-tool-for-mit-servo-motors/)
- [DAMIAO DM-J4310-2EC — AIFITLAB（仕様・24V/48V版）](https://aifitlab.com/products/damiao-dm-j4310-2ec-v1-1-servo-motor)
- [USB to CAN FD Converter (Canable 2.0ベース) — Amazon](https://www.amazon.com/Converter-Adapter-Based-Canable-Supports/dp/B0F9F9J3WN)

> 注: 電流値・接続仕様は代理店/版により表記差があります。実機導入時は必ず付属の
> 最新ユーザーマニュアルと安全定格を確認してください。
