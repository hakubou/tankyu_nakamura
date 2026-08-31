// 振子エンコーダ 読み取り + PC送信  【TIM1 ハードウェアエンコーダモード版】
// NUCLEO-G474RE / Arduino IDE (STM32 MCU based boards 3.0.0)
//
// ■ 8/20 指導教員のご指摘により、ソフトウェア割り込み方式から移行したもの
//   「ソフトウェア割り込みはやめたほうがいいです．遅延が大きく，4逓倍で使おうとすると
//     さらに遅延が大きくなるうえに，モータを動かしながらだとノイズによる誤動作が
//     かなり入ります．タイミングを見てTIMエンコーダカウントモードに移行するのが
//     おすすめです．」
//
//   ソフト割り込み版で「取りこぼし無し」を確認したのは【モータを止めた状態】のみ。
//   モータ稼働時のノイズ耐性は未検証だった。TIMのデジタル入力フィルタがその対策になる。
//
// ■ 配線（ソフト割り込み版から A相 だけ D9 → D7 に移動）
//   A相 = D7 (PA8) = TIM1_CH1   ... FT_a   5Vトレラント
//   B相 = D8 (PA9) = TIM1_CH2   ... FT_fda 5Vトレラント
//   5V  = 5V
//   GND = GND
//
//   ※ PA6(D12) / PA7(D11) は TT_a で5Vトレラントではない。絶対に繋がないこと。
//
// ■ カウンタが16bitである件
//   TIM1 は16bit。だが 1kHz でサンプリングして【16bitの差分】を足し込むため、
//   ラップは自然に吸収される。1ms の間に 32768 カウント（16回転）進まない限り厳密。
//   → オーバーフロー割り込みは不要。int32 累積という要件はそのまま満たされる。

#include <Arduino.h>

// ---- 立ち上げ用フラグ ----
//  true  : シリアルモニタで読めるテキスト出力（配線確認・立ち上げ用）
//  false : 本番の13バイトバイナリフレーム
bool textMode = false;

// ---- 入力フィルタの強さ（0〜15）----
//  15 で最も強い。TIM1クロック170MHz、CKD=DIV1 のとき
//  f_SAMPLING = 170MHz/32 = 5.3MHz を8回連続で同じ値、つまり約1.5µs より短いヒゲを捨てる。
//  こちらのエッジ間隔は最速時でも約120µs あるので、15 にしても信号は削られない。
//  モータ稼働時にカウントが暴れるようなら、まずここを疑う（下げる理由は基本的に無い）。
const uint32_t ENC_FILTER = 0xF;

TIM_HandleTypeDef htim_enc;

void encoderInit() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;          // エンコーダはプッシュプル出力なのでプル不要
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF6_TIM1;        // PeripheralPins.c で確認済み
  HAL_GPIO_Init(GPIOA, &g);

  htim_enc.Instance               = TIM1;
  htim_enc.Init.Prescaler         = 0;
  htim_enc.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim_enc.Init.Period            = 0xFFFF;
  htim_enc.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_enc.Init.RepetitionCounter = 0;
  htim_enc.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  TIM_Encoder_InitTypeDef e = {0};
  e.EncoderMode  = TIM_ENCODERMODE_TI12;      // TI1・TI2 の両エッジ = 4逓倍
  e.IC1Polarity  = TIM_ICPOLARITY_RISING;
  e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  e.IC1Prescaler = TIM_ICPSC_DIV1;
  e.IC1Filter    = ENC_FILTER;
  e.IC2Polarity  = TIM_ICPOLARITY_RISING;
  e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  e.IC2Prescaler = TIM_ICPSC_DIV1;
  e.IC2Filter    = ENC_FILTER;

  HAL_TIM_Encoder_Init(&htim_enc, &e);
  TIM1->CNT = 0;
  HAL_TIM_Encoder_Start(&htim_enc, TIM_CHANNEL_ALL);
}

// ---- 1 kHz サンプリング ----
HardwareTimer *sampleTimer;

volatile int32_t  count   = 0;
static   uint16_t prevCnt = 0;

volatile int32_t  s_count = 0;
volatile uint32_t s_t_us  = 0;
volatile uint16_t s_seq   = 0;
volatile bool     s_ready = false;

void onSample() {
  uint16_t now = (uint16_t)TIM1->CNT;
  count  += (int16_t)(now - prevCnt);   // ★16bitの差にキャストするのが肝。ラップを吸収する
  prevCnt = now;

  s_count = count;
  s_t_us  = micros();
  s_seq++;
  s_ready = true;
}

// CRC-8 (多項式 0x07, 初期値 0x00)  ※PC側もこれに合わせること
uint8_t crc8(const uint8_t *p, size_t n) {
  uint8_t c = 0x00;
  for (size_t i = 0; i < n; i++) {
    c ^= p[i];
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

void setup() {
  Serial.begin(921600);     // 13B x 1kHz = 130 kbit/s。115200 では足りない

  encoderInit();
  prevCnt = (uint16_t)TIM1->CNT;

  sampleTimer = new HardwareTimer(TIM6);   // 他で使われていたら TIM7 に変える
  sampleTimer->setOverflow(1000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(onSample);
  sampleTimer->resume();
}

void loop() {
  if (!s_ready) return;

  int32_t  c  = s_count;
  uint32_t t  = s_t_us;
  uint16_t sq = s_seq;
  s_ready = false;

  if (textMode) {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 50) {          // 20 Hz に間引いて表示
      lastPrint = millis();
      Serial.print(c);
      Serial.print("  ");
      Serial.println(c * 360.0 / 2048.0, 2);   // 確認用の度表示。本番では送らない
    }
  } else {
    uint8_t f[13];
    f[0]  = 0xA5;  f[1]  = 0xA5;
    f[2]  =  sq        & 0xFF;  f[3]  = (sq >>  8) & 0xFF;
    f[4]  =  t         & 0xFF;  f[5]  = (t  >>  8) & 0xFF;
    f[6]  = (t  >> 16) & 0xFF;  f[7]  = (t  >> 24) & 0xFF;
    f[8]  =  c         & 0xFF;  f[9]  = (c  >>  8) & 0xFF;
    f[10] = (c  >> 16) & 0xFF;  f[11] = (c  >> 24) & 0xFF;
    f[12] = crc8(f, 12);
    Serial.write(f, 13);
  }
}
