// 8/26 作成: 振子を真下から振り上げ、上向き付近で BalanceControl と同じ
// LQR（キャッチ制御）に切り替える。
//
// ■ 方式：エネルギー整形則（ブランコを漕ぐのと同じ原理）
//
// model/furuta_model.py の座標系（α=0が上向き、α=πが真下）で、振子の力学的エネルギーを
//   E(α, α̇) = ½·J_pivot·α̇² + m_p·g·l_p·cosα        （J_pivot = J_p + m_p·l_p²、支点まわり）
// と定義する。E は上向き静止で最大(+m_p·g·l_p)、真下静止で最小(−m_p·g·l_p)。
//
// Lagrange法で導出すると、アームの接線方向加速度 a（≈L_r·θ̈）が振子に及ぼす効果は
//   dE/dt = −m_p·l_p·a·α̇·cosα
// となる（8/26、メンバーAがこのファイルのために手計算で導出。model/furuta_model.py の
// 導出と同じLagrangianの枠組みだが、αの式だけを取り出したもの）。
// これが常に正になるよう a を選べばエネルギーが単調に増える:
//   a = k_e·(E−E_top)·sign(α̇·cosα)      （k_e>0）
// を代入すると dE/dt = −m_p·l_p·k_e·(E−E_top)·|α̇cosα| ≥ 0（E<E_topの間は真に正）。
//
// 実装では a→トルクの厳密な変換（J_r, L_r を介す）はせず、
//   u_swing = SwingGain·(E−E_top)·sign(α̇·cosα)  を直接トルク指令として使う
// （SwingGain は経験的に調整するN·m/Jのゲイン。物理的な換算はコメントに留め、
//   実際の比例定数は実機で合わせる）。アーム自身の摩擦補償も別途加える
// （FurutaGains.FrictionPositive/Negative を流用、BalanceControlと同じ式）。
//
// ■ 開始条件：振子が真下で静止するのを自動検出する
//
// メンバーAの指摘（8/26）どおり、手で真上に持っていって離す方式は毎回の初期条件が
// ばらつく。振子は真下（α=π）が自然な安定平衡点なので、**何もしなくても
// 勝手に静止する**。これを自動検出してからゼロ点を校正する
// （BalanceControlの「真上に持っていってEnter」とは校正方法が異なる）。
//
// ■ 切り替え（キャッチ）
//
// 振子角が上向きから CatchAngleDeg 以内に入ったら、その場で
// PendulumTelemetry.FurutaObserver（BalanceControlと全く同じ、8/26にBryson重みを
// θ=0.10radへ絞って再設計したK・L）に切り替える。以降はBalanceControlと同じ
// ループ構造（実際に送ったトルクをStep()へ渡す）。
//
// 実行:
//   dotnet run --project src/SwingUp -- COM4 COM5                              ← 既定値
//   dotnet run --project src/SwingUp -- COM4 COM5 15 0.8 25                    ← 引数指定
//                                        振子側 モータ側 SwingGain 振り上げ中の
//                                                        トルク上限[N·m] キャッチ角度[deg]
//
// 安全:
//   ・アーム角が開始角から ArmAngleLimitDeg を超えたら即座にトルク0＋失能して終了
//   ・キャッチ後、振子角が上向きから PendulumAngleAbortDeg を超えたら同様に終了
//     （再度の振り上げは行わない。まず1回の往復を確実に成功させることを優先）
//   ・振り上げがSwingUpTimeoutSecondsを超えても上がらなければタイムアウトで終了
//   ・Ctrl+C／電源スイッチでいつでも止められる状態で実行すること

using System.Diagnostics;
using DamiaoCan;
using PendulumTelemetry;

// ★8/28追加: --history フラグが立っているときだけコード更新履歴を表示する。
// 位置引数（COMポートやSwingGainなど）の並びに影響しないよう、判定後に配列から取り除く。
bool showHistory = args.Contains("--history");
// ★8/28: 試行後の自動制動（--no-damp）は削除したが、コマンド履歴に残った
// --no-damp をそのまま実行してもエラーにならないよう、引数としては読み飛ばす。
args = args.Where(a => a != "--history" && a != "--no-damp").ToArray();

string pendulumPort = args.Length > 0 ? args[0] : "COM4";
string motorPort = args.Length > 1 ? args[1] : "COM5";
double swingGain = args.Length > 2 ? double.Parse(args[2]) : 30.0;          // N·m / J
// ★8/27の53回の試行で、15.0だとキャッチ角度25°付近での押す力が約0.019 N·mしかなく、
// 摩擦の残差（静摩擦0.154 N·mの補償残り約10%）と拮抗して「頭打ち」になることが分かった。
// 30.0にしたところ5回中5回成功したため、こちらを既定値とする。
// α=180°（真下）ではGain=30で0.81 N·mとなり、下のトルク上限0.8 N·mにちょうど当たる。
// ★8/28 15:3x: 0.8→1.5。8/28の測定29回で失敗5件が全て「アーム角が±140°に到達」だった
// （振子角ガード到達・頭打ちは0件）。原因を調べると、アームを引き戻す項
// uArmHold = -5.0×(|θ|-30°) は**アーム角39°の時点で既に上限0.8 N·mに張り付いており**、
// バネ定数を上げても閾値を変えても出力は0.8のまま増えなかった（＝引き戻す力の不足は
// ゲインではなく上限が原因）。
// 一方、振り上げの押す力 uEnergy = 30×(E−E_top) は式で決まり最大0.81 N·m（真下）なので、
// **上限を1.5に上げても振り上げの挙動は変わらず、引き戻し側だけが約1.9倍になる。**
// MotorScaling.TorqueMax=2.0 の内側なので、その先のクランプは従来どおり効く。
//
// ★8/28 15:38 再修正: 1.5 → 1.0。
// 1.5にした直後の11回は失敗ゼロだったが、続く17回で4件失敗し、失敗の型は
// 上限0.8のときと同じ「アーム角±140°到達」だった。通算28回で成功率85.7%となり、
// 上限0.8の82.8%とほぼ変わらない。**最初の11回連続成功は偶然の連続で、
// 統計的な改善はほとんど無かった。** 速くなった分アームの運動量も増えるため、
// 引き戻す力が強くなった効果と相殺されていたと考えられる。
// **加えてメンバーAから「速度が速くて怖い」との報告があった。** 実際に操作している人の
// 危険の実感を、数字の上での小さな差より優先する。
// 0.8では引き戻し項が39°で頭打ちになる問題が残るため、中間の1.0とした。
double swingTorqueLimitNm = args.Length > 3 ? double.Parse(args[3]) : 1.0;
double catchAngleDeg = args.Length > 4 ? double.Parse(args[4]) : 25.0;
// ★8/28追加: 最小射影法のオフセットc。0以下なら従来の発見的な条件（|α|<25°かつ|α̇|<3.0）を使う。
// 既定は0＝従来動作のまま。V(x)の値は従来動作でも常に画面に表示するので、
// 何回か走らせてVを観察してからcを決められる。
double minProjC = args.Length > 5 ? double.Parse(args[5]) : 0.0;
bool useMinProjection = minProjC > 0.0;
// ★8/28追加: 保持がこの秒数続いたら自動終了する（毎回Ctrl+Cする手間を省く）。
// 0以下を渡すと従来通りCtrl+Cか安全ガードまで続ける。
double autoStopAfterHoldSeconds = args.Length > 6 ? double.Parse(args[6]) : 5.0;
const int MotorId = 4;

// --- model/furuta_model.py PARAMS（8/26時点）と同じ値。★PARAMSを変えたらここも変えること★ ---
const double PendulumMassKg = 0.024;      // m_p
const double PendulumCgRadiusM = 0.05735; // l_p（回転軸→重心）
const double PendulumInertiaCgKgM2 = 3.332e-5; // J_p（重心まわり）
const double GravityMPerS2 = 9.80665;
const double PivotInertiaKgM2 = PendulumInertiaCgKgM2 + PendulumMassKg * PendulumCgRadiusM * PendulumCgRadiusM; // J_pivot
const double EnergyTopJ = PendulumMassKg * GravityMPerS2 * PendulumCgRadiusM; // E_top（上向き静止のエネルギー）
const double SwingFrictionCompScale = 0.9; // FurutaObserverのCompScaleと同じ値


// ★8/26追加: 振り上げ中、アームには位置を戻す力が一切無く「押しっぱなし」になり、
// 0.4秒でアーム角±140°の安全ガードに到達する事象が実機で発生した（振子が真下から
// 水平に近づくまでの約1/4周期の間、sign(α̇cosα)が変わらないため）。
//
// ★8/26さらに改訂: 常時弱いバネ（Kp=1.0）で押さえたところ暴走は止まったが、
// 今度は振子の振れ幅が11.6秒間まったく育たなくなった。バネがエネルギー整形則の
// ポンピング動作そのものを打ち消していたと判断。**普段は何もせず、
// ArmHoldThresholdDegを超えたときだけ強く押し戻す**方式に変更。
// 常時ONの弱い減衰（ArmHoldKdAlways）だけは残し、帯域内での過剰な暴れを抑える。
const double ArmHoldThresholdDeg = 30.0;
const double ArmHoldKpOutside = 5.0;   // N·m/rad（閾値を超えた分にだけ掛かる）
const double ArmHoldKdAlways = 0.02;   // N·m·s/rad（常時、ごく弱く）

const double ArmAngleLimitDeg = 140.0;

// ★8/28: 60.0→70.0。8/27〜8/28のログでキャッチ失敗の多くが「+61.2°」「-60.3°」など
// 境界のすぐ外側で止まっており、LQRが収束途中の一時的なオーバーシュートを
// ガードが早めに打ち切っていた可能性がある。70°でも1回転（±180°）には余裕がある。
const double PendulumAngleAbortDeg = 70.0;
const double SwingUpTimeoutSeconds = 45.0;

// ★8/28追加: エネルギー整形則は設計上、E→E_topに近づくほど押す力が0に近づく
// （自己収束のための意図した挙動）。ただし角度・速度のキャッチ条件を満たす
// タイミングとうまく噛み合わないと、押す力が弱いまま何秒も足踏みして
// タイムアウト（頭打ち）に至ることがある（8/28実機で観測）。
// 対策: 押す力（|uEnergy|）が弱い状態がWeakPushGraceSecondsを超えて続いたら、
// 超えた秒数に応じてswingGainを徐々に増やす。強い状態に戻れば即リセットする。
// ★8/28追加: 指導教員ご指摘の最小射影法（Minimum Projection Method）。
//
//   W(x) = min( V(x), (E(x) - Er)² + c )
//
// V(x)=xᵀPx はLQRのリヤプノフ関数（PはFurutaGains.RiccatiP＝リカッチ方程式の解）、
// E(x)は現在の力学的エネルギー、Erは目標エネルギー（上向き静止）、cは設計オフセット。
// Vが小さい側ではLQR、そうでない側ではエネルギー整形則、という切り替えがminそのもので
// 決まるため、従来の「|α|<25°かつ|α̇|<3.0」という発見的な条件と違って
// 閉ループの安定性が理論的に保証される。
//
// ★8/28 11:40頃 バグ修正: 当初、(E-Er)²（真下でも7.3e-4しかない）をVと桁を揃える
// つもりで480倍していたが、これが誤りだった。真下でのV(x)≒480に対して枝の値が
// 480×1+c≒580となり、開始直後（真下で静止、V=479.9）から「V≤枝」が常に成立して
// しまい、押し始める前に即キャッチへ誤って切り替わる不具合が実機で発生した
// （スケール自体が真下のVと一致するよう選んだせいで、+cの分だけ枝が必ずVを
// 上回ってしまう構造だった）。
//
// 生の物理単位（J²）に戻すと、(E-Er)²は真下でも0.024gの振子では最大7.3e-4しかなく、
// c（100前後を想定）に対して無視できるほど小さい。つまり枝はほぼcで一定になり、
// 実質的には「V(x)がcを下回ったら切り替える」という単純なV閾値判定になる。
// これは指導教員の式をそのまま（余計な変形をせず）実装した結果であり、
// このFuruta振子ではエネルギー項の寄与が物理的に小さいというだけで、式自体は
// 忠実に実装している。将来エネルギースケールを合わせたくなった場合は、
// 「真下の値と一致させる」以外の基準（cより十分小さく保つ等）を使うこと。
//
//   W(x) = min( V(x), (E(x) - Er)² + c )


// ★cの決め方（8/28、c=100で失敗して修正）:
//
// 当初「成功例のアーム角57°ならV=75、失敗例の105°ならV=208」と見積もってc=100としたが、
// これは実測ではなく机上で構成した状態での値で、完全に誤っていた。
// trial_log.csvに実際に記録された成功時のVは2.5〜7.3、失敗時が110.3。桁が違う。
//
// またPの最小固有値は1.16e-3と極端に小さく、V(x)には「ほとんど増えない方向」がある。
// そのため角度が大きくてもθ・θ̇・α̇の組み合わせ次第でVは小さくなり得る。実際、
// α=150°でもVは最小74まで下がるため、c=100だと振り上げ開始直後に誤ってキャッチへ
// 切り替わった（実機で2回発生）。
//
// cは必ず「保証される角度」に直してから決めること:
//
//     V(x) ≤ c  ⟹  |α| ≤ √(c · RiccatiPinvAlphaAlpha)  [rad]
//
//   c=  5 → |α| ≤ 39.0°
//   c= 10 → |α| ≤ 55.1°     ← 実測の成功時V（2.5〜7.3）を全て含み、失敗時110.3を明確に弾く
//   c= 20 → |α| ≤ 78.0°
//   c=100 → |α| ≤ 174.4°    ← 事実上ほぼ無条件。使ってはいけない
const double MinProjDefaultC = 10.0;

const double WeakPushTorqueFraction = 0.3;  // |uEnergy|がswingTorqueLimitNmのこの割合を下回ったら「弱い」
const double WeakPushGraceSeconds = 3.0;    // 弱い状態がこの秒数続くまでは何もしない
const double WeakPushRampPerSecond = 0.25;  // 猶予を超えた秒数1秒あたり+25%
const double WeakPushRampMax = 3.0;         // 最大でswingGainの3倍まで

// ★8/27追加: キャッチ判定に速度条件を追加するための閾値。
// LQR設計（model/design_and_simulate.py）のBrysonのα̇許容偏差max_dev[3]=3.0rad/sに合わせた。
// エネルギー整形則 a = k_e*(E-E_top)*sign(α̇cosα) は、E>E_topだとdE/dt<0になり
// 自動的にエネルギーを削る側に働く（導出済み）。角度条件（±catchAngleDeg）だけで
// 切り替えると、E_topを大きく超えた高速な状態のまま真上を通過してしまい、
// LQRの最大トルクでは止めきれずに振子角ガードへ到達する事例が8/27に発生
// （振り上げ1.1秒→キャッチ0.1秒後にガード到達）。速度が高いうちは切り替えを
// 見送り、法則自身にエネルギーを削らせてから次の通過を待つ。
const double CatchAlphaDotMaxRadPerSec = 3.0;

// ★8/27追加: このプログラムに手を入れるたびに、先頭に1行足すこと（新しい順）。
// ★8/27 15:34〜: 日付だけでなく時刻も記録する（メンバーAの指示）。
// 下4件のうち上3件は打刻し忘れていたため、trial_log.csvの実行時刻から
// 逆算した概算（ビルド直後に走った試行の直前の空き時間から推定）。
string[] CodeChangeLog =
[
    "2026-08-28 14:40頃: ★試行後の自動制動（エネルギー整形則でE_refをE_bottomにする方式）を13:56〜14:32に4回試みたが、最後まで安定させられず機能ごと削除した（メンバーAの判断）。半自動（保持で自動終了→アームを初期位置へ戻す→静止待ち）でも十分スムーズなため。経緯: (1)アーム引き戻しバネに制動トルクが潰され無効 (2)その対策でバネを弱めた結果アームが暴走しマイコンに衝突（8/26に判明済みの暴走モードの再発。バネこそが保護だった） (3)ゲイン不足で完了判定に到達できず毎回20秒待ち (4)速度推定が立ち上がる前の値で通電判断し「振幅0.4°」と誤認して再び暴走。この過程で得た惰走予測ガードとアーム初期位置復帰は有用なので残してある。--no-damp は引数として読み飛ばすだけにした",
    "2026-08-28 11:55頃: ★最小射影法のcの目安を100→10へ訂正。100は机上で構成した状態から見積もった誤った値で、実測（成功時V=2.5〜7.3、失敗時110.3）とは桁が違っていた。Pの最小固有値が1.16e-3と小さくVには増えにくい方向があるため、c=100ではα=150°でも通ってしまい振り上げ開始直後に誤キャッチしていた。cが保証する角度 |α|≤√(c·(P⁻¹)[1,1]) を起動時に表示するようにした",
    "2026-08-28 15:38頃: ★トルク上限を1.5→1.0へ戻した。1.5直後の11回は失敗ゼロだったが続く17回で4件失敗し、失敗の型は上限0.8のときと同じ「アーム角±140°到達」。通算28回で成功率85.7%となり上限0.8の82.8%とほぼ変わらず、最初の11回連続成功は偶然の連続だった（11回で「狙いどおり」と結論づけたのは早計。95%信頼区間の下限は74%しかなかった）。速くなった分アームの運動量も増えるため引き戻す力の増加と相殺されていたと考えられる。加えてメンバーAから「速度が速くて怖い」との報告があり、実際に操作している人の危険の実感を数字上の小さな差より優先した。0.8では引き戻し項が39°で頭打ちになる問題が残るため中間の1.0とした",
    "2026-08-28 15:25頃: 振り上げ中のトルク上限の既定値を0.8→1.5に変更。8/28の測定29回で失敗5件が全て「アーム角±140°到達」だったことへの対策。当初はアーム保持のバネを遠方で強める案を検討したが、数値を確認するとuArmHold=-5.0×(|θ|-30°)はアーム角39°の時点で既に上限0.8に張り付いており、バネを強くしても閾値を変えても出力は増えないことが判明（引き戻し力の不足はゲインではなく上限が原因だった）。一方uEnergyは式で決まり最大0.81 N·mなので、上限を上げても振り上げの挙動は変わらず引き戻し側だけが約1.9倍になる",
    "2026-08-28 15:04頃: ★★惰走予測ガードを撤回し、8/28朝までの単純な±140°ガードに戻した。実測アーム速度17.8rad/sに対し式は惰走131°を予測するが、それならアーム角81°から212°へ到達しているはずで、朝まで±140°のみで運用して衝突は一度も起きていない。式が「クーロン摩擦だけで減速する自由なアーム」を仮定しており、実際は振子との結合でエネルギーが移り減速機の粘性摩擦もあるため大幅に過大予測していた。実測で検証しないまま入れた結果、条件固定35回で失敗8件が全てこのガードの誤停止（他の失敗はゼロ＝本来100%）、直後の13回でも3件誤停止。再度入れる場合は「トルクを切ってから何度流れるか」を速度別に実測してからにすること。未使用になった定数も削除",
    "2026-08-28 14:56頃: ★惰走ガードの安全率を1.5→1.0に修正。条件固定35回の測定で失敗8件が全てこのガードの誤停止（それ以外の失敗はゼロ＝本来100%だった）。発動位置が80.1〜87.4°に集中しており、振り上げ中にアームが80°付近を高速通過する正常な動作を毎回止めていた。安全率1.5だと予測が上限60°に張り付き80+60=140で成立してしまう。実測式どおりの1.0なら同じ速度で41.5°、80+41.5=121.5と余裕がある。停止メッセージにアーム速度[rad/s]も出すようにして、実際の速度を切り分けられるようにした",
    "2026-08-28 14:20頃: ★バグ修正: Ctrl+Cで終了すると、静止待ちと3秒カウントダウンをそのまま通過して制御ループだけが即終了し、経過秒0.0の偽の「頭打ち」がtrial_log.csvに1行記録されていた（実際に6行混入。成功率の集計が狂う）。静止待ち・カウントダウンをcancellation対応にし、制御が始まる前に終わった回は記録しないようにした。既存の6行にも中断理由欄へ「集計から除外すること」と印を入れた",
    "2026-08-28 14:11頃: ★★アーム角ガードに惰走の予測を追加。マイコンは配線の都合で可動範囲外へ移動できないため、ソフト側で守る必要がある。トルクを0にしてもアームは J_r·ω²/(2·τ_c) だけ流れ、実測値(J_r=2.13e-3, τ_c=0.147)では ω=10rad/s で41.5°、ω=15rad/s で93.4°にもなる。つまり「±140°で止める」だけでは実際には181°や233°まで到達しており、これが衝突の直接原因。現在速度から惰走を予測し、今止めれば140°以内に収まるうちに停止するようにした（安全率1.5、予測上限60°）。低速時は従来どおり139°付近まで使えるので振り上げ性能は犠牲にしない。最大アーム角に達するのは折り返し点＝低速時なので相性がよい。制動ループ側の60°ガードにも同じ予測を入れた",
    "2026-08-28 14:07頃: ★★安全上の重大な修正: 14:01の変更でアームが暴走し、マイコンに衝突させてしまった（メンバーAの報告）。原因は、制動が効かない対策としてアーム引き戻しバネを弱め（閾値30→70°・Kp5.0→0.5）、さらにDampGainを300に上げたこと。バネ項は8/26に判明していた暴走モード「位置を戻す力が無いとsign(α̇cosα)が変わらない間ずっと同じ向きに押し続け0.4秒で±140°に達する」への保護そのものであり、それを自分で外していた。加えて成功保持の直後は振子が真上＝エネルギー最大で、uEnergy=7.68 N·mと即飽和していた。対策: (1)アーム保持を振り上げと同じ実績値に戻す (2)DampGain 300→15 (3)制動のトルク上限0.8→0.3 (4)振子が振幅120°以下に落ちてくるまで無通電で待つ (5)±140°とは別に制動専用ガード60°で早めに打ち切る",
    "2026-08-28 13:44頃: ★バグ修正: アームを戻す→静止判定、の順番が逆だった。振子の静止判定を先に済ませた後でアームを動かしていたため、その動きで振子が再び揺れた状態のまま振り上げが始まっていた（メンバーAの指摘）。アームを戻す動作を先頭に移動し、その後で振子・アームの静止をまとめて判定するようにした",
    "2026-08-28 13:37頃: 2回目以降の試行で、アームを最初の位置（armHomeRad）へMitMoveで戻してから始めるようにした。従来は試行のたびに「その時点の位置」を0°として測り直していたため、繰り返すうちにアームの絶対位置がじわじわズレていた。戻した後は振子の真下静止待ちと同じ窓判定で「0°付近で静止したか」を確認する（メンバーAの指示）。振子が真下で静止した後にアームを動かす順にして、キャッチ失敗直後などまだ振子が揺れている状態でいきなり動かすリスクを避けた",
    "2026-08-28 13:29頃: 1試行終わるごとにプログラムごと終了する代わりに、そのまま「真下で静止待ち」に戻って次の試行を始めるようにした（メンバーAの指示、毎回コマンドを打ち直す手間を省くため）。安全確認・機器接続は最初の1回だけ。Ctrl+Cを押したときだけプログラム全体を終了する",
    "2026-08-28 13:24頃: ★バグ修正: 表示ループの終了条件がabortReasonのみを見ておりautoStoppedを見ていなかったため、5秒保持で自動終了してもCtrl+Cするまで数値が出続けて次の試行に移れなかった（メンバーAの報告）。表示ループの条件に!autoStoppedを追加",
    "2026-08-28 13:20頃: 保持がautoStopAfterHoldSeconds（第7引数、既定5秒）続いたら自動終了するようにした。毎回Ctrl+Cする手間を省くため（メンバーAの指示）。安全ガードによる中断とは区別して表示する",
    "2026-08-28 13:14頃: PendulumEncoderReaderが元々持っていたCRCエラー数・欠落数・受信フレーム数を画面に表示するようにした。USB-Cハブ経由に配線を変更した直後、振り上げ中の振子角が100msごとに±150°近く飛ぶ異常な挙動が発生（推定α̇は1rad/s未満なので物理的にありえないジャンプ）。通信起因かどうかを切り分けるために追加",
    "2026-08-28 11:45頃: コード更新履歴の表示を既定で非表示にし、--historyを付けたときだけ表示するようにした（メンバーAの指示、毎回出ると邪魔）",
    "2026-08-28 11:41頃: ★バグ修正: 最小射影法のエネルギー項に掛けていた480倍のスケールを削除（生のJ²単位に戻した）。真下のVとほぼ一致する値にスケールしたせいで、開始直後（真下静止）から誤ってキャッチへ切り替わる不具合が実機で発生（実行して即キャッチ失敗）。この振子ではエネルギー項が物理的に無視できるほど小さく、実質V(x)≤cの判定になる",
    $"2026-08-28 11:25頃: ★指導教員ご指摘の最小射影法を実装。第6引数にc（目安{MinProjDefaultC:0}）を渡すと、従来の「|α|<25°かつ|α̇|<3.0」に代えて W(x)=min(V(x), (E-Er)²+c) で切り替える。引数なし（c=0）なら従来動作。V(x)は常時画面右端に表示",
    "2026-08-28 11:20頃: model/design_and_simulate.pyがリカッチ方程式の解PをFurutaGains.RiccatiPとして出力するようにした（Kは従来と一致することを確認済み）",
    "2026-08-28 11:08頃: 振子角の中断ガードPendulumAngleAbortDegを±60°→±70°に緩和。キャッチ失敗の多くが境界のすぐ外側（60〜62°）で止まっていたため",
    "2026-08-28 11:02頃: WeakPushRampPerSecondを毎秒+50%→+25%に変更（メンバーAの指示、急激すぎることへの懸念）",
    "2026-08-28 10:58頃（概算）: 押す力（|uEnergy|）が弱い状態がWeakPushGraceSeconds（3秒）を超えて続いたら、swingGainを徐々に強めるようにした（最大3倍）。キャッチ直前で押しが弱いまま足踏みして頭打ちになる事象への対策。強い状態に戻れば即リセットする",
    "2026-08-28 10:40頃: trial_log.csvにSwingGain・振り上げトルク上限・キャッチ角度の3列を追加。どの設定で回した試行かが後から分かるようにした（8/27の53行にはこの3列が空のまま入っている）",
    "2026-08-28 10:38頃: SwingGainの既定値を15.0から30.0へ変更。引数なしで実行しても成功する設定にした",
    $"2026-08-27 15:26頃: キャッチ判定に速度条件を追加。|α̇|<{CatchAlphaDotMaxRadPerSec:0.0}rad/sのときだけ切り替える（高速で真上を通過する際にガードへ到達する問題への対策）",
    "2026-08-27 15:21頃: 起動時にこの変更履歴を表示するようにした",
    "2026-08-27 15:13頃（概算）: キャッチ切替時にオブザーバへ実測値（角度・アームの実速度・振子の推定速度）を種として渡すようにした。切替直後にu_totalが飽和しやすかった問題への対策",
    "2026-08-27 15:10頃（概算）: 振り上げ時間（振り上げ開始からキャッチ切替までの秒数）もtrial_log.csvへ記録するようにした",
    "2026-08-27 14:55頃（概算）: 試行結果を src/SwingUp/trial_log.csv へ自動記録するようにした（成功/キャッチ失敗/頭打ちを自動判定）",
];

Console.WriteLine("=== スイングアップ（振り上げ→キャッチ）===");
Console.WriteLine();
Console.WriteLine($"SwingGain: {swingGain:0.0} N·m/J ／ 振り上げ中のトルク上限: {swingTorqueLimitNm:0.00} N·m ／ キャッチ角度: ±{catchAngleDeg:0}°");
Console.WriteLine(autoStopAfterHoldSeconds > 0
    ? $"自動終了: 保持{autoStopAfterHoldSeconds:0}秒で自動終了（第7引数で変更可、0以下で無効）"
    : "自動終了: 無効（Ctrl+Cか安全ガードまで継続）");
Console.WriteLine(useMinProjection
    ? $"キャッチ判定: ★最小射影法（指導教員ご指摘）★ c={minProjC:0.0}／W(x)=min(V(x), (E-Er)²+c)"
      + $"\n  ＊この振子ではエネルギー項が桁で無視できるほど小さく、実質 V(x)≤c の判定になる"
      + $"\n  ＊この c で保証される角度: |α| ≤ {Math.Sqrt(minProjC * FurutaGains.RiccatiPinvAlphaAlpha) * 180.0 / Math.PI:0.0}°"
    : $"キャッチ判定: 従来の発見的な条件（|α|<{catchAngleDeg:0}° かつ |α̇|<{CatchAlphaDotMaxRadPerSec:0.0}rad/s）"
      + $"／最小射影法を使うには第6引数にcを渡す（目安 {MinProjDefaultC:0}）。V(x)は画面右端に常時表示");
Console.WriteLine();

// ★8/27追加: このプログラムに手を入れるたびに、ここへ1行足すこと。
// 何回も試行を繰り返す中で「今動いているのはどのコードか」を実行結果と
// 突き合わせられるようにするため（trial_log.csvのタイムスタンプと対応させる）。
// ★8/28追加: 毎回出ると邪魔なので、--historyを付けたときだけ表示する。
if (showHistory)
{
    Console.WriteLine("--- コード更新履歴（新しい順） ---");
    foreach (string entry in CodeChangeLog)
    {
        Console.WriteLine($"  ・{entry}");
    }
    Console.WriteLine();
}
else
{
    Console.WriteLine("（コード更新履歴は非表示。見るには --history を付けて実行）");
    Console.WriteLine();
}
Console.WriteLine("★★★ 実行前に必ず確認 ★★★");
Console.WriteLine();
Console.WriteLine("  1. 振子モジュールがベース板に確実に固定されているか（M4・M3・側面レールすべて）");
Console.WriteLine("  2. モータ本体が土台に固定されているか（反力が出ます）");
Console.WriteLine("  3. アームが自由に±140°、振子が自由に1回転弱動ける状態か（ケーブル・手・物が可動範囲に無いか）");
Console.WriteLine("  4. 電源スイッチにすぐ手が届くか（Ctrl+Cでも停止できます）");
Console.WriteLine($"  5. アーム角が開始角から±{ArmAngleLimitDeg:0}°、キャッチ後に振子角が上向きから±{PendulumAngleAbortDeg:0}°を");
Console.WriteLine("     超えたら自動でトルク0＋失能します（それでも手を離さないこと）");
Console.WriteLine();
Console.Write("上記すべて確認したら Enter（中止は Ctrl+C）: ");
Console.ReadLine();

using var timerResolution = WindowsTimerResolution.Begin(1);

Console.WriteLine($">> 振子エンコーダ: {pendulumPort} (921600 baud) / モータ: {motorPort}");

using var encoder = new PendulumEncoderReader(pendulumPort);
using var motor = new Motor(motorPort, MotorId);

using var cancellation = new CancellationTokenSource();
Console.CancelKeyPress += (_, e) =>
{
    e.Cancel = true;
    cancellation.Cancel();
};

var pumpTask = Task.Run(() => encoder.PumpUntilCancelled(cancellation.Token), cancellation.Token);

MotorFeedback? initialStatus = motor.ReadStatus();
if (initialStatus is null)
{
    Console.Error.WriteLine("モータが応答しません。24V電源とケーブルを確認してください。");
    cancellation.Cancel();
    return 1;
}
Console.WriteLine($">> モータ接続OK。現在位置 {initialStatus.PositionRad * 180.0 / Math.PI:0.0} 度。");

// ★8/28追加: アームの「0°」は最初に測ったこの位置に固定する。毎試行ここへ戻ってから
// 次を始めることで、試行を重ねるうちにアームの絶対位置がじわじわズレていくのを防ぐ
// （以前は試行のたびに「その時点の位置」を0°として測り直していた）。
double armHomeRad = initialStatus.PositionRad;
bool isFirstTrial = true;

Console.WriteLine(">> 振子エンコーダからのフレーム待機中...");
var waitStart = DateTime.UtcNow;
while (encoder.Latest is null)
{
    if ((DateTime.UtcNow - waitStart).TotalSeconds > 5)
    {
        Console.Error.WriteLine("振子エンコーダからフレームが届きません。マイコンの電源・COMポート・ボーレート(921600)を確認してください。");
        cancellation.Cancel();
        return 1;
    }
    Thread.Sleep(50);
}
Console.WriteLine(">> フレーム受信OK。");

// ★8/28追加: 5秒保持で自動終了した後もプログラムごと終了せず、そのままこの
// 「真下で静止待ち」から次の試行を始める（毎回コマンドを打ち直す手間を省くため）。
// Ctrl+Cで外側のcancellationが立ったときだけループを抜けてプログラムを終了する。
while (!cancellation.IsCancellationRequested)
{

// ★8/28修正: アームを先に戻してから、振子・アームの静止判定を行う順番にした。
// 逆順（静止判定の後でアームを戻す）だと、判定が終わった直後にアームを動かすことになり、
// その動きで振子が再び揺れた状態のまま振り上げが始まってしまっていた（メンバーAの指摘）。
if (!isFirstTrial)
{
    Console.WriteLine();
    Console.WriteLine(">> アームを初期位置へ戻しています。");
    double currentArmRad = motor.ReadStatus()?.PositionRad ?? armHomeRad;
    double deltaToHomeRad = armHomeRad - currentArmRad;
    if (Math.Abs(deltaToHomeRad) > 0.01) // 約0.6°未満ならそもそも動かさない
    {
        motor.MitMove(deltaToHomeRad, speed: 1.0, verbose: false, cancellationToken: cancellation.Token);
    }
}

// --- 振子が真下で静止するのを自動検出 ---（8/26、手で真上に持っていく方式をやめた）
// ★8/28: アームを戻した後に判定するので、その動きで振子が揺れていてもここで待ちきる。
Console.WriteLine();
Console.WriteLine(">> 振子が真下でぶら下がって静止するまで待っています。触らないでください。");
{
    var settleSw = Stopwatch.StartNew();
    var window = new Queue<(double t, double deg)>();
    const double SettleWindowSec = 1.0;
    const double SettleRangeDeg = 2.0;
    const double MaxWaitSec = 20.0;
    bool settled = false;
    while (!settled && settleSw.Elapsed.TotalSeconds < MaxWaitSec && !cancellation.IsCancellationRequested)
    {
        if (encoder.Latest is { } f)
        {
            double now = settleSw.Elapsed.TotalSeconds;
            window.Enqueue((now, PendulumEncoderReader.CountToDegrees(f.Count)));
            while (window.Count > 0 && now - window.Peek().t > SettleWindowSec) window.Dequeue();
            if (now > SettleWindowSec && window.Count >= 5)
            {
                double mn = double.MaxValue, mx = double.MinValue;
                foreach (var (_, deg) in window) { if (deg < mn) mn = deg; if (deg > mx) mx = deg; }
                if (mx - mn < SettleRangeDeg) settled = true;
            }
        }
        Thread.Sleep(50);
    }
    Console.WriteLine(settled ? ">> 静止を確認しました。" : ">> 静止を確認できませんでしたが続行します。");
}

// ★8/28追加: 「静止したか」だけでなく「0°（armHomeRad）に近い位置で静止したか」を
// 振子の真下静止待ちと同じ窓判定で確認する（メンバーAの指示）。アームを戻す動作自体は
// 上ですでに完了しているので、ここは移動後の落ち着きを確認するだけ。
if (!isFirstTrial)
{
    Console.WriteLine(">> アームが0°付近で静止するまで待っています。");
    var armSettleSw = Stopwatch.StartNew();
    var armWindow = new Queue<(double t, double deg)>();
    const double ArmSettleWindowSec = 0.5;
    const double ArmSettleRangeDeg = 2.0;
    const double ArmZeroToleranceDeg = 3.0;
    const double ArmSettleMaxWaitSec = 10.0;
    bool armSettled = false;
    while (!armSettled && armSettleSw.Elapsed.TotalSeconds < ArmSettleMaxWaitSec && !cancellation.IsCancellationRequested)
    {
        if (motor.ReadStatus() is { } st)
        {
            double now = armSettleSw.Elapsed.TotalSeconds;
            double deg = (st.PositionRad - armHomeRad) * 180.0 / Math.PI;
            armWindow.Enqueue((now, deg));
            while (armWindow.Count > 0 && now - armWindow.Peek().t > ArmSettleWindowSec) armWindow.Dequeue();
            if (now > ArmSettleWindowSec && armWindow.Count >= 3)
            {
                double mn = double.MaxValue, mx = double.MinValue;
                foreach (var (_, d) in armWindow) { if (d < mn) mn = d; if (d > mx) mx = d; }
                if (mx - mn < ArmSettleRangeDeg && Math.Abs(deg) < ArmZeroToleranceDeg) armSettled = true;
            }
        }
        Thread.Sleep(50);
    }
    Console.WriteLine(armSettled ? ">> アームの静止（0°付近）を確認しました。" : ">> アームの静止（0°付近）を確認できませんでしたが続行します。");
}
isFirstTrial = false;

// ゼロ点校正：真下（α=π）を基準に、上向き(α=0)に相当するカウントを逆算する。
// count - zeroOffsetCount = 1024（2048カウント/回転の半分＝π）が真下になるように置く。
int bottomCount = encoder.Latest!.Value.Count;
int zeroOffsetCount = bottomCount - 1024;
double armZeroRad = armHomeRad; // ★8/28変更: 毎回測り直すのではなく、最初の位置に固定する
Console.WriteLine($">> ゼロ点を校正しました（真下の生カウント {bottomCount} を α=180° として、アーム {armZeroRad * 180.0 / Math.PI:0.0}度 を0度とします）。");

Console.WriteLine();
// ★8/28追加: Ctrl+C後はここで抜ける（従来は静止待ちとカウントダウンを通過してしまい、
// 経過秒0.0の偽の「頭打ち」がtrial_log.csvに1行記録されていた）。
if (cancellation.IsCancellationRequested) break;

Console.WriteLine(">> 3秒後に振り上げを開始します。");
for (int i = 3; i >= 1 && !cancellation.IsCancellationRequested; i--)
{
    Console.WriteLine($"   {i}...");
    Thread.Sleep(1000);
}
Console.WriteLine(">> 振り上げ開始。Ctrl+Cでいつでも停止できます。");
Console.WriteLine();
Console.WriteLine(" 状態     振子角    アーム角  | 推定α̇     u_swing/u_total | obs更新Hz  最大dt[ms] |    V(x)  E側の枝  | エンコーダ受信状態");

double armAngleLimitRad = ArmAngleLimitDeg * Math.PI / 180.0;

object stateLock = new();
double sharedArmDeg = 0.0;
double sharedPendulumDeg = 0.0;
string sharedPhase = "振り上げ";
double sharedLastTorque = 0.0;
double sharedV = 0.0;              // ★8/28追加: リヤプノフ関数V(x)=xᵀPx（表示・c決定用）
double sharedEnergyBranch = 0.0;   // ★8/28追加: 最小射影法のエネルギー側の枝
double? catchV = null;             // ★8/28追加: キャッチ切替の瞬間のV（ログ用）
FurutaObserver observer = new();
long observerUpdateCount = 0;
string? abortReason = null;
bool caught = false; // ★8/27追加: 終了時のログ分類（頭打ち/キャッチ失敗/成功）に使うため外側スコープへ
double? swingUpSeconds = null; // ★8/27追加: 振り上げ開始からキャッチ切替までの秒数（ログ用）
Stopwatch? holdSw = null;      // ★8/28追加: キャッチ後、保持を続けている時間の計測用
bool autoStopped = false;      // ★8/28追加: 保持成功による自動終了かどうか（安全ガードによる中断と区別）

double maxDtMs = 0.0;
object dtLock = new();

double maxAbsArmDeg = 0.0;
double maxAbsPendulumDeg = 0.0;
object extremaLock = new();

motor.Enable();
var controlStartSw = Stopwatch.StartNew();

var controlTask = Task.Run(() =>
{
    try
    {
        var sw = Stopwatch.StartNew();
        double lastT = sw.Elapsed.TotalSeconds;
        double appliedTorque = 0.0;
        double prevAlphaRad = 0.0;
        double alphaDotFiltered = 0.0;
        bool haveEstimate = false;
        Stopwatch? weakPushSw = null; // ★8/28追加: 押す力が弱い状態が続いている時間の計測用

        while (!cancellation.IsCancellationRequested)
        {
            motor.TorqueCommand(appliedTorque);
            MotorFeedback? arm = motor.ReadFeedback(TimeSpan.FromMilliseconds(5));
            PendulumFrame? frame = encoder.Latest;
            if (arm is null || frame is not { } f)
            {
                continue;
            }

            double now = sw.Elapsed.TotalSeconds;
            double dt = now - lastT;
            lastT = now;

            double dtMs = dt * 1000.0;
            lock (dtLock) { if (dtMs > maxDtMs) maxDtMs = dtMs; }

            double thetaRad = arm.PositionRad - armZeroRad;
            double alphaRad = PendulumEncoderReader.CountToDegrees(f.Count - zeroOffsetCount) * Math.PI / 180.0;
            double alphaWrapped = Wrap180(alphaRad * 180.0 / Math.PI);

            // ★安全ガード（常時）：アーム角
            //
            // ★8/28、ここに惰走予測 J_r·ω²/(2·τ_c) を足すガードを入れたが撤回した。
            // 理由: 式が「クーロン摩擦だけで減速する自由なアーム」を仮定しており、
            // 実態と合っていなかった。実測アーム速度17.8rad/sに対し式は惰走131°を
            // 予測するが、それならアーム角81°から212°まで到達しているはずで、
            // 8/28朝まで±140°のみで運用していて衝突は一度も起きていない。
            // 実際にはアームは振子と結合していてエネルギーが振子側へ移り、
            // 減速機の粘性摩擦もあるため、惰走はこの式よりはるかに小さい。
            // 検証しないまま入れた結果、条件固定35回の測定で失敗8件が全てこのガードの
            // 誤停止（それ以外の失敗はゼロ＝本来100%）となり、正常な振り上げ動作を
            // 毎回止めていた。
            //
            // なお8/28の衝突は、削除済みの「試行後の自動制動」が暴走してアームが
            // 0.9N·mで連続加速していた別の状況で起きたもので、通常の振り上げとは
            // 速度域が違う。その機能自体を削除したので、この予測ガードの前提も消えた。
            //
            // 再度入れる場合は、まず実機で「トルクを切ってから何度流れるか」を
            // 速度別に実測し、減速トルクを同定してからにすること。
            if (Math.Abs(thetaRad) > armAngleLimitRad)
            {
                abortReason = $"アーム角が開始角から{thetaRad * 180.0 / Math.PI:+0.0;-0.0}°（上限±{ArmAngleLimitDeg:0}°）";
                appliedTorque = 0.0;
                motor.TorqueCommand(0.0);
                break;
            }

            // ★タイムアウト（振り上げ中のみ）
            if (!caught && controlStartSw.Elapsed.TotalSeconds > SwingUpTimeoutSeconds)
            {
                abortReason = $"振り上げが{SwingUpTimeoutSeconds:0}秒以内に完了しませんでした";
                appliedTorque = 0.0;
                motor.TorqueCommand(0.0);
                break;
            }

            double displayTorque;

            if (!caught)
            {
                // --- 振り上げフェーズ：エネルギー整形則 ---
                if (dt > 0 && dt < 0.5 && haveEstimate)
                {
                    double raw = (alphaRad - prevAlphaRad) / dt;
                    alphaDotFiltered = 0.7 * alphaDotFiltered + 0.3 * raw;
                }
                prevAlphaRad = alphaRad;
                haveEstimate = true;

                // ★8/28追加: 最小射影法の2つの枝を毎周期計算する。
                // useMinProjectionがfalseでも表示用に計算しておき、cを決める材料にする。
                double energyNow = 0.5 * PivotInertiaKgM2 * alphaDotFiltered * alphaDotFiltered
                                  + PendulumMassKg * GravityMPerS2 * PendulumCgRadiusM * Math.Cos(alphaRad);
                double vLyap = LyapunovV(thetaRad, alphaWrapped * Math.PI / 180.0,
                                         arm.VelocityRadPerSec, alphaDotFiltered);
                double energyErr = energyNow - EnergyTopJ;
                double energyBranch = energyErr * energyErr + minProjC;
                lock (stateLock) { sharedV = vLyap; sharedEnergyBranch = energyBranch; }

                // 切り替え条件。最小射影法では min が V を選んだ瞬間＝LQRの領域に入った瞬間。
                bool switchToLqr = useMinProjection
                    ? vLyap <= energyBranch
                    : Math.Abs(alphaWrapped) < catchAngleDeg && Math.Abs(alphaDotFiltered) < CatchAlphaDotMaxRadPerSec;

                if (switchToLqr)
                {
                    caught = true;
                    swingUpSeconds = controlStartSw.Elapsed.TotalSeconds;
                    catchV = vLyap;
                    holdSw = Stopwatch.StartNew();
                    // ★8/27追加: オブザーバの内部状態を実測値で初期化してから渡す。
                    // 角度は巻き戻し済みの値（キャッチフェーズと同じ扱い）、速度は
                    // アームがCANフィードバックの実測値、振子が振り上げ中に推定していた
                    // alphaDotFilteredをそのまま使う。
                    observer.Seed(thetaRad, alphaWrapped * Math.PI / 180.0, arm.VelocityRadPerSec, alphaDotFiltered);
                    Console.WriteLine();
                    Console.WriteLine($">> 上向き付近（{alphaWrapped:+0.0;-0.0}°）に到達。バランス制御へ切り替えます。（振り上げ時間 {swingUpSeconds:0.0}秒、V={vLyap:0.0}）");
                    lock (stateLock) { sharedPhase = "キャッチ"; }
                    appliedTorque = 0.0; // 切り替え直後は一旦0から。次ループでStep()が計算する
                    displayTorque = 0.0;
                }
                else
                {
                    double energy = energyNow;   // ★8/28: 上の最小射影法の判定で計算済みの値を使い回す
                    double sign = Math.Sign(alphaDotFiltered * Math.Cos(alphaRad));
                    if (sign == 0) sign = 1.0;

                    // ★8/28追加: 押す力が弱い状態が続いたらswingGainを徐々に強める。
                    double weakThresholdNm = WeakPushTorqueFraction * swingTorqueLimitNm;
                    double rawUEnergyAbs = Math.Abs(swingGain * (energy - EnergyTopJ));
                    if (rawUEnergyAbs < weakThresholdNm)
                    {
                        weakPushSw ??= Stopwatch.StartNew();
                    }
                    else
                    {
                        weakPushSw = null; // 強い状態に戻ったら即リセット
                    }
                    double gainRamp = 1.0;
                    if (weakPushSw is { } wsw)
                    {
                        double weakSeconds = wsw.Elapsed.TotalSeconds - WeakPushGraceSeconds;
                        if (weakSeconds > 0)
                        {
                            gainRamp = Math.Min(1.0 + WeakPushRampPerSecond * weakSeconds, WeakPushRampMax);
                        }
                    }

                    double uEnergy = swingGain * gainRamp * (energy - EnergyTopJ) * sign;

                    double armVel = arm.VelocityRadPerSec;
                    // ★アームを開始位置へ戻す項。閾値を超えた分にだけバネを掛け、通常時は弱い減衰のみ
                    // （「押しっぱなし」でのアーム暴走は防ぎつつ、エネルギー整形則のポンピングは邪魔しない）。
                    double thresholdRad = ArmHoldThresholdDeg * Math.PI / 180.0;
                    double excessRad = Math.Abs(thetaRad) - thresholdRad;
                    double uArmHold = -ArmHoldKdAlways * armVel;
                    if (excessRad > 0)
                    {
                        uArmHold -= ArmHoldKpOutside * excessRad * Math.Sign(thetaRad);
                    }

                    double tau = armVel >= 0 ? FurutaGains.FrictionPositive : FurutaGains.FrictionNegative;
                    double uFric = SwingFrictionCompScale * tau * Math.Tanh(armVel / FurutaGains.FrictionEpsilon);

                    double uSwing = Math.Clamp(uEnergy + uArmHold, -swingTorqueLimitNm, swingTorqueLimitNm);
                    double uTotal = Math.Clamp(uSwing + uFric, -MotorScaling.TorqueMax, MotorScaling.TorqueMax);
                    appliedTorque = uTotal;
                    displayTorque = uTotal;
                }
            }
            else
            {
                // --- キャッチフェーズ：BalanceControlと同じLQR ---
                if (Math.Abs(alphaWrapped) > PendulumAngleAbortDeg)
                {
                    abortReason = $"振子角が上向きから{alphaWrapped:+0.0;-0.0}°（上限±{PendulumAngleAbortDeg:0}°）";
                    appliedTorque = 0.0;
                    motor.TorqueCommand(0.0);
                    break;
                }

                // ★8/28追加: 保持がautoStopAfterHoldSeconds続いたら自動終了する。
                // 毎回Ctrl+Cする手間を省き、次の試行にすぐ移れるようにするため。
                if (autoStopAfterHoldSeconds > 0 && holdSw is { } hsw && hsw.Elapsed.TotalSeconds >= autoStopAfterHoldSeconds)
                {
                    autoStopped = true;
                    appliedTorque = 0.0;
                    motor.TorqueCommand(0.0);
                    break;
                }

                // ★振り上げ中に蓄積した「巻き戻さない」alphaRadをそのまま渡すと、LQRが
                // 「αが何百度もズレている」と誤認して即座に飽和トルクを出す（8/26に実機で発生）。
                // オブザーバはα=0（上向き）近傍の小さいズレしか想定していないため、
                // ここでは±180°に巻き戻した値を使う（BalanceControlと同じ扱いに揃える）。
                double alphaForCatchRad = alphaWrapped * Math.PI / 180.0;
                double uCmd = observer.Step(thetaRad, alphaForCatchRad, dt, appliedTorqueNm: appliedTorque);
                appliedTorque = uCmd;
                displayTorque = uCmd;
                Interlocked.Increment(ref observerUpdateCount);
            }

            lock (stateLock)
            {
                sharedArmDeg = thetaRad * 180.0 / Math.PI;
                sharedPendulumDeg = alphaWrapped;
                sharedLastTorque = displayTorque;
            }
            lock (extremaLock)
            {
                double armDegAbs = Math.Abs(thetaRad * 180.0 / Math.PI);
                double pendulumDegAbs = Math.Abs(alphaWrapped);
                if (armDegAbs > maxAbsArmDeg) maxAbsArmDeg = armDegAbs;
                if (pendulumDegAbs > maxAbsPendulumDeg) maxAbsPendulumDeg = pendulumDegAbs;
            }
        }
    }
    finally
    {
        try { motor.TorqueCommand(0.0); motor.ReadFeedback(TimeSpan.FromMilliseconds(5)); } catch { /* 終了処理中の例外は無視 */ }
        try { motor.Disable(); } catch { /* 終了処理中の例外は無視 */ }
    }
}, cancellation.Token);

var displaySw = Stopwatch.StartNew();
long lastCount = 0;
double lastDisplayT = 0;

// ★8/28修正: autoStoppedをここで見ていなかったため、5秒保持で自動終了しても
// このループが終わらず、Ctrl+Cするまで数値が出続けてしまっていた（メンバーAの報告）。
while (!cancellation.IsCancellationRequested && abortReason is null && !autoStopped)
{
    double armDeg, pendulumDeg, lastTorque, vNow, branchNow;
    string phase;
    lock (stateLock)
    {
        armDeg = sharedArmDeg;
        pendulumDeg = sharedPendulumDeg;
        phase = sharedPhase;
        lastTorque = sharedLastTorque;
        vNow = sharedV;
        branchNow = sharedEnergyBranch;
    }

    long count = Interlocked.Read(ref observerUpdateCount);
    double nowT = displaySw.Elapsed.TotalSeconds;
    double obsHz = (nowT - lastDisplayT) > 0 ? (count - lastCount) / (nowT - lastDisplayT) : 0;
    lastCount = count;
    lastDisplayT = nowT;

    double maxDtSnapshot;
    lock (dtLock) { maxDtSnapshot = maxDtMs; }

    Console.WriteLine(
        $" {phase,-6} {pendulumDeg,7:+0.0;-0.0}   {armDeg,7:+0.0;-0.0}  | " +
        $"{lastTorque,15:+0.00;-0.00}  | " +
        $"{obsHz,6:0}  {maxDtSnapshot,9:0.0}  | " +
        $"{vNow,7:0.0} {branchNow,7:0.0}  | " +
        $"crc{encoder.CrcErrorCount,4} drop{encoder.DropCount,5} frame{encoder.FrameCount,7}");

    Thread.Sleep(100);
}

// ★8/28変更: ここでcancellation.Cancel()を呼ぶと、常時動いているpumpTask
// （エンコーダ受信）まで止まってしまい、次の試行の「真下で静止待ち」が動かなくなる。
// controlTaskはabort/autoStopで自分のwhileループをすでにbreakしているはずなので、
// ここではcontrolTaskだけを軽く待つ（Ctrl+Cが押されていればcancellationは既に立っている）。
try { controlTask.Wait(500); } catch { /* 中断時の例外は無視 */ }

Console.WriteLine();
if (abortReason is not null)
{
    Console.WriteLine($">> ★安全ガードにより自動停止しました: {abortReason}");
}
else if (autoStopped)
{
    Console.WriteLine($">> {autoStopAfterHoldSeconds:0}秒保持できたので自動終了しました。次の試行にどうぞ。");
}
double heldSeconds = controlStartSw.Elapsed.TotalSeconds;
double maxArmSnapshot, maxPendulumSnapshot;
lock (extremaLock) { maxArmSnapshot = maxAbsArmDeg; maxPendulumSnapshot = maxAbsPendulumDeg; }
Console.WriteLine($">> 経過時間 {heldSeconds:0.0}秒 / 最大アーム角 {maxArmSnapshot:0.0}° / 最大振子角 {maxPendulumSnapshot:0.0}°");
Console.WriteLine(">> モータは安全な状態（トルク0→失能）です。終了しました。");

// ★8/27 追加: 何回も試したときに集計しやすいよう、1行サマリをCSVへ自動追記する。
// 「caught（キャッチ判定を一度でも通ったか）」だけで判定する。振り上げ中に
// タイムアウト・手動中断した場合は頭打ち型、キャッチ後に安全ガード等で
// 止まった場合はキャッチ失敗型、キャッチ後に安全ガード等なく終わった場合は成功。
string trialResult = (caught, abortReason) switch
{
    (false, _) => "頭打ち",
    (true, not null) => "キャッチ失敗",
    (true, null) => "成功",
};
// ★8/28追加: Ctrl+Cで制御が始まる前に終わった分は記録しない（成功率の集計が狂うため）。
if (!caught && heldSeconds < 1.0)
{
    Console.WriteLine(">> 制御が始まる前に終了したため、この回は記録しません。");
}
else
try
{
    const string logPath = "src/SwingUp/trial_log.csv";
    if (!File.Exists(logPath))
    {
        File.WriteAllText(logPath, "日時,結果,振り上げ秒,経過秒,最大アーム角deg,最大振子角deg,中断理由,SwingGain,振り上げトルク上限Nm,キャッチ角度deg,振子角ガードdeg,最小射影法c,キャッチ時V\n");
    }
    string timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
    string abortField = abortReason?.Replace(",", "，") ?? "";
    string swingUpField = swingUpSeconds is { } s ? $"{s:0.0}" : "";
    File.AppendAllText(logPath,
        $"{timestamp},{trialResult},{swingUpField},{heldSeconds:0.0},{maxArmSnapshot:0.0},{maxPendulumSnapshot:0.0},{abortField}"
      + $",{swingGain:0.0},{swingTorqueLimitNm:0.00},{catchAngleDeg:0}"
      + $",{PendulumAngleAbortDeg:0},{(useMinProjection ? minProjC.ToString("0.0") : "")},{(catchV is { } cv ? cv.ToString("0.0") : "")}\n");
    Console.WriteLine($">> 記録しました（{trialResult}）: {logPath}");
}
catch (Exception ex)
{
    Console.WriteLine($">> 警告: ログの記録に失敗しました（{ex.Message}）。実験自体は正常に終了しています。");
}


} // ★8/28追加: ここまでが1試行分。Ctrl+Cが押されるまで先頭（真下で静止待ち）に戻る。

// ここに来るのはCtrl+Cでcancellationが立ったとき。pumpTaskをここで初めて止める。
cancellation.Cancel();
try { pumpTask.Wait(500); } catch { /* 中断時の例外は無視 */ }
Console.WriteLine(">> プログラムを終了します。");
return 0;

// ★8/28追加: LQRのリヤプノフ関数 V(x) = xᵀPx。最小射影法の切り替え判定に使う。
// PはFurutaGains.RiccatiP（リカッチ方程式の解、model/design_and_simulate.pyが出力）。
// 状態は観測値をそのまま使う（オブザーバは上向き付近の線形化モデル前提なので、
// 振り上げ中の推定値は当てにならない。θ・αは実測、θ̇はCANフィードバック、
// α̇は差分＋LPFの推定値）。
static double LyapunovV(double thetaRad, double alphaRad, double thetaDot, double alphaDot)
{
    double[] x = [thetaRad, alphaRad, thetaDot, alphaDot];
    double[,] P = FurutaGains.RiccatiP;
    double v = 0.0;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++) v += x[i] * P[i, j] * x[j];
    }
    return v;
}

static double Wrap180(double deg)
{
    deg %= 360.0;
    if (deg > 180.0) deg -= 360.0;
    if (deg < -180.0) deg += 360.0;
    return deg;
}
