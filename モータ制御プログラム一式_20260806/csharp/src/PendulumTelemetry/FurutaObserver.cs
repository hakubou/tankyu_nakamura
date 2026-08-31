// 8/21 作成: FurutaGains（A, B, K, L）とメンバーBの model/design_and_simulate.py の
// 制御ループを、そのままC#で再現した「計算だけ」のプレビュー。
//
// ★★★ モータへは一切送信しない。表示のみ。★★★
// ★★★ 符号（時計回り正 vs 数学的な右手系）を実機と照合していない。 ★★★
//      furuta_model.py のθ・αの向きの定義と、8/19に実機で決めた
//      「時計回りが正」が一致しているかは未検証。実際にモータへ
//      流す前に、必ずメンバーBと符号を突き合わせること。
//
// 再現元: model/design_and_simulate.py の以下の部分（コメントの行番号は8/21時点）
//   xhat = xhat + dt_ctrl * (A @ xhat + B*u_cmd + L @ (y - C @ xhat))
//   u_fb   = -K @ xhat
//   u_fric = comp_scale * tau_c(sign) * tanh(dtheta_hat / eps_comp)
//   u_cmd  = u_fb + u_fric
//
// comp_scale=0.9 は FurutaGains.cs には出力されていない定数なので、ここに直接書く。
// 値がズレたら model/design_and_simulate.py 側と要突き合わせ。

using DamiaoCan;

namespace PendulumTelemetry;

public sealed class FurutaObserver
{
    // design_and_simulate.py の HW["comp_scale"]。FurutaGains.cs には出力されていない値。
    private const double CompScale = 0.9;

    // design_and_simulate.py の np.clip(u_cmd, -torque_max, torque_max) 相当。
    // MotorScaling.TorqueMax（実機のT_MAXレジスタと一致させてある値）をそのまま使う。
    // torque_lsb は「12bitで±TorqueMaxを表す」ことから算出（MotorScaling.FloatToUIntと同じ式）。
    private static readonly double TorqueLsb = 2.0 * MotorScaling.TorqueMax / 4095.0;

    // 状態 x = [θ, α, θ̇, α̇]（furuta_model.py の座標系のまま）
    private readonly double[] _xhat = new double[4];

    public IReadOnlyList<double> State => _xhat;
    public double ThetaHatDeg => _xhat[0] * 180.0 / Math.PI;
    public double AlphaHatDeg => _xhat[1] * 180.0 / Math.PI;
    public double ThetaDotHat => _xhat[2];
    public double AlphaDotHat => _xhat[3];

    public double LastUFeedback { get; private set; }
    public double LastUFriction { get; private set; }
    public double LastUTotal { get; private set; }

    // ★8/27追加: SwingUpのキャッチ切替直後、内部状態が[0,0,0,0]から
    // 始まるままだと「実際は動いているのに静止していると誤認」し、
    // 数サンプルかけて急激に補正しようとして飽和トルクが出る
    // （8/26〜8/27に実機でキャッチ直後のu_totalが毎回±2.00へ張り付く形で観測）。
    // 切替の瞬間に実測値（角度はそのまま、速度はモータのCANフィードバックと
    // 振り上げ側で推定したα̇）を種として与え、この不整合を避ける。
    public void Seed(double thetaRad, double alphaRad, double thetaDot, double alphaDot)
    {
        _xhat[0] = thetaRad;
        _xhat[1] = alphaRad;
        _xhat[2] = thetaDot;
        _xhat[3] = alphaDot;
    }

    /// <summary>
    /// 1ステップ更新する。thetaRad/alphaRad は実測値[rad]（furuta_model.pyの座標系）。
    /// dtSeconds は実測した経過時間。design_and_simulate.py は固定 1/300 を使っているが、
    /// ここでは呼び出し間隔が一定しないため実測値を使う（数値積分としてはこちらが正しい）。
    ///
    /// appliedTorqueNm は「実際にモータへ送ったトルク」。このプログラムは常に0を送るので、
    /// 呼び出し側は常に 0.0 を渡すこと。★計算上のu_cmd（戻り値）とは別物★。
    /// ここを「計算したu_cmd（送ったつもり）」にすると、送っていないのに送った前提で
    /// 予測してしまい、実測との食い違いが次のu_cmdをさらに増幅する正のフィードバックで
    /// 指数発散する（8/21に実機で確認した不具合）。
    ///
    /// gainScale は状態フィードバック(u_fb = -K·x̂)だけに掛ける倍率（既定1.0＝そのまま）。
    /// 8/26、初めて閉ループでモータへ送る際に「低いゲインから慎重に」試すために追加。
    ///
    /// frictionScale は摩擦補償(u_fric)だけに掛ける倍率（既定1.0＝そのまま）。
    /// 8/26、実機試験でアーム角θが持続的に流れる不具合が出た際の切り分け用に追加。
    /// ★frictionScale=0で試したところ、θ・αとも0.5秒程度でより速く発散した。
    /// 摩擦補償が原因ではなく、むしろ残っていた実際の摩擦がコントローラの弱さを
    /// 隠していたと分かったため、既定の1.0へ戻す。
    ///
    /// thetaGainBoost は状態フィードバックのうちθの項（K[0]）だけに掛ける追加倍率（既定1.0）。
    /// 8/26、u_fbが小さいまま(0.01〜0.09)θが数十度流れる現象が繰り返し観測されたため、
    /// α側のゲイン（K[1]など）はそのままに、θの復元力だけを個別に強めて試すために追加。
    /// gainScaleとは独立：gainScaleは全体に、thetaGainBoostはθ項にだけ掛かる。
    /// </summary>
    public double Step(double thetaRad, double alphaRad, double dtSeconds, double appliedTorqueNm, double gainScale = 1.0, double frictionScale = 1.0, double thetaGainBoost = 1.0)
    {
        double[,] A = FurutaGains.A;
        double[] B = FurutaGains.B;
        double[,] L = FurutaGains.ObserverGain;
        double[] K = FurutaGains.StateFeedback;

        // 観測オブザーバの誤差ダイナミクス(A-LC)の固有値は最大|λ|≈111（実測パラメータから算出）。
        // 前進オイラー法が数値的に安定なのはおおよそ dt < 13ms（実測で確認）。
        // CAN応答がまれに遅れて dt がこれを超えると、そこだけ誤差が数倍に増幅され、
        // 積み重なって発散する（8/21に実機で発生）。
        // 対策: 大きい dt は小さく分割して複数回積分する（測定値 y はその間一定とみなす＝零次ホールド）。
        const double MaxSubStepSeconds = 0.002;   // 13msの安定限界に対して十分な余裕を持たせる

        // innov = y - C·x̂    （C = [[1,0,0,0],[0,1,0,0]] なので単純に引くだけ）
        if (dtSeconds > 0 && dtSeconds < 0.5)   // 初回・異常値は積分しない
        {
            int steps = Math.Max(1, (int)Math.Ceiling(dtSeconds / MaxSubStepSeconds));
            double subDt = dtSeconds / steps;
            Span<double> xdot = stackalloc double[4];

            for (int s = 0; s < steps; s++)
            {
                double innov0 = thetaRad - _xhat[0];
                double innov1 = alphaRad - _xhat[1];

                for (int i = 0; i < 4; i++)
                {
                    double v = 0.0;
                    for (int j = 0; j < 4; j++) v += A[i, j] * _xhat[j];
                    v += B[i] * appliedTorqueNm;
                    v += L[i, 0] * innov0 + L[i, 1] * innov1;
                    xdot[i] = v;
                }
                for (int i = 0; i < 4; i++) _xhat[i] += xdot[i] * subDt;
            }
        }

        double uFbTheta = -K[0] * _xhat[0] * thetaGainBoost;
        double uFbRest = 0.0;
        for (int i = 1; i < 4; i++) uFbRest -= K[i] * _xhat[i];
        double uFb = (uFbTheta + uFbRest) * gainScale;

        double thetaDotHat = _xhat[2];
        double tau = thetaDotHat >= 0 ? FurutaGains.FrictionPositive : FurutaGains.FrictionNegative;
        double uFric = frictionScale * CompScale * tau * Math.Tanh(thetaDotHat / FurutaGains.FrictionEpsilon);

        double uCmd = uFb + uFric;

        // ★飽和処理（design_and_simulate.py の np.clip → np.round(/lsb)*lsb と同じ順序）。
        // 実測で±2.0N·mの7倍(約14N·m)が出ていたため、表示値と実際に送る値を一致させる。
        uCmd = Math.Clamp(uCmd, -MotorScaling.TorqueMax, MotorScaling.TorqueMax);
        uCmd = Math.Round(uCmd / TorqueLsb) * TorqueLsb;

        LastUFeedback = uFb;
        LastUFriction = uFric;
        LastUTotal = uCmd;

        return uCmd;
    }
}
