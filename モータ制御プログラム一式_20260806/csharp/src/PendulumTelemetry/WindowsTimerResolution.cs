// 8/21 作成: Windows のタイマ分解能を一時的に上げる。
//
// ■ なぜ必要か（CLAUDE.md の指摘そのもの）
//
//   「Thread.Sleep() を周期生成に使わない。Windows の既定タイマ分解能は約 15.6 ms で、
//     Thread.Sleep(5) が 15 ms 眠りうる。Stopwatch + スピン待ち、または
//     timeBeginPeriod の P/Invoke で分解能を上げる」
//
//   SeeedCanBus.ReadByte() は SpinWait でポーリングしていて一見安全に見えるが、
//   SpinWait は回数を重ねると内部で Thread.Sleep(1) を呼ぶ。既定分解能のままだと
//   この 1ms が最大 15.6ms になる。
//
//   8/21 の実測で、オブザーバの誤差ダイナミクス A-LC の固有値から
//   「dt < 13ms でないと前進オイラー法が数値的に不安定」と分かっている。
//   15.6ms はこの閾値をまたぐため、まれな長い待ちが発散の引き金になりうる。
//
// ■ 注意
//
//   Windows 10 2004 以降、この設定はプロセス単位で効く（昔はシステム全体だった）。
//   必ず Dispose で元に戻すこと。

using System.Runtime.InteropServices;
using System.Runtime.Versioning;

namespace PendulumTelemetry;

public readonly struct WindowsTimerResolution : IDisposable
{
    private readonly uint _period;
    private readonly bool _applied;

    private WindowsTimerResolution(uint period, bool applied)
    {
        _period = period;
        _applied = applied;
    }

    [SupportedOSPlatform("windows")]
    [DllImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
    private static extern uint TimeBeginPeriod(uint uMilliseconds);

    [SupportedOSPlatform("windows")]
    [DllImport("winmm.dll", EntryPoint = "timeEndPeriod")]
    private static extern uint TimeEndPeriod(uint uMilliseconds);

    private const uint TimerrNoerror = 0;

    /// <summary>
    /// タイマ分解能を periodMs に上げる。Windows以外や失敗時は何もしない
    /// （精度が落ちるだけで動作はするので、例外にはしない）。
    /// </summary>
    public static WindowsTimerResolution Begin(uint periodMs = 1)
    {
        if (!OperatingSystem.IsWindows()) return new WindowsTimerResolution(periodMs, false);

        try
        {
            bool ok = TimeBeginPeriod(periodMs) == TimerrNoerror;
            if (ok)
                Console.WriteLine($">> タイマ分解能を {periodMs} ms に設定しました（既定は約15.6ms）。");
            else
                Console.WriteLine($">> タイマ分解能の変更に失敗しました。既定（約15.6ms）のまま続行します。");
            return new WindowsTimerResolution(periodMs, ok);
        }
        catch (DllNotFoundException)
        {
            return new WindowsTimerResolution(periodMs, false);
        }
        catch (EntryPointNotFoundException)
        {
            return new WindowsTimerResolution(periodMs, false);
        }
    }

    public void Dispose()
    {
        if (!_applied) return;
        if (!OperatingSystem.IsWindows()) return;
        try { TimeEndPeriod(_period); } catch { /* 終了処理中の例外は無視 */ }
    }
}
