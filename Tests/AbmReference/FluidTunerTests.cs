using System;
using Xunit;
using Xunit.Abstractions;

namespace AbmReference;

// Thin xUnit wrapper so the tuner can be invoked via the existing dotnet test pipeline.
// Gated behind the RUN_TUNER env var so normal `dotnet test` runs are silent no-ops.
//
// Run (PowerShell):
//   $Env:RUN_TUNER = "1"; dotnet test Tests/AbmReference/ --filter TunerRunner; Remove-Item Env:RUN_TUNER
// Run (bash):
//   RUN_TUNER=1 dotnet test Tests/AbmReference/ --filter TunerRunner
public class TunerRunner
{
    readonly ITestOutputHelper _out;
    public TunerRunner(ITestOutputHelper output) => _out = output;

    [Fact]
    public void RunHillClimbAndWriteJson()
    {
        if (Environment.GetEnvironmentVariable("RUN_TUNER") != "1")
        {
            _out.WriteLine("Skipped (set RUN_TUNER=1 to run).");
            return;
        }

        var result = FluidTuner.Run(maxIterations: 50, log: _out.WriteLine);
        _out.WriteLine($"Converged after {result.Iterations} iterations, final score {result.Score:F4}");

        string path = FluidTuner.ResolveStreamingAssetsPath();
        FluidTuner.WriteJson(result.Best, path);
        _out.WriteLine($"Wrote tuned params to: {path}");
    }
}
