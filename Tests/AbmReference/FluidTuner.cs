using System.IO;
using System.Numerics;
using System.Text;

namespace AbmReference;

// Headless hill-climb parameter tuner. Not a unit test — gated behind env var in the
// TunerRunner test class below so normal `dotnet test` runs are silent.
//
// To run (PowerShell):
//   $Env:RUN_TUNER = "1"; dotnet test Tests/AbmReference/ --filter TunerRunner; Remove-Item Env:RUN_TUNER
//
// The tuner writes the best-found params to Assets/StreamingAssets/tuned_params.json,
// which a [ContextMenu] on SwarmSimulation can then apply to the inspector.
public static class FluidTuner
{
    // Names map to SimSettings fields; kept here for transparency and manual tweaking.
    static readonly string[] TunableParams =
    {
        nameof(SimSettings.PressureMultiplier),
        nameof(SimSettings.IdealNeighborCount),
        nameof(SimSettings.Viscosity),
        nameof(SimSettings.CohesionStrength),
        nameof(SimSettings.Drag),
        nameof(SimSettings.CollisionStiffness),
    };

    // Nudge factors — each iteration tries decreasing AND increasing each param by these factors.
    static readonly float[] NudgeFactors = { 0.85f, 1.15f };

    // Fitness weights: higher = more punishing.
    const float WDensity = 2.0f;
    const float WSettle = 1.0f;
    const float WCluster = 0.5f;

    public readonly struct TuneResult
    {
        public readonly SimSettings Best;
        public readonly float Score;
        public readonly int Iterations;
        public TuneResult(SimSettings best, float score, int iters)
        { Best = best; Score = score; Iterations = iters; }
    }

    public static TuneResult Run(int maxIterations = 50, System.Action<string>? log = null)
    {
        log ??= _ => { };
        var current = SimSettings.Default();
        float currentScore = Evaluate(current);
        log($"[start] score={currentScore:F3}  {DescribeSettings(current)}");

        int iter;
        for (iter = 0; iter < maxIterations; iter++)
        {
            bool improved = false;
            foreach (var name in TunableParams)
            {
                foreach (var factor in NudgeFactors)
                {
                    var candidate = Nudge(current, name, factor);
                    float score = Evaluate(candidate);
                    if (score > currentScore + 1e-4f)
                    {
                        log($"[iter {iter}] {name} *= {factor} -> {score:F3}");
                        current = candidate;
                        currentScore = score;
                        improved = true;
                        break;
                    }
                }
            }
            if (!improved)
            {
                log($"[iter {iter}] no improvement, converged");
                break;
            }
        }
        log($"[done] score={currentScore:F3}  {DescribeSettings(current)}");
        return new TuneResult(current, currentScore, iter);
    }

    // Compression penalty weight. Soft penalty (not binary reject) so the tuner can climb
    // out of bad-stiffness regions instead of being trapped at -1e9.
    const float WCompression = 50.0f;

    static float Evaluate(SimSettings s)
    {
        // Sanity: out-of-range params → reject.
        if (s.PressureMultiplier < 0 || s.IdealNeighborCount < 0 || s.Viscosity < 0 ||
            s.CohesionStrength < 0 || s.Drag <= 0 || s.CollisionStiffness < 0)
            return -1e9f;

        // Canonical setup: 49 particles, settled under gravity.
        int side = 7;
        int n = side * side;
        float spacing = 0.3f;
        var pos = new Vector2[n];
        var vel = new Vector2[n];
        for (int i = 0; i < n; i++)
        {
            int x = i % side, y = i / side;
            pos[i] = new Vector2((x - side / 2f) * spacing, (y - side / 2f) * spacing);
        }
        var sim = new ReferenceSim(s, pos, vel);

        float floor = 2f * s.CollisionRadius - 1e-3f;
        float worstOverlap = 0f; // largest (floor - minDist) seen across the run
        for (int step = 0; step < 600; step++)
        {
            sim.Step(1f / 180f);
            float minD = FluidMetrics.MinPairwiseDistance(sim);
            if (minD < floor)
            {
                float overlap = floor - minD;
                if (overlap > worstOverlap) worstOverlap = overlap;
            }
        }

        float density = FluidMetrics.DensityStdDev(sim);
        float settle = FluidMetrics.AvgSpeed(sim);
        float cluster = FluidMetrics.MaxCentroidDistance(sim);

        if (float.IsNaN(density) || float.IsNaN(settle) || float.IsNaN(cluster)) return -1e9f;

        // Higher = better. All four terms are subtracted; tuner maximizes.
        return -(WDensity * density + WSettle * settle + WCluster * cluster + WCompression * worstOverlap);
    }

    static SimSettings Nudge(SimSettings s, string param, float factor)
    {
        switch (param)
        {
            case nameof(SimSettings.PressureMultiplier):  s.PressureMultiplier *= factor; break;
            case nameof(SimSettings.IdealNeighborCount):  s.IdealNeighborCount *= factor; break;
            case nameof(SimSettings.Viscosity):           s.Viscosity *= factor; break;
            case nameof(SimSettings.CohesionStrength):    s.CohesionStrength *= factor; break;
            case nameof(SimSettings.Drag):                s.Drag *= factor; break;
            case nameof(SimSettings.CollisionStiffness):  s.CollisionStiffness *= factor; break;
            default: throw new System.ArgumentException($"Unknown tunable param: {param}");
        }
        return s;
    }

    static string DescribeSettings(SimSettings s)
        => $"PM={s.PressureMultiplier:F2} IdealN={s.IdealNeighborCount:F2} Visc={s.Viscosity:F2} " +
           $"Coh={s.CohesionStrength:F2} Drag={s.Drag:F3} Stiff={s.CollisionStiffness:F2}";

    // Serializes the tuned params into a format consumable by Unity's JsonUtility.
    // The field names match the lowerCamelCase ones in SwarmSimulation.cs.
    public static void WriteJson(SimSettings s, string path)
    {
        var sb = new StringBuilder();
        sb.Append('{').Append('\n');
        sb.Append($"  \"pressureMultiplier\": {s.PressureMultiplier},\n");
        sb.Append($"  \"idealNeighborCount\": {s.IdealNeighborCount},\n");
        sb.Append($"  \"viscosity\": {s.Viscosity},\n");
        sb.Append($"  \"cohesionStrength\": {s.CohesionStrength},\n");
        sb.Append($"  \"drag\": {s.Drag},\n");
        sb.Append($"  \"collisionStiffness\": {s.CollisionStiffness}\n");
        sb.Append('}').Append('\n');
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        File.WriteAllText(path, sb.ToString());
    }

    // Walks up from the test binary output to find the repo root (contains Assets/).
    public static string ResolveStreamingAssetsPath()
    {
        string? dir = Directory.GetCurrentDirectory();
        while (dir != null && !Directory.Exists(Path.Combine(dir, "Assets")))
            dir = Directory.GetParent(dir)?.FullName;
        if (dir == null)
            throw new DirectoryNotFoundException("Couldn't locate repo root (no 'Assets/' found above CWD).");
        return Path.Combine(dir, "Assets", "StreamingAssets", "tuned_params.json");
    }
}
