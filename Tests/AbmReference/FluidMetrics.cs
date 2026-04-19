using System.Numerics;

namespace AbmReference;

// Pure measurements over a ReferenceSim state. Shared by FluidQualityTests and FluidTuner.
public static class FluidMetrics
{
    // Stddev of neighbor counts (Densities[i].X). Low = uniform packing.
    public static float DensityStdDev(ReferenceSim sim)
    {
        int n = sim.NumParticles;
        if (n == 0) return 0;
        double mean = 0;
        for (int i = 0; i < n; i++) mean += sim.Densities[i].X;
        mean /= n;
        double sumSq = 0;
        for (int i = 0; i < n; i++)
        {
            double d = sim.Densities[i].X - mean;
            sumSq += d * d;
        }
        return (float)System.Math.Sqrt(sumSq / n);
    }

    // Mean velocity magnitude. Low = settled.
    public static float AvgSpeed(ReferenceSim sim)
    {
        int n = sim.NumParticles;
        if (n == 0) return 0;
        double sum = 0;
        for (int i = 0; i < n; i++) sum += sim.Velocities[i].Length();
        return (float)(sum / n);
    }

    // Closest pair in the whole system. Below 2*CollisionRadius means overlap.
    public static float MinPairwiseDistance(ReferenceSim sim)
    {
        int n = sim.NumParticles;
        if (n < 2) return float.MaxValue;
        float min = float.MaxValue;
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
            {
                float d = Vector2.Distance(sim.Positions[i], sim.Positions[j]);
                if (d < min) min = d;
            }
        return min;
    }

    // Furthest particle from the cluster centroid. Bounded = coherent flock.
    public static float MaxCentroidDistance(ReferenceSim sim)
    {
        int n = sim.NumParticles;
        if (n == 0) return 0;
        Vector2 centroid = Vector2.Zero;
        for (int i = 0; i < n; i++) centroid += sim.Positions[i];
        centroid /= n;
        float max = 0;
        for (int i = 0; i < n; i++)
        {
            float d = Vector2.Distance(sim.Positions[i], centroid);
            if (d > max) max = d;
        }
        return max;
    }
}
