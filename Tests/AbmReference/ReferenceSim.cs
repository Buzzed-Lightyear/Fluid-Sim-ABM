using System.Numerics;

namespace AbmReference;

public enum AgentState : uint { Calm = 0, Scared = 1, Huddle = 2 }

// CPU port of Assets/Scripts/Sim 2D/Compute/SwarmSim.compute.
// Faithful to the GPU math; brute-force neighbors (no spatial hash - tests use small N).
// Keep this file in sync when editing the HLSL kernel.
public class ReferenceSim
{
    public SimSettings Settings;
    public Vector2[] Positions;
    public Vector2[] Velocities;
    public Vector2[] PredictedPositions;
    public Vector2[] Densities;
    public float[] CollisionRadii;
    public uint[] States;
    public Vector2 InteractionPoint;
    public float InteractionStrength;

    public int NumParticles => Positions.Length;

    public ReferenceSim(SimSettings settings, Vector2[] initialPositions, Vector2[] initialVelocities)
    {
        Settings = settings;
        Positions = (Vector2[])initialPositions.Clone();
        Velocities = (Vector2[])initialVelocities.Clone();
        PredictedPositions = (Vector2[])initialPositions.Clone();
        Densities = new Vector2[initialPositions.Length];
        CollisionRadii = new float[initialPositions.Length];
        States = new uint[initialPositions.Length];
        for (int i = 0; i < CollisionRadii.Length; i++)
            CollisionRadii[i] = settings.CollisionRadius;
    }

    (float ideal, float cohMult, float speedMult) GetStateParams(uint s) => s switch
    {
        (uint)AgentState.Scared => (0f, 0f, 1.5f),
        (uint)AgentState.Huddle => (20f, 3f, 1.0f),
        _ => (Settings.IdealNeighborCount, 1f, 1f),
    };

    public void Step(float dt)
    {
        ExternalForces(dt);
        HardCollision(dt);
        UpdateBehavior(dt);
        UpdatePositions(dt);
    }

    // Pre-Task-A execution path. Kept as the oracle for the null-hypothesis test.
    public void LegacyStep(float dt)
    {
        ExternalForces(dt);
        LegacyUpdateBoids(dt);
        UpdatePositions(dt);
    }

    // Kernel 1
    void ExternalForces(float dt)
    {
        for (int i = 0; i < NumParticles; i++)
        {
            Vector2 force = new(0, Settings.Gravity);

            if (InteractionStrength != 0)
            {
                Vector2 offset = InteractionPoint - Positions[i];
                float sqrDst = Vector2.Dot(offset, offset);
                float r = Settings.InteractionRadius;
                if (sqrDst < r * r)
                {
                    float dst = MathF.Sqrt(sqrDst);
                    Vector2 dir = offset / dst;
                    float t = 1f - dst / r;
                    force += dir * (InteractionStrength * t);
                }
            }

            Velocities[i] += force * dt;
            PredictedPositions[i] = Positions[i] + Velocities[i] * (1f / 120f);
        }
    }

    // Kernel 2: Hard collision (granular physics). Per-agent radii.
    // No force clamp - collision physics should be immediate.
    void HardCollision(float dt)
    {
        for (int i = 0; i < NumParticles; i++)
        {
            Vector2 pos = PredictedPositions[i];
            Vector2 colForce = Vector2.Zero;

            for (int j = 0; j < NumParticles; j++)
            {
                if (j == i) continue;
                Vector2 neighborPos = PredictedPositions[j];
                Vector2 offset = neighborPos - pos;
                float sqrDst = Vector2.Dot(offset, offset);

                float minDst = CollisionRadii[i] + CollisionRadii[j];
                if (sqrDst >= minDst * minDst) continue;

                float dst = MathF.Sqrt(sqrDst);
                if (dst <= 0.0001f) continue;

                float kick = (minDst - dst) / minDst;
                colForce -= offset / dst * (kick * kick * Settings.CollisionStiffness);
            }

            Velocities[i] += colForce * dt;
        }
    }

    // Kernel 3: Behavioral steering. Separation / alignment / cohesion.
    void UpdateBehavior(float dt)
    {
        float searchRad = Settings.SensorRadius;

        for (int i = 0; i < NumParticles; i++)
        {
            Vector2 pos = PredictedPositions[i];
            Vector2 myVel = Velocities[i];

            Vector2 pressureForce = Vector2.Zero;
            Vector2 viscosityForce = Vector2.Zero;
            Vector2 avgPos = Vector2.Zero;
            int neighborCount = 0;

            for (int j = 0; j < NumParticles; j++)
            {
                if (j == i) continue;
                Vector2 neighborPos = PredictedPositions[j];
                Vector2 offset = neighborPos - pos;
                float sqrDst = Vector2.Dot(offset, offset);

                if (sqrDst >= searchRad * searchRad) continue;

                float dst = MathF.Sqrt(sqrDst);
                if (dst <= 0.0001f) continue;

                if (dst < Settings.SensorRadius)
                {
                    neighborCount++;
                    avgPos += neighborPos;
                    pressureForce -= offset / dst * (1f / dst);
                    viscosityForce += (Velocities[j] - myVel) / dst;
                }
            }

            var (ideal, cohMult, speedMult) = GetStateParams(States[i]);

            Vector2 totalForce = Vector2.Zero;

            if (neighborCount > 0)
            {
                float densityError = neighborCount - ideal;
                if (densityError > 0)
                    totalForce += pressureForce * Settings.PressureMultiplier * densityError;

                totalForce += viscosityForce * Settings.Viscosity;

                avgPos /= neighborCount;
                totalForce += (avgPos - pos) * Settings.CohesionStrength * cohMult;
            }

            totalForce -= myVel * Settings.Drag;

            float fLen = totalForce.Length();
            if (fLen > Settings.MaxForce)
                totalForce = totalForce / fLen * Settings.MaxForce;

            Velocities[i] += totalForce * dt;

            float speedCap = Settings.MaxSpeed * speedMult;
            float vLen = Velocities[i].Length();
            if (vLen > speedCap)
                Velocities[i] = Velocities[i] / vLen * speedCap;

            Densities[i] = new Vector2(neighborCount, 0);
        }
    }

    // Pre-Task-A fused kernel. Kept verbatim as an oracle.
    void LegacyUpdateBoids(float dt)
    {
        float searchRad = MathF.Max(Settings.SensorRadius, Settings.CollisionRadius * 2.01f);

        for (int i = 0; i < NumParticles; i++)
        {
            Vector2 pos = PredictedPositions[i];
            Vector2 myVel = Velocities[i];

            Vector2 pressureForce = Vector2.Zero;
            Vector2 viscosityForce = Vector2.Zero;
            Vector2 colForce = Vector2.Zero;
            int neighborCount = 0;

            for (int j = 0; j < NumParticles; j++)
            {
                if (j == i) continue;
                Vector2 neighborPos = PredictedPositions[j];
                Vector2 offset = neighborPos - pos;
                float sqrDst = Vector2.Dot(offset, offset);

                if (sqrDst >= searchRad * searchRad) continue;

                float dst = MathF.Sqrt(sqrDst);
                if (dst <= 0.0001f) continue;

                float minDst = Settings.CollisionRadius * 2f;
                if (dst < minDst)
                {
                    float kick = (minDst - dst) / minDst;
                    colForce -= offset / dst * (kick * kick * Settings.CollisionStiffness);
                }

                if (dst < Settings.SensorRadius)
                {
                    neighborCount++;
                    pressureForce -= offset / dst * (1f / dst);
                    viscosityForce += (Velocities[j] - myVel) / dst;
                }
            }

            Vector2 totalForce = colForce;

            if (neighborCount > 0)
            {
                float densityError = neighborCount - Settings.IdealNeighborCount;
                if (densityError > 0)
                    totalForce += pressureForce * Settings.PressureMultiplier * densityError;

                totalForce += viscosityForce * Settings.Viscosity;
            }

            totalForce -= myVel * Settings.Drag;

            float fLen = totalForce.Length();
            if (fLen > Settings.MaxForce)
                totalForce = totalForce / fLen * Settings.MaxForce;

            Velocities[i] += totalForce * dt;

            float vLen = Velocities[i].Length();
            if (vLen > Settings.MaxSpeed)
                Velocities[i] = Velocities[i] / vLen * Settings.MaxSpeed;

            Densities[i] = new Vector2(neighborCount, 0);
        }
    }

    // Kernel 4
    void UpdatePositions(float dt)
    {
        Vector2 halfSize = Settings.BoundsSize * 0.5f;

        for (int i = 0; i < NumParticles; i++)
        {
            Positions[i] += Velocities[i] * dt;
            Vector2 pos = Positions[i];
            Vector2 vel = Velocities[i];

            if (MathF.Abs(pos.X) > halfSize.X)
            {
                pos.X = halfSize.X * MathF.Sign(pos.X);
                vel.X *= -Settings.WallBounciness;
            }
            if (MathF.Abs(pos.Y) > halfSize.Y)
            {
                pos.Y = halfSize.Y * MathF.Sign(pos.Y);
                vel.Y *= -Settings.WallBounciness;
            }

            // Rectangular obstacle. Zero ObstacleSize is a no-op.
            Vector2 obstacleHalfSize = Settings.ObstacleSize * 0.5f;
            Vector2 obstacleEdgeDst = obstacleHalfSize - new Vector2(
                MathF.Abs(pos.X - Settings.ObstacleCentre.X),
                MathF.Abs(pos.Y - Settings.ObstacleCentre.Y));
            if (obstacleEdgeDst.X >= 0 && obstacleEdgeDst.Y >= 0)
            {
                if (obstacleEdgeDst.X < obstacleEdgeDst.Y)
                {
                    pos.X = obstacleHalfSize.X * MathF.Sign(pos.X - Settings.ObstacleCentre.X) + Settings.ObstacleCentre.X;
                    vel.X *= -Settings.WallBounciness;
                }
                else
                {
                    pos.Y = obstacleHalfSize.Y * MathF.Sign(pos.Y - Settings.ObstacleCentre.Y) + Settings.ObstacleCentre.Y;
                    vel.Y *= -Settings.WallBounciness;
                }
            }

            Positions[i] = pos;
            Velocities[i] = vel;
        }
    }
}

public struct SimSettings
{
    public float Gravity;
    public float WallBounciness;
    public Vector2 BoundsSize;

    public float SensorRadius;
    public float IdealNeighborCount;
    public float PressureMultiplier;
    public float Viscosity;
    public float Drag;

    public float CollisionRadius;
    public float CollisionStiffness;
    public float CohesionStrength;

    public float MaxSpeed;
    public float MaxForce;

    public float InteractionRadius;

    public Vector2 ObstacleSize;
    public Vector2 ObstacleCentre;

    public static SimSettings Default() => new()
    {
        Gravity = -9.81f,
        WallBounciness = 0.24f,
        BoundsSize = new Vector2(17, 9),
        SensorRadius = 1f,
        IdealNeighborCount = 8,
        PressureMultiplier = 8,
        Viscosity = 1,
        Drag = 0.1f,
        CollisionRadius = 0.05f,
        CollisionStiffness = 20,
        CohesionStrength = 0.5f,
        MaxSpeed = 5,
        MaxForce = 50,
        InteractionRadius = 3,
        ObstacleSize = Vector2.Zero,
        ObstacleCentre = Vector2.Zero,
    };
}
