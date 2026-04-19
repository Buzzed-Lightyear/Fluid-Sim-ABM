# Fluid-Sim-ABM

A Unity 2D simulator that hybridizes **SPH-style fluid physics** with **boids-style agent-based modeling (ABM)**. Particles act as solid bodies (hard collisions) while also obeying soft local rules (separation, alignment, cohesion) over a sensing radius. The result sits between fluid simulation and flocking — a base for granular flow, crowds, herds, or panic dynamics.

## What's in here

| Scene | Driver script | Compute shader | Notes |
|---|---|---|---|
| `Test A (2D)`, `Test B (2D)` | [Simulation2D.cs](Assets/Scripts/Sim%202D/Simulation2D.cs) | [FluidSim2D.compute](Assets/Scripts/Sim%202D/Compute/FluidSim2D.compute) | Pure SPH fluid (SebLague's original, with minor edits). |
| `Test C (3D)` | [Simulation3D.cs](Assets/Scripts/Sim%203D/Simulation3D.cs) | [FluidSim3D.compute](Assets/Scripts/Sim%203D/Compute/FluidSim3D.compute) | 3D SPH. ABM extensions not ported. |
| **`Test D (2D - ABM)`** | [SwarmSimulation.cs](Assets/Scripts/Sim%202D/SwarmSimulation.cs) | [SwarmSim.compute](Assets/Scripts/Sim%202D/Compute/SwarmSim.compute) | The ABM scene. 5 GPU kernels: external forces → spatial hash → hard collision → behavior → integrate. |

## ABM features (Test D)

| Feature | Where | Notes |
|---|---|---|
| Per-agent **collision radius** | `CollisionRadii` buffer | Heterogeneous body sizes (small/large agents collide correctly). |
| Per-agent **state** | `States` buffer + `StateColors[3]` shader uniform | `Calm` / `Scared` / `Huddle`. Each state overrides `idealNeighborCount`, cohesion strength, max speed. State is visualized with display color tinting. |
| **Cohesion** (full 2D) | `cohesionStrength` field | Real center-of-mass steering — pulls agents toward the mean position of neighbors within `sensorRadius`. |
| **Obstacle** | `obstacleSize`, `obstacleCentre` | AABB collision with the same restitution as walls. Zero size = disabled. Visible as a green wire cube in the scene gizmos. |
| **Mouse interaction** | LMB / RMB | Attract / repel. RMB triggers Scared state on Calm agents within `panicRadius`. |
| **Hold `H`** | Keyboard | Forces all agents into Huddle state until released. |

## Running the simulation

Open the project in **Unity 2022.3+**. Open `Test D (2D - ABM)` to play with the agent simulation, or any of the other scenes for the pure-fluid sims.

Useful inspector knobs on `SwarmSimulation` (in order of impact):
- `idealNeighborCount` — low (0–2) gives gas, high (8+) gives liquid/pile.
- `pressureMultiplier` — separation force when over-crowded.
- `cohesionStrength` — clumping force toward neighbor centroid.
- `viscosity` — alignment / velocity-matching strength.
- `collisionStiffness` — hardness of the no-overlap penalty.
- `maxForce` — must stay well above `|gravity|` or particles tunnel through the floor.

A summary of the manually-tuned "good fluid" parameters is in the doc-comment at the top of [SwarmSimulation.cs:4-27](Assets/Scripts/Sim%202D/SwarmSimulation.cs:4).

## CPU reference + test harness

The `Tests/AbmReference/` project is a **standalone .NET 8 xUnit project** that mirrors the GPU math in plain C# (`ReferenceSim.cs`). It's the source of truth for the simulation rules — when changing kernels, change the CPU port first, write tests, then mirror to HLSL.

```bash
dotnet test Tests/AbmReference/AbmReference.csproj
```

Should report **24 tests passing**. Suites:

| Suite | Count | What it covers |
|---|---:|---|
| Baseline | 6 | Original kernel invariants (energy, ordering, etc.) |
| `TaskATests` | 4 | Hard-collision split + per-agent radii |
| `TaskBTests` | 4 | Full 2D cohesion |
| `TaskCTests` | 5 | Agent-state behavior |
| `FluidQualityTests` | 4 | Density uniformity, settlement, no-compression, cluster coherence |
| `TunerRunner` | 1 | Env-gated; skipped unless `RUN_TUNER=1` |

## Parameter auto-tuner

A headless hill-climb tuner ([FluidTuner.cs](Tests/AbmReference/FluidTuner.cs)) replaces eyeballing parameters. Run it via the env-gated test:

```bash
# bash
RUN_TUNER=1 dotnet test Tests/AbmReference/AbmReference.csproj --filter TunerRunner --logger "console;verbosity=detailed"

# PowerShell
$Env:RUN_TUNER = "1"; dotnet test Tests/AbmReference/AbmReference.csproj --filter TunerRunner --logger "console;verbosity=detailed"; Remove-Item Env:RUN_TUNER
```

Output is written to [Assets/StreamingAssets/tuned_params.json](Assets/StreamingAssets/tuned_params.json). To apply it in the Unity editor:

1. Select the SwarmSimulation GameObject in the Test D scene.
2. Click the ⋮ on the SwarmSimulation component → **Apply Tuned Params**.
3. The 6 tunable inspector fields are overwritten from the JSON.

The tuner uses a soft-penalty fitness combining density spread, settlement speed, cluster coherence, and minimum pairwise distance (compression). Weights and tunable params are constants at the top of `FluidTuner.cs`.

## Repo layout

```
Assets/
├── Scenes/                       # Test A/B (fluid), Test C (3D), Test D (ABM)
├── Scripts/
│   ├── Sim 2D/                   # SPH (Simulation2D.cs) and ABM (SwarmSimulation.cs) drivers
│   │   ├── Compute/              # HLSL kernels
│   │   └── Display/              # GPU instanced particle rendering + shader
│   ├── Sim 3D/                   # 3D SPH only
│   └── Compute Helpers/          # Buffer + dispatch utilities
└── StreamingAssets/
    └── tuned_params.json         # Auto-tuner output

Tests/AbmReference/               # .NET 8 xUnit, CPU oracle for the ABM scene
├── ReferenceSim.cs               # CPU port of SwarmSim.compute
├── ReferenceSimTests.cs          # Baseline + Task A/B/C + FluidQualityTests
├── FluidMetrics.cs               # Pure metric functions
├── FluidTuner.cs                 # Hill-climb tuner
└── FluidTunerTests.cs            # Env-gated runner

Tasks/                            # Per-task implementation specs (all complete)
ABM_ROADMAP.md                    # Architecture + parameter reference
AGENTS.md                         # Guidance for AI agents working on the repo
```

## Status

The four planned ABM tasks (hard-collision split, full cohesion, agent states, fitness suite + tuner) are complete and merged.  

## Credits

Built on top of [SebLague](https://github.com/SebLague)'s [Fluid-Sim](https://github.com/SebLague/Fluid-Sim). Development videos: [Simulation](https://youtu.be/rSKMYc1CQHE?si=KNw_i1sN2_CWEmzA), [Rendering](https://youtu.be/kOkfC5fLfgE?si=1hXtw9nIiHllA6gn). Built in Unity 2022.3.

References (SPH simulation + rendering):
- [Particle-based Fluid Simulation for Interactive Applications (Müller et al.)](https://matthias-research.github.io/pages/publications/sca03.pdf)
- [Particle-based Viscoelastic Fluid Simulation (Clavet et al.)](https://web.archive.org/web/20250106201614/http://www.ligum.umontreal.ca/Clavet-2005-PVFS/pvfs.pdf)
- [SPH Tutorial (Koschier et al.)](https://sph-tutorial.physics-simulation.org/pdf/SPH_Tutorial.pdf)
- [NVIDIA particles paper](https://web.archive.org/web/20140725014123/https://docs.nvidia.com/cuda/samples/5_Simulations/particles/doc/particles.pdf)
- [Direct3D Effects (NVIDIA, GDC 2010)](https://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf)
- [Spray, Foam, Bubbles (CGI 2012)](https://cg.informatik.uni-freiburg.de/publications/2012_CGI_sprayFoamBubbles.pdf)
