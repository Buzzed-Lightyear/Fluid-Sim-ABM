# AGENTS.md — Conventions for AI Agents Working on This Repo

This file is the orientation guide for any AI agent (Claude Code, Cursor, Copilot agents, etc.) sent to extend or fix this codebase. Read it before reading anything else.

It also contains a **Worklog** at the bottom — append a short entry there whenever you finish a meaningful unit of work, so the next agent inherits accurate context.

---

## Repo at a glance

| Path | Purpose |
|---|---|
| `Assets/Scripts/Sim 2D/SwarmSimulation.cs` | The ABM driver (Test D scene). Owns GPU buffers, dispatches kernels, handles input + state transitions. |
| `Assets/Scripts/Sim 2D/Compute/SwarmSim.compute` | The HLSL compute shader. 5 kernels: `ExternalForces` → `UpdateSpatialHash` → `HardCollision` → `UpdateBehavior` → `UpdatePositions`. |
| `Assets/Scripts/Sim 2D/Simulation2D.cs` + `FluidSim2D.compute` | The pure-SPH fluid scenes (Test A/B). Don't modify unless your task is explicitly fluid-side. |
| `Tests/AbmReference/ReferenceSim.cs` | **The CPU oracle.** Plain C# port of `SwarmSim.compute`. Source of truth for the math. |
| `Tests/AbmReference/ReferenceSimTests.cs` | xUnit tests covering invariants + per-task suites (`TaskATests`, `TaskBTests`, `TaskCTests`, `FluidQualityTests`). |
| `Tests/AbmReference/FluidMetrics.cs` | Pure metric functions used by both the test suite and the tuner. |
| `Tests/AbmReference/FluidTuner.cs` + `FluidTunerTests.cs` | Hill-climb parameter tuner, env-gated as a test. |
| `Tasks/TASK_*.md` | Per-feature implementation specs. All marked complete; treat as historical reference. |
| `ABM_ROADMAP.md` | Architecture, parameter table, kernel diagrams. Living doc — keep in sync with code. |
| `Assets/StreamingAssets/tuned_params.json` | Tuner output, consumed by the inspector ContextMenu. |

---

## The non-negotiable workflow

When you change ABM behavior, **always** in this order:

1. **Edit `ReferenceSim.cs` first.** It's plain .NET, debuggable in seconds, and is the oracle.
2. **Add tests in `ReferenceSimTests.cs`.** Watch them fail before they pass.
3. **Run `dotnet test Tests/AbmReference/AbmReference.csproj`.** All green is your stop condition. Today that means **24 tests pass**.
4. **Mirror the math line-for-line into `SwarmSim.compute`.** If the CPU does X, the GPU must do X.
5. **Update the C# driver `SwarmSimulation.cs`** — buffer creation, kernel binding, `UpdateSettings` uniform pushes, gizmo drawing.
6. **The user does the visual Unity verification.** You do not need to open Unity (and usually can't).

If a test fails, **fix the math, never weaken the assertion**. That's how regressions are caught.

---

## Hard rules

- **CPU and GPU must agree.** Whenever you edit `SwarmSim.compute`, also edit `ReferenceSim.cs` (and vice versa). The doc-comment at the top of `ReferenceSim.cs` literally says "Keep this file in sync when editing the HLSL kernel." Do not ship a one-sided change.
- **Don't touch `SimulationXD.cs` or `FluidSimXD.compute` for ABM tasks.** Those are SebLague's pure-fluid scenes; they're upstream territory.
- **Don't add `using UnityEngine` to `Tests/AbmReference/`.** That project must run as plain .NET 8 — that's the whole point of having a CPU oracle.
- **Don't create new docs unless asked.** Update existing ones (`README.md`, `ABM_ROADMAP.md`, the relevant `Tasks/*.md`, this file).
- **`MaxForce` must stay well above `|Gravity|`** in any test or default. Below ~5×, particles tunnel through the floor and tests behave erratically.
- **Commit only when the user asks.** Don't pre-emptively `git commit`.

---

## Conventions worth knowing

### Field naming
- Public serialized fields on `SwarmSimulation` are **lowerCamelCase** (Unity convention): `idealNeighborCount`, `cohesionStrength`, `pressureMultiplier`.
- Same fields on the C# `SimSettings` struct (in `ReferenceSim.cs`) are **PascalCase**: `IdealNeighborCount`, `CohesionStrength`, `PressureMultiplier`.
- The tuner's `FluidTuner.WriteJson` uses the **lowerCamelCase** form because `JsonUtility.FromJsonOverwrite` matches Unity's serialized name. **If you add a new tunable param, the JSON key must match the inspector field name exactly** or the ContextMenu won't apply it.

### Kernel structure
- All ABM kernels use `[numthreads(64,1,1)]` and the `if (id.x >= numParticles) return;` early exit pattern.
- Neighbor loops walk the 9 cells around the current one via `offsets2D[i]` and the `SpatialHash`/`SpatialOffsets` tables. Don't reinvent this.
- Every per-agent attribute lives in its own `RWStructuredBuffer<T>`: `Positions`, `Velocities`, `PredictedPositions`, `Densities`, `CollisionRadii`, `States`. Add a new buffer for any new per-agent attribute — don't pack them.

### State system
- 3-state enum (`Calm=0`, `Scared=1`, `Huddle=2`) is uploaded as `RWStructuredBuffer<uint> States`.
- Per-state parameter overrides are uploaded as a packed `float4 StateParams[3]` (x=ideal, y=cohMult, z=speedMult). When adding a new per-state parameter, prefer reusing the unused `w` slot before adding a new array.
- Transitions are CPU-driven today (`SwarmSimulation.HandleInput` reads the `States` buffer back, applies rules, writes it back). For 16K particles this is fine. If you need to scale further, add an `UpdateState` GPU kernel — but mirror it in `ReferenceSim.cs` with a CPU equivalent first.

### Display
- `Particle2D.shader` reads `Velocities`, `Densities`, **and `States`** to color each instance. Color = `lerp(velocityColor, StateColors[stateIdx].rgb, StateColors[stateIdx].a)`.
- `ParticleDisplayGPU.cs` has overloaded `Init(Simulation2D)` and `Init(SwarmSimulation)`. The fluid scene gets a dummy zero-filled states buffer so the shader binding always succeeds.

### Soft-penalty over binary reject
- `FluidTuner.Evaluate` learned the hard way that binary `-1e9` rejects on a constraint violation create a non-navigable fitness landscape. Use a graduated penalty (`WCompression * worstOverlap`) for any new constraint you add. Documented in `FluidTuner.cs:82-84`.

---

## How to run things

```bash
# All tests (24, ~1 sec)
dotnet test Tests/AbmReference/AbmReference.csproj

# Just one suite
dotnet test Tests/AbmReference/AbmReference.csproj --filter "FullyQualifiedName~TaskC"

# The tuner (writes Assets/StreamingAssets/tuned_params.json)
RUN_TUNER=1 dotnet test Tests/AbmReference/AbmReference.csproj --filter TunerRunner --logger "console;verbosity=detailed"

# PowerShell variant
$Env:RUN_TUNER = "1"; dotnet test Tests/AbmReference/AbmReference.csproj --filter TunerRunner --logger "console;verbosity=detailed"; Remove-Item Env:RUN_TUNER
```

Unity build is gated behind opening the editor (Unity 2022.3+). Agents don't run Unity; the user does.

---

## Adding a new ABM feature — recipe

1. Write `Tasks/TASK_X_<feature>.md` with:
   - Background (link to relevant `ABM_ROADMAP.md` sections)
   - Goal in 1-2 sentences
   - Files to edit (table)
   - Implementation detail per file (CPU first, then HLSL, then driver)
   - Tests to add (table with name + assertion)
   - Stop condition (`dotnet test` green)
2. Implement on the CPU side in `ReferenceSim.cs`. Add tests. Make them pass.
3. Mirror to HLSL. Update buffers, kernel IDs, uniform pushes in `SwarmSimulation.cs`.
4. Update `ABM_ROADMAP.md` (kernel pipeline diagram, param table, status).
5. Update `README.md` if there's a user-facing knob.
6. Append a **Worklog** entry below.

---

## Worklog

Append a one-paragraph entry below the most recent one when you complete meaningful work. Format:

```
### YYYY-MM-DD — <agent / model> — <commit hash if any>
**What:** one-line summary.
**Why:** the user need it served (or the bug it fixed).
**Where:** key files touched.
**Gotchas / next steps:** anything the next agent needs to know.
```

Keep entries dense — no decorative prose. The point is downstream context, not a changelog.

 