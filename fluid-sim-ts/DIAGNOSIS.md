# Black Screen Diagnosis

## Root Causes (static analysis — no browser run needed)

### Bug 1 — `Loop(() => {...})` generates invalid WGSL (PRIMARY CAUSE)

**Files:** `src/sim/kernels/hardCollision.ts`, `src/sim/kernels/updateBehavior.ts`

`Loop(callback)` with no range argument produces a `LoopNode` with zero `for`-loop headers
but still emits the `break` / `continue` statements from inside the body.
WGSL forbids `break` outside a loop — the WebGPU shader compiler rejects the shader.

`renderer.computeAsync(hardCollisionNode)` throws on frame 1.
Because the frame callback is `async` and `renderer.render()` lives *after* the `await`,
the render call is skipped on every rejected frame.
`requestAnimationFrame(frame)` is also skipped — the loop dies after frame 1.
The canvas never presents any content: **black screen**.

**Step in diagnosis order that would have surfaced this:** Step 2 (frame counter)
would show "frame 1" in the console but nothing after; Step 3 (magenta clear)
would show a black canvas even with the clear color set, because `renderer.render()`
never ran.

**Fix:** replace the bare-callback `Loop(() => {...})` with a ranged loop:
```ts
// Before (broken — no for-loop header, break outside loop = WGSL error)
;(Loop as any)(() => {
  If(currIdx.greaterThanEqual(numParticles), () => Break())
  // ...
  currIdx.addAssign(uint(1))
})

// After (correct — auto-incremented for-loop from spatialOffset to numParticles)
Loop(
  { type: 'uint', start: spatialOffsets.element(key), end: numParticles },
  ({ i: idx }) => {
    // ... use idx for array access; loop auto-increments
  }
)
```

---

### Bug 2 — Async frame loop not protected

**File:** `src/main.ts`

```ts
async function frame() {
  await sim.update(dt)   // if this throws...
  renderer.render(...)   // ...never runs
  requestAnimationFrame(frame)  // ...loop dies
}
```

Even with Bug 1 fixed, any future GPU error would permanently kill the render loop.

**Fix:** wrap `sim.update` in try/catch; always call `renderer.render()` and
`requestAnimationFrame(frame)` regardless of whether compute succeeded.

---

### Bug 3 — Compute nodes recreated every frame

**Files:** `src/sim/SwarmSimulation.ts`, `src/sim/bitonicSort.ts`

`(this.kExternalForces as any)().compute(N)` was called inside `runStep()`,
creating a new `ComputeNode` object on every frame.
Three.js caches compiled pipelines by object identity: a new `ComputeNode` instance
every frame means the WGSL shader is recompiled every frame (~100 ms per kernel).
For N=4096, this causes 6+ recompilations per frame → extreme stutter.

**Fix:** call `.compute(N)` once in the constructor, store the `ComputeNode`,
dispatch the same node every frame.

---

### Bug 4 — uint × float type mismatches in WGSL

**Files:** `src/sim/spatialHash.ts`, `src/sim/bitonicSort.ts`

```ts
// spatialHash.ts: HASH_K1 = 15823 is a JS number → TSL converts to float(15823)
ucx.mul(HASH_K1)          // uint × float → WGSL type error
// Fix:
ucx.mul(uint(HASH_K1))    // uint × uint → correct

// bitonicSort.ts: groupWidthU is uniform<uint>
groupWidthU.sub(1)        // uint − float(1) → WGSL type error
// Fix:
groupWidthU.sub(uint(1))  // uint − uint → correct
```

---

## Diagnosis Step at Which Visible Output First Appears

With all four fixes applied:

| Step | Expected output |
|---|---|
| Step 1 (init log) | Console: `[DIAG-1] renderer.init() OK  backend: WebGPUBackend  isWebGPU: true` |
| Step 2 (frame counter) | Console: `[DIAG-2] frame 1`, then `frame 60`, then `frame 300` |
| Step 3 (magenta clear) | **Magenta canvas** on frame 1, then particles appear as shaders compile |
| Step 5 (static buffer) | Not needed — draw path was correct; the bug was in compute |

## What Was Not Broken

- `renderer.init()` awaited before first frame ✓
- `setDrawRange(0, N)` with no position attribute — WebGPU uses `drawRange.count` directly ✓
- `positionNode = Fn(() => vec3(positions.element(vertexIndex), 0))()` — correct pattern ✓
- `OrthographicCamera` frustum covers the spawn region ✓
- `body { margin:0; overflow:hidden }` — canvas has explicit pixel dimensions from `setSize` ✓
