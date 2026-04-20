// @ts-nocheck
import type { WebGPURenderer } from 'three/webgpu'
import { BufferManager } from './BufferManager.js'
import { createSimUniforms, updateSimUniforms, type SimUniforms } from './SimUniforms.js'
import { BitonicSort } from './bitonicSort.js'
import { buildExternalForcesKernel } from './kernels/externalForces.js'
import { buildUpdateSpatialHashKernel } from './kernels/updateSpatialHash.js'
import { buildHardCollisionKernel } from './kernels/hardCollision.js'
import { buildUpdateBehaviorKernel } from './kernels/updateBehavior.js'
import { buildUpdatePositionsKernel } from './kernels/updatePositions.js'
import { spawnParticles } from './ParticleSpawner.js'
import { DEFAULT_SETTINGS, type SimSettings } from '../types/SimSettings.js'
import { AgentState } from '../ref/ReferenceSim.js'

const CALM_DURATION_FRAMES = 180  // ~3s at 60fps before Scared → Calm

export class SwarmSimulation {
  readonly buf: BufferManager
  readonly uniforms: SimUniforms

  settings: SimSettings
  iterationsPerFrame = 3
  timeScale = 1.0
  paused = false

  private readonly renderer: WebGPURenderer
  private readonly N: number
  private readonly sort: BitonicSort

  // TSL kernel Fn nodes (built once — ComputeNodes created below)
  private readonly kExternalForces: ReturnType<typeof buildExternalForcesKernel>
  private readonly kSpatialHash: ReturnType<typeof buildUpdateSpatialHashKernel>
  private readonly kHardCollision: ReturnType<typeof buildHardCollisionKernel>
  private readonly kBehavior: ReturnType<typeof buildUpdateBehaviorKernel>
  private readonly kPositions: ReturnType<typeof buildUpdatePositionsKernel>

  // ComputeNodes created once — reused every frame (prevents per-frame shader recompile)
  private readonly cExternalForces: any
  private readonly cSpatialHash: any
  private readonly cHardCollision: any
  private readonly cBehavior: any
  private readonly cPositions: any

  // CPU interaction state
  private interactionPoint = { x: 0, y: 0 }
  private interactionStrength = 0

  // Per-agent scared timer (counts down frames until Calm)
  private scaredTimers: Int32Array

  constructor(renderer: WebGPURenderer, N = 4096, settings?: Partial<SimSettings>) {
    this.renderer = renderer
    this.N = N
    this.settings = { ...DEFAULT_SETTINGS, ...settings }

    this.buf = new BufferManager(N)
    this.uniforms = createSimUniforms()
    this.sort = new BitonicSort(this.buf, N)
    this.scaredTimers = new Int32Array(N)

    // Build kernel Fn nodes (TSL AST constructed here; actual WGSL compilation is lazy)
    this.kExternalForces = buildExternalForcesKernel(this.buf, this.uniforms)
    this.kSpatialHash    = buildUpdateSpatialHashKernel(this.buf, this.uniforms)
    this.kHardCollision  = buildHardCollisionKernel(this.buf, this.uniforms)
    this.kBehavior       = buildUpdateBehaviorKernel(this.buf, this.uniforms)
    this.kPositions      = buildUpdatePositionsKernel(this.buf, this.uniforms)

    // Create ComputeNodes ONCE — prevents per-frame WGSL recompilation
    ;(this as any).cExternalForces = (this.kExternalForces as any).compute(N)
    ;(this as any).cSpatialHash    = (this.kSpatialHash    as any).compute(N)
    ;(this as any).cHardCollision  = (this.kHardCollision  as any).compute(N)
    ;(this as any).cBehavior       = (this.kBehavior       as any).compute(N)
    ;(this as any).cPositions      = (this.kPositions      as any).compute(N)

    spawnParticles(this.buf, this.settings, N)
  }

  // Called from main.ts on each animation frame
  async update(frameTime: number): Promise<void> {
    if (this.paused) return

    const dt = (frameTime / this.iterationsPerFrame) * this.timeScale
    this.buf.updateStateParams(this.settings.idealNeighborCount)
    updateSimUniforms(
      this.uniforms, this.settings, dt, this.N,
      this.interactionPoint, this.interactionStrength,
    )

    for (let iter = 0; iter < this.iterationsPerFrame; iter++) {
      await this.runStep()
    }
  }

  private async runStep(): Promise<void> {
    const N = this.N
    await this.renderer.computeAsync(this.cExternalForces)
    await this.renderer.computeAsync(this.cSpatialHash)
    await this.sort.sortAndCalculateOffsets(this.renderer, N)
    await this.renderer.computeAsync(this.cHardCollision)
    await this.renderer.computeAsync(this.cBehavior)
    await this.renderer.computeAsync(this.cPositions)
  }

  // --- CPU state machine (mirrors SwarmSimulation.cs UpdateStatesCPU) ---

  setInteraction(worldX: number, worldY: number, strength: number): void {
    this.interactionPoint.x = worldX
    this.interactionPoint.y = worldY
    this.interactionStrength = strength
  }

  clearInteraction(): void {
    this.interactionStrength = 0
  }

  // Trigger panic for particles within panicRadius of worldX/worldY.
  // Call each frame while RMB is held (with positive strength).
  triggerPanic(worldX: number, worldY: number, panicRadius: number): void {
    const { positionsCPU, statesCPU } = this.buf
    const r2 = panicRadius * panicRadius
    let changed = false
    for (let i = 0; i < this.N; i++) {
      const px = positionsCPU[i * 2 + 0]
      const py = positionsCPU[i * 2 + 1]
      const dx = px - worldX, dy = py - worldY
      if (dx * dx + dy * dy < r2) {
        if (statesCPU[i] !== AgentState.Scared) {
          statesCPU[i] = AgentState.Scared
          this.scaredTimers[i] = CALM_DURATION_FRAMES
          changed = true
        }
      }
    }
    if (changed) this.buf.markStatesForUpload()
  }

  // Tick scared timers down; return to Calm when expired.
  tickStateTimers(): void {
    let changed = false
    for (let i = 0; i < this.N; i++) {
      if (this.buf.statesCPU[i] === AgentState.Scared) {
        this.scaredTimers[i]--
        if (this.scaredTimers[i] <= 0) {
          this.buf.statesCPU[i] = AgentState.Calm
          changed = true
        }
      }
    }
    if (changed) this.buf.markStatesForUpload()
  }

  setAllHuddle(on: boolean): void {
    const state = on ? AgentState.Huddle : AgentState.Calm
    this.buf.statesCPU.fill(state)
    this.buf.markStatesForUpload()
  }
}
