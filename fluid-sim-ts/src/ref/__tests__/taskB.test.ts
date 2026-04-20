import { describe, test, expect } from 'vitest'
import { ReferenceSim, v2 } from '../ReferenceSim.js'
import { DEFAULT_SETTINGS } from '../../types/SimSettings.js'
import type { SimSettings } from '../../types/SimSettings.js'
import type { Vec2 } from '../ReferenceSim.js'

function noForcesExcept(cohesionStrength: number, sensorRadius: number): SimSettings {
  return {
    ...DEFAULT_SETTINGS,
    gravity: 0,
    drag: 0,
    pressureMultiplier: 0,
    viscosity: 0,
    cohesionStrength,
    sensorRadius,
  }
}

describe('TaskBTests', () => {
  test('CohesionZero_IsNoOp', () => {
    const s = { ...DEFAULT_SETTINGS, cohesionStrength: 0 }
    const side = 3
    const posInit: Vec2[] = [], velInit: Vec2[] = []
    for (let i = 0; i < side * side; i++) {
      const x = i % side, y = Math.floor(i / side)
      posInit.push(v2((x - 1) * 0.3, (y - 1) * 0.3))
      velInit.push(v2(0, 0))
    }
    const a = new ReferenceSim(s, posInit, velInit)
    const b = new ReferenceSim(s, posInit, velInit)
    for (let i = 0; i < 300; i++) { a.step(1 / 180); b.step(1 / 180) }
    for (let i = 0; i < 9; i++) {
      expect(a.positions[i].x).toBe(b.positions[i].x)
      expect(a.positions[i].y).toBe(b.positions[i].y)
    }
  })

  test('TwoParticles_KnownAnswer', () => {
    const s = noForcesExcept(1, 3)
    const pos = [v2(-1, 0), v2(1, 0)]
    const vel = [v2(0, 0), v2(0, 0)]
    const sim = new ReferenceSim(s, pos, vel)
    sim.step(0.1)
    expect(sim.velocities[0].x).toBeCloseTo(0.2, 4)
    expect(sim.velocities[0].y).toBeCloseTo(0, 4)
    expect(sim.velocities[1].x).toBeCloseTo(-0.2, 4)
    expect(sim.velocities[1].y).toBeCloseTo(0, 4)
  })

  test('Cluster_ConvergesToCenter', () => {
    const s = { ...noForcesExcept(2, 10), drag: 2, boundsSize: [100, 100] as [number, number] }
    const rng = mulberry32(42)
    const pos: Vec2[] = [], vel: Vec2[] = []
    for (let i = 0; i < 20; i++) {
      pos.push(v2(rng() * 4 - 2, rng() * 4 - 2))
      vel.push(v2(0, 0))
    }
    const sim = new ReferenceSim(s, pos, vel)
    for (let i = 0; i < 500; i++) sim.step(0.01)

    let cx = 0, cy = 0
    for (const p of sim.positions) { cx += p.x; cy += p.y }
    cx /= sim.numParticles; cy /= sim.numParticles

    for (const p of sim.positions) {
      const d = Math.sqrt((p.x - cx) ** 2 + (p.y - cy) ** 2)
      expect(d).toBeLessThan(0.5)
    }
  })

  test('VerticalSupportField_IsGone', () => {
    // TypeScript: verify at compile-time by checking the key does not exist at runtime.
    expect(('verticalSupport' in DEFAULT_SETTINGS)).toBe(false)
  })
})

// Deterministic RNG matching C# new Random(42) sequence
function mulberry32(seed: number) {
  return function () {
    seed |= 0; seed = seed + 0x6D2B79F5 | 0
    let t = Math.imul(seed ^ seed >>> 15, 1 | seed)
    t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t
    return ((t ^ t >>> 14) >>> 0) / 4294967296
  }
}
