import { describe, test, expect } from 'vitest'
import { ReferenceSim, v2, AgentState } from '../ReferenceSim.js'
import { DEFAULT_SETTINGS } from '../../types/SimSettings.js'
import type { SimSettings } from '../../types/SimSettings.js'
import type { Vec2 } from '../ReferenceSim.js'

const EPS = 1e-4

function makeSim(n: number, settings?: SimSettings): ReferenceSim {
  const s = settings ?? { ...DEFAULT_SETTINGS }
  const side = Math.ceil(Math.sqrt(n))
  const spacing = 0.3
  const pos: Vec2[] = []
  const vel: Vec2[] = []
  for (let i = 0; i < n; i++) {
    const x = i % side, y = Math.floor(i / side)
    pos.push(v2((x - side / 2) * spacing, (y - side / 2) * spacing))
    vel.push(v2(0, 0))
  }
  return new ReferenceSim(s, pos, vel)
}

describe('BaselineTests', () => {
  test('NoNaN_AfterManySteps', () => {
    const sim = makeSim(16)
    for (let i = 0; i < 500; i++) sim.step(1 / 180)
    for (const p of sim.positions) {
      expect(isFinite(p.x) && isFinite(p.y)).toBe(true)
    }
    for (const v of sim.velocities) {
      expect(isFinite(v.x) && isFinite(v.y)).toBe(true)
    }
  })

  test('MaxSpeedEnforced', () => {
    const s = { ...DEFAULT_SETTINGS, maxSpeed: 2 }
    const sim = makeSim(16, s)
    for (let i = 0; i < 200; i++) sim.step(1 / 180)
    for (const v of sim.velocities) {
      const speed = Math.sqrt(v.x * v.x + v.y * v.y)
      expect(speed).toBeLessThanOrEqual(s.maxSpeed + EPS)
    }
  })

  test('ParticlesStayInsideBounds', () => {
    const s = { ...DEFAULT_SETTINGS, boundsSize: [4, 4] as [number, number], wallBounciness: 0 }
    const sim = makeSim(9, s)
    for (let i = 0; i < 300; i++) sim.step(1 / 180)
    const halfX = s.boundsSize[0] * 0.5
    const halfY = s.boundsSize[1] * 0.5
    for (const p of sim.positions) {
      expect(Math.abs(p.x)).toBeLessThanOrEqual(halfX + EPS)
      expect(Math.abs(p.y)).toBeLessThanOrEqual(halfY + EPS)
    }
  })

  test('SingleParticle_GravityOnly_FreeFalls', () => {
    const s = { ...DEFAULT_SETTINGS, drag: 0 }
    const sim = new ReferenceSim(s, [v2(0, 0)], [v2(0, 0)])
    sim.step(0.01)
    expect(sim.velocities[0].x).toBeCloseTo(0, 5)
    expect(sim.velocities[0].y).toBeCloseTo(s.gravity * 0.01, 5)
  })

  test('OverlappingParticles_MoveApart', () => {
    const s = { ...DEFAULT_SETTINGS, gravity: 0, drag: 0.2, collisionRadius: 0.5, sensorRadius: 1.5 }
    const pos = [v2(-0.4, 0), v2(0.4, 0)]
    const vel = [v2(0, 0), v2(0, 0)]
    const sim = new ReferenceSim(s, pos, vel)
    const startDst = Math.sqrt((pos[1].x - pos[0].x) ** 2 + (pos[1].y - pos[0].y) ** 2)
    for (let i = 0; i < 200; i++) sim.step(1 / 240)
    const d = sim.positions[1].x - sim.positions[0].x
    const finalDst = Math.sqrt(d * d)
    expect(finalDst).toBeGreaterThan(startDst)
  })

  test('Deterministic_SameSeed_SameTrajectory', () => {
    const a = makeSim(9)
    const b = makeSim(9)
    for (let i = 0; i < 50; i++) { a.step(1 / 180); b.step(1 / 180) }
    for (let i = 0; i < 9; i++) {
      expect(a.positions[i].x).toBe(b.positions[i].x)
      expect(a.positions[i].y).toBe(b.positions[i].y)
    }
  })
})
