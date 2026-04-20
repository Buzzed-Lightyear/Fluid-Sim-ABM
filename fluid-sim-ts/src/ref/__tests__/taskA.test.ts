import { describe, test, expect } from 'vitest'
import { ReferenceSim, v2 } from '../ReferenceSim.js'
import { DEFAULT_SETTINGS } from '../../types/SimSettings.js'
import type { Vec2 } from '../ReferenceSim.js'

const EPS = 1e-4

function grid(n: number, spacing: number): { pos: Vec2[], vel: Vec2[] } {
  const side = Math.ceil(Math.sqrt(n))
  const pos: Vec2[] = []
  const vel: Vec2[] = []
  for (let i = 0; i < n; i++) {
    const x = i % side, y = Math.floor(i / side)
    pos.push(v2((x - side / 2) * spacing, (y - side / 2) * spacing))
    vel.push(v2(0, 0))
  }
  return { pos, vel }
}

describe('TaskATests', () => {
  test('NullHypothesis_UniformRadii_MatchesLegacyBehavior', () => {
    const s = { ...DEFAULT_SETTINGS, cohesionStrength: 0 }
    const { pos: posA, vel: velA } = grid(16, 0.3)
    const { pos: posB, vel: velB } = grid(16, 0.3)

    const legacy = new ReferenceSim(s, posA, velA)
    const split = new ReferenceSim(s, posB, velB)

    for (let i = 0; i < 200; i++) {
      legacy.legacyStep(1 / 180)
      split.step(1 / 180)
    }

    for (let i = 0; i < 16; i++) {
      const dx = Math.abs(legacy.positions[i].x - split.positions[i].x)
      const dy = Math.abs(legacy.positions[i].y - split.positions[i].y)
      expect(dx).toBeLessThan(EPS)
      expect(dy).toBeLessThan(EPS)
    }
  })

  test('HardCollision_ResolvesOverlap_BeyondBehaviorAlone', () => {
    const s = { ...DEFAULT_SETTINGS, sensorRadius: 0, drag: 0.5 }
    const pos = [v2(-0.75, 0), v2(0.75, 0)]
    const vel = [v2(0, 0), v2(0, 0)]
    const sim = new ReferenceSim(s, pos, vel)
    sim.collisionRadii[0] = 1.0
    sim.collisionRadii[1] = 1.0

    for (let i = 0; i < 2000; i++) sim.step(1 / 240)

    const d = Math.sqrt((sim.positions[1].x - sim.positions[0].x) ** 2 + (sim.positions[1].y - sim.positions[0].y) ** 2)
    expect(d).toBeGreaterThanOrEqual(1.9)
  })

  test('HeterogeneousRadii_LargerAgentsPushFurther', () => {
    const s = { ...DEFAULT_SETTINGS, sensorRadius: 0 }
    const pos = [v2(-0.5, 0), v2(0, 0), v2(0.5, 0)]
    const vel = [v2(0, 0), v2(0, 0), v2(0, 0)]
    const sim = new ReferenceSim(s, pos, vel)
    sim.collisionRadii[0] = 0.1
    sim.collisionRadii[1] = 0.6
    sim.collisionRadii[2] = 0.1

    for (let i = 0; i < 2000; i++) sim.step(1 / 240)

    expect(sim.positions[0].x).toBeLessThan(-1)
    expect(sim.positions[2].x).toBeGreaterThan(1)
  })

  test('CollisionStiffnessZero_NoOp', () => {
    const s = { ...DEFAULT_SETTINGS, sensorRadius: 0, drag: 0.5, collisionStiffness: 0 }
    const pos = [v2(-0.75, 0), v2(0.75, 0)]
    const vel = [v2(0, 0), v2(0, 0)]
    const sim = new ReferenceSim(s, pos, vel)
    sim.collisionRadii[0] = 1.0
    sim.collisionRadii[1] = 1.0

    const startDst = Math.sqrt((pos[1].x - pos[0].x) ** 2)
    for (let i = 0; i < 100; i++) sim.step(1 / 240)
    const finalDst = Math.sqrt((sim.positions[1].x - sim.positions[0].x) ** 2)

    expect(Math.abs(finalDst - startDst)).toBeLessThan(EPS)
  })
})
