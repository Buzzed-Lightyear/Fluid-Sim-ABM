import { describe, test, expect } from 'vitest'
import { ReferenceSim, v2, AgentState } from '../ReferenceSim.js'
import { DEFAULT_SETTINGS } from '../../types/SimSettings.js'
import type { Vec2 } from '../ReferenceSim.js'

function grid(n: number, spacing: number): { pos: Vec2[], vel: Vec2[] } {
  const side = Math.ceil(Math.sqrt(n))
  const pos: Vec2[] = [], vel: Vec2[] = []
  for (let i = 0; i < n; i++) {
    const x = i % side, y = Math.floor(i / side)
    pos.push(v2((x - side / 2) * spacing, (y - side / 2) * spacing))
    vel.push(v2(0, 0))
  }
  return { pos, vel }
}

function mulberry32(seed: number) {
  return function () {
    seed |= 0; seed = seed + 0x6D2B79F5 | 0
    let t = Math.imul(seed ^ seed >>> 15, 1 | seed)
    t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t
    return ((t ^ t >>> 14) >>> 0) / 4294967296
  }
}

describe('TaskCTests', () => {
  test('CalmAllAgents_MatchesBaseline', () => {
    const s = { ...DEFAULT_SETTINGS }
    const { pos: posA, vel: velA } = grid(9, 0.3)
    const { pos: posB, vel: velB } = grid(9, 0.3)

    const a = new ReferenceSim(s, posA, velA)
    const b = new ReferenceSim(s, posB, velB)
    for (let i = 0; i < 9; i++) b.states[i] = AgentState.Calm

    for (let i = 0; i < 300; i++) { a.step(1 / 180); b.step(1 / 180) }

    for (let i = 0; i < 9; i++) {
      expect(a.positions[i].x).toBe(b.positions[i].x)
      expect(a.positions[i].y).toBe(b.positions[i].y)
    }
  })

  test('ScaredAgent_IdealZero_SeparatesFromNeighbors', () => {
    const s = {
      ...DEFAULT_SETTINGS,
      gravity: 0, drag: 0,
      pressureMultiplier: 20,
      sensorRadius: 1.0,
      idealNeighborCount: 8,
    }
    const { pos, vel } = grid(9, 0.15)
    const sim = new ReferenceSim(s, pos, vel)
    const scaredIdx = 4

    let startCX = 0, startCY = 0
    for (const p of pos) { startCX += p.x; startCY += p.y }
    startCX /= pos.length; startCY /= pos.length
    const startDst = Math.sqrt((pos[scaredIdx].x - startCX) ** 2 + (pos[scaredIdx].y - startCY) ** 2)

    sim.states[scaredIdx] = AgentState.Scared
    for (let i = 0; i < 200; i++) sim.step(0.01)

    let endCX = 0, endCY = 0
    for (const p of sim.positions) { endCX += p.x; endCY += p.y }
    endCX /= sim.positions.length; endCY /= sim.positions.length
    const endDst = Math.sqrt((sim.positions[scaredIdx].x - endCX) ** 2 + (sim.positions[scaredIdx].y - endCY) ** 2)

    expect(endDst).toBeGreaterThan(startDst)
  })

  test('HuddleAgents_ClumpTighter', () => {
    const s = {
      ...DEFAULT_SETTINGS,
      gravity: 0, drag: 2,
      boundsSize: [100, 100] as [number, number],
      sensorRadius: 10,
    }
    const rng = mulberry32(42)
    const n = 20
    const pos: Vec2[] = [], vel: Vec2[] = []
    for (let i = 0; i < n; i++) {
      pos.push(v2(rng() * 5 - 2.5, rng() * 5 - 2.5))
      vel.push(v2(0, 0))
    }

    const calm = new ReferenceSim(s, pos, vel)
    const hudd = new ReferenceSim(s, pos, vel)
    for (let i = 0; i < n; i++) hudd.states[i] = AgentState.Huddle

    for (let i = 0; i < 500; i++) { calm.step(0.01); hudd.step(0.01) }

    function meanDst(arr: Vec2[]): number {
      let sum = 0, count = 0
      for (let i = 0; i < arr.length; i++)
        for (let j = i + 1; j < arr.length; j++) {
          sum += Math.sqrt((arr[i].x - arr[j].x) ** 2 + (arr[i].y - arr[j].y) ** 2)
          count++
        }
      return sum / count
    }

    const calmMean = meanDst(calm.positions)
    const huddMean = meanDst(hudd.positions)
    expect(huddMean).toBeLessThan(calmMean)
  })

  test('StateTransition_ScaredMaxSpeedHigher', () => {
    const s = {
      ...DEFAULT_SETTINGS,
      gravity: -1000,
      drag: 0,
      boundsSize: [1e6, 1e6] as [number, number],
      maxForce: 1e6,
    }
    const calm = new ReferenceSim(s, [v2(0, 0)], [v2(0, 0)])
    const scared = new ReferenceSim(s, [v2(0, 0)], [v2(0, 0)])
    scared.states[0] = AgentState.Scared

    for (let i = 0; i < 500; i++) { calm.step(0.01); scared.step(0.01) }

    const calmSpeed = Math.sqrt(calm.velocities[0].x ** 2 + calm.velocities[0].y ** 2)
    const scaredSpeed = Math.sqrt(scared.velocities[0].x ** 2 + scared.velocities[0].y ** 2)
    expect(calmSpeed).toBeCloseTo(s.maxSpeed, 3)
    expect(scaredSpeed).toBeCloseTo(s.maxSpeed * 1.5, 3)
  })

  test('UnknownStateValue_FallsBackToCalm', () => {
    const s = { ...DEFAULT_SETTINGS }
    const { pos: posA, vel: velA } = grid(4, 0.3)
    const { pos: posB, vel: velB } = grid(4, 0.3)

    const calm = new ReferenceSim(s, posA, velA)
    const unknown = new ReferenceSim(s, posB, velB)
    unknown.states[0] = 99

    for (let i = 0; i < 100; i++) { calm.step(1 / 180); unknown.step(1 / 180) }

    for (let i = 0; i < 4; i++) {
      expect(calm.positions[i].x).toBe(unknown.positions[i].x)
      expect(calm.positions[i].y).toBe(unknown.positions[i].y)
    }
  })
})
