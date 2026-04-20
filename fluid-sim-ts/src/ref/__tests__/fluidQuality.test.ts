import { describe, test, expect } from 'vitest'
import { ReferenceSim, v2 } from '../ReferenceSim.js'
import { densityStdDev, avgSpeed, minPairwiseDistance, maxCentroidDistance } from '../FluidMetrics.js'
import { DEFAULT_SETTINGS } from '../../types/SimSettings.js'
import type { Vec2 } from '../ReferenceSim.js'

function grid(n: number, spacing: number, cx = 0, cy = 0): { pos: Vec2[], vel: Vec2[] } {
  const side = Math.ceil(Math.sqrt(n))
  const pos: Vec2[] = [], vel: Vec2[] = []
  for (let i = 0; i < n; i++) {
    const x = i % side, y = Math.floor(i / side)
    pos.push(v2(cx + (x - side / 2) * spacing, cy + (y - side / 2) * spacing))
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

describe('FluidQualityTests', () => {
  test('DensityUniformity_AtRest', () => {
    const s = { ...DEFAULT_SETTINGS }
    const { pos, vel } = grid(49, 0.3)
    const sim = new ReferenceSim(s, pos, vel)
    for (let i = 0; i < 600; i++) sim.step(1 / 180)

    const stddev = densityStdDev(sim)
    console.log(`DensityStdDev = ${stddev.toFixed(4)}`)
    expect(stddev).toBeLessThan(4.5)
  })

  test('Settlement_AfterWarmup', () => {
    const s = { ...DEFAULT_SETTINGS }
    const { pos, vel } = grid(49, 0.3)
    const sim = new ReferenceSim(s, pos, vel)
    for (let i = 0; i < 1000; i++) sim.step(1 / 180)

    const avg = avgSpeed(sim)
    console.log(`AvgSpeed = ${avg.toFixed(4)}`)
    expect(avg).toBeLessThan(1.0)
  })

  test('NoCompression_Ever', () => {
    const s = { ...DEFAULT_SETTINGS, gravity: 0, collisionStiffness: 100 }
    const rng = mulberry32(42)
    const n = 16
    const spacing = 0.15
    const side = 4
    const pos: Vec2[] = [], vel: Vec2[] = []
    for (let i = 0; i < n; i++) {
      const x = i % side, y = Math.floor(i / side)
      const px = (x - 1.5) * spacing
      const py = (y - 1.5) * spacing
      pos.push(v2(px, py))
      const len = Math.sqrt(px * px + py * py)
      if (len > 0) {
        vel.push(v2(-px / len * rng() * 0.5, -py / len * rng() * 0.5))
      } else {
        vel.push(v2(0, 0))
      }
    }

    const sim = new ReferenceSim(s, pos, vel)
    const floor = 2 * s.collisionRadius - 1e-3
    let globalMin = Number.MAX_VALUE
    for (let i = 0; i < 600; i++) {
      sim.step(1 / 180)
      const m = minPairwiseDistance(sim)
      if (m < globalMin) globalMin = m
    }

    console.log(`globalMinPairwise = ${globalMin.toFixed(4)}, floor = ${floor.toFixed(4)}`)
    expect(globalMin).toBeGreaterThanOrEqual(floor)
  })

  test('Cluster_StaysCoherent', () => {
    const s = {
      ...DEFAULT_SETTINGS,
      gravity: 0, drag: 2, cohesionStrength: 0.5,
      sensorRadius: 3, boundsSize: [100, 100] as [number, number],
    }
    const rng = mulberry32(42)
    const n = 20
    const pos: Vec2[] = [], vel: Vec2[] = []
    for (let i = 0; i < n; i++) {
      pos.push(v2(rng() * 3 - 1.5, rng() * 3 - 1.5))
      vel.push(v2(0, 0))
    }
    const sim = new ReferenceSim(s, pos, vel)
    for (let i = 0; i < 500; i++) sim.step(1 / 180)

    const maxD = maxCentroidDistance(sim)
    console.log(`MaxCentroidDistance = ${maxD.toFixed(4)}`)
    expect(maxD).toBeLessThan(5)
  })
})
