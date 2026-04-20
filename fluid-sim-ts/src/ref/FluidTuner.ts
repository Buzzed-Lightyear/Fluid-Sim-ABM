// Port of Tests/AbmReference/FluidTuner.cs
// Hill-climb optimizer for sim parameters. Only runs when RUN_TUNER=1 is set.

import { ReferenceSim, v2, type Vec2 } from './ReferenceSim.js'
import { densityStdDev, avgSpeed, minPairwiseDistance, maxCentroidDistance } from './FluidMetrics.js'
import { DEFAULT_SETTINGS, type SimSettings } from '../types/SimSettings.js'
import { writeFileSync } from 'fs'
import { join } from 'path'

interface TunableParams {
  pressureMultiplier: number
  idealNeighborCount: number
  viscosity: number
  cohesionStrength: number
  drag: number
  collisionStiffness: number
}

const NUDGE_FACTORS = [0.85, 1.15]
const W_DENSITY = 1, W_SETTLE = 2, W_CLUSTER = 0.5, W_COMPRESSION = 10
const SIM_STEPS = 600
const TUNER_N = 49  // 7×7 grid

function makeSim(params: TunableParams): ReferenceSim {
  const s: SimSettings = {
    ...DEFAULT_SETTINGS,
    pressureMultiplier: params.pressureMultiplier,
    idealNeighborCount: params.idealNeighborCount,
    viscosity: params.viscosity,
    cohesionStrength: params.cohesionStrength,
    drag: params.drag,
    collisionStiffness: params.collisionStiffness,
  }
  const side = Math.ceil(Math.sqrt(TUNER_N))
  const spacing = 0.3
  const pos: Vec2[] = [], vel: Vec2[] = []
  for (let i = 0; i < TUNER_N; i++) {
    const x = i % side, y = Math.floor(i / side)
    pos.push(v2((x - side / 2) * spacing, (y - side / 2) * spacing))
    vel.push(v2(0, 0))
  }
  return new ReferenceSim(s, pos, vel)
}

function evaluate(params: TunableParams): number {
  const sim = makeSim(params)
  for (let i = 0; i < SIM_STEPS; i++) sim.step(1 / 180)
  const density = densityStdDev(sim)
  const settle = avgSpeed(sim)
  const cluster = maxCentroidDistance(sim)
  const worstOverlap = Math.max(0, 2 * DEFAULT_SETTINGS.collisionRadius - minPairwiseDistance(sim))
  return -(W_DENSITY * density + W_SETTLE * settle + W_CLUSTER * cluster + W_COMPRESSION * worstOverlap)
}

export class FluidTuner {
  run(): TunableParams {
    let params: TunableParams = {
      pressureMultiplier: DEFAULT_SETTINGS.pressureMultiplier,
      idealNeighborCount: DEFAULT_SETTINGS.idealNeighborCount,
      viscosity: DEFAULT_SETTINGS.viscosity,
      cohesionStrength: DEFAULT_SETTINGS.cohesionStrength,
      drag: DEFAULT_SETTINGS.drag,
      collisionStiffness: DEFAULT_SETTINGS.collisionStiffness,
    }
    let best = evaluate(params)

    const keys = Object.keys(params) as Array<keyof TunableParams>

    for (let iter = 0; iter < 100; iter++) {
      let improved = false
      for (const key of keys) {
        for (const factor of NUDGE_FACTORS) {
          const candidate = { ...params, [key]: params[key] * factor }
          const score = evaluate(candidate)
          if (score > best) {
            best = score
            params = candidate
            improved = true
          }
        }
      }
      if (!improved) break
    }

    const outPath = join(process.cwd(), 'tuned_params.json')
    writeFileSync(outPath, JSON.stringify(params, null, 2))
    console.log(`Tuned params written to ${outPath}`)
    return params
  }
}
