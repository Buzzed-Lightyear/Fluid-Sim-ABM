import type { ReferenceSim } from './ReferenceSim.js'
import { v2len, v2sub } from './ReferenceSim.js'

export function densityStdDev(sim: ReferenceSim): number {
  const n = sim.numParticles
  if (n === 0) return 0
  let mean = 0
  for (let i = 0; i < n; i++) mean += sim.densities[i].x
  mean /= n
  let sumSq = 0
  for (let i = 0; i < n; i++) {
    const d = sim.densities[i].x - mean
    sumSq += d * d
  }
  return Math.sqrt(sumSq / n)
}

export function avgSpeed(sim: ReferenceSim): number {
  const n = sim.numParticles
  if (n === 0) return 0
  let sum = 0
  for (let i = 0; i < n; i++) sum += v2len(sim.velocities[i])
  return sum / n
}

export function minPairwiseDistance(sim: ReferenceSim): number {
  const n = sim.numParticles
  if (n < 2) return Number.MAX_VALUE
  let min = Number.MAX_VALUE
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      const d = v2len(v2sub(sim.positions[i], sim.positions[j]))
      if (d < min) min = d
    }
  }
  return min
}

export function maxCentroidDistance(sim: ReferenceSim): number {
  const n = sim.numParticles
  if (n === 0) return 0
  let cx = 0, cy = 0
  for (let i = 0; i < n; i++) { cx += sim.positions[i].x; cy += sim.positions[i].y }
  cx /= n; cy /= n
  let max = 0
  for (let i = 0; i < n; i++) {
    const d = v2len(v2sub(sim.positions[i], { x: cx, y: cy }))
    if (d > max) max = d
  }
  return max
}
