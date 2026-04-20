import type { BufferManager } from './BufferManager.js'
import type { SimSettings } from '../types/SimSettings.js'

// Grid-based spawner, port of ParticleSpawner.cs
export function spawnParticles(buf: BufferManager, settings: SimSettings, N: number): void {
  const side = Math.ceil(Math.sqrt(N))
  const spacing = settings.sensorRadius * 0.5
  const jitter = 0.01

  for (let i = 0; i < N; i++) {
    const col = i % side
    const row = Math.floor(i / side)
    const x = (col - side / 2 + 0.5) * spacing + (Math.random() - 0.5) * jitter
    const y = (row - side / 2 + 0.5) * spacing + (Math.random() - 0.5) * jitter

    buf.positionsCPU[i * 2 + 0] = x
    buf.positionsCPU[i * 2 + 1] = y
    buf.velocitiesCPU[i * 2 + 0] = 0
    buf.velocitiesCPU[i * 2 + 1] = 0
    buf.collisionRadiiCPU[i] = settings.collisionRadius
    buf.statesCPU[i] = 0  // Calm
  }

  buf.markPositionsForUpload()
  buf.markStatesForUpload()
  buf.updateStateParams(settings.idealNeighborCount)
}
