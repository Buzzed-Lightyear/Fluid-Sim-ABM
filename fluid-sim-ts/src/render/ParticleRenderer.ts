// @ts-nocheck
import { BufferGeometry, Points, Scene } from 'three'
import { buildParticleMaterial } from './particleMaterial.js'
import type { BufferManager } from '../sim/BufferManager.js'

export class ParticleRenderer {
  private readonly points: Points
  readonly material: ReturnType<typeof buildParticleMaterial>['material']

  constructor(scene: Scene, buf: BufferManager, N: number, maxSpeed = 5) {
    // Minimal geometry — no position attribute needed since we read from storage buffer
    const geometry = new BufferGeometry()
    geometry.setDrawRange(0, N)

    const { material, maxSpeedU } = buildParticleMaterial(buf, maxSpeed)
    this.material = material

    this.points = new Points(geometry, material)
    this.points.frustumCulled = false  // always draw all particles
    scene.add(this.points)
  }

  dispose(): void {
    this.points.geometry.dispose()
    this.material.dispose()
  }
}
