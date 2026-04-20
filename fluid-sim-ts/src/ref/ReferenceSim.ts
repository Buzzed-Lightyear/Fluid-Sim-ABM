// CPU oracle: faithful port of Assets/Scripts/Sim 2D/Compute/SwarmSim.compute
// via Tests/AbmReference/ReferenceSim.cs.
// No Three.js imports — pure TypeScript with brute-force neighbor loops.
// Keep in sync with the HLSL kernel when editing physics formulas.

import type { SimSettings } from '../types/SimSettings.js'

export const AgentState = { Calm: 0, Scared: 1, Huddle: 2 } as const
export type AgentStateValue = typeof AgentState[keyof typeof AgentState]

export interface Vec2 { x: number; y: number }

export function v2(x: number, y: number): Vec2 { return { x, y } }
export function v2add(a: Vec2, b: Vec2): Vec2 { return { x: a.x + b.x, y: a.y + b.y } }
export function v2sub(a: Vec2, b: Vec2): Vec2 { return { x: a.x - b.x, y: a.y - b.y } }
export function v2scale(a: Vec2, s: number): Vec2 { return { x: a.x * s, y: a.y * s } }
export function v2dot(a: Vec2, b: Vec2): number { return a.x * b.x + a.y * b.y }
export function v2len(a: Vec2): number { return Math.sqrt(v2dot(a, a)) }
export function v2norm(a: Vec2): Vec2 { const l = v2len(a); return l > 0 ? v2scale(a, 1 / l) : { x: 0, y: 0 } }
export function v2divs(a: Vec2, s: number): Vec2 { return { x: a.x / s, y: a.y / s } }

export class ReferenceSim {
  settings: SimSettings
  positions: Vec2[]
  velocities: Vec2[]
  predictedPositions: Vec2[]
  densities: Vec2[]
  collisionRadii: number[]
  states: number[]
  interactionPoint: Vec2 = v2(0, 0)
  interactionStrength: number = 0

  get numParticles(): number { return this.positions.length }

  constructor(settings: SimSettings, initialPositions: Vec2[], initialVelocities: Vec2[]) {
    this.settings = settings
    this.positions = initialPositions.map(p => ({ ...p }))
    this.velocities = initialVelocities.map(v => ({ ...v }))
    this.predictedPositions = initialPositions.map(p => ({ ...p }))
    this.densities = initialPositions.map(() => v2(0, 0))
    this.collisionRadii = initialPositions.map(() => settings.collisionRadius)
    this.states = new Array(initialPositions.length).fill(0)
  }

  private getStateParams(s: number): { ideal: number; cohMult: number; speedMult: number } {
    if (s === AgentState.Scared) return { ideal: 0, cohMult: 0, speedMult: 1.5 }
    if (s === AgentState.Huddle) return { ideal: 20, cohMult: 3, speedMult: 1.0 }
    return { ideal: this.settings.idealNeighborCount, cohMult: 1, speedMult: 1 }
  }

  step(dt: number): void {
    this.externalForces(dt)
    this.hardCollision(dt)
    this.updateBehavior(dt)
    this.updatePositions(dt)
  }

  // Pre-Task-A fused kernel. Kept as oracle for the null-hypothesis baseline test.
  legacyStep(dt: number): void {
    this.externalForces(dt)
    this.legacyUpdateBoids(dt)
    this.updatePositions(dt)
  }

  private externalForces(dt: number): void {
    const { interactionPoint: ip, interactionStrength: is_ } = this
    const r = this.settings.interactionRadius
    for (let i = 0; i < this.numParticles; i++) {
      const force = v2(0, this.settings.gravity)

      if (is_ !== 0) {
        const offset = v2sub(ip, this.positions[i])
        const sqrDst = v2dot(offset, offset)
        if (sqrDst < r * r) {
          const dst = Math.sqrt(sqrDst)
          const dir = v2divs(offset, dst)
          const t = 1 - dst / r
          force.x += dir.x * (is_ * t)
          force.y += dir.y * (is_ * t)
        }
      }

      this.velocities[i] = v2add(this.velocities[i], v2scale(force, dt))
      this.predictedPositions[i] = v2add(this.positions[i], v2scale(this.velocities[i], 1 / 120))
    }
  }

  private hardCollision(dt: number): void {
    const n = this.numParticles
    for (let i = 0; i < n; i++) {
      const pos = this.predictedPositions[i]
      const colForce = v2(0, 0)

      for (let j = 0; j < n; j++) {
        if (j === i) continue
        const neighborPos = this.predictedPositions[j]
        const offset = v2sub(neighborPos, pos)
        const sqrDst = v2dot(offset, offset)

        const minDst = this.collisionRadii[i] + this.collisionRadii[j]
        if (sqrDst >= minDst * minDst) continue

        const dst = Math.sqrt(sqrDst)
        if (dst <= 0.0001) continue

        const kick = (minDst - dst) / minDst
        const f = kick * kick * this.settings.collisionStiffness
        colForce.x -= (offset.x / dst) * f
        colForce.y -= (offset.y / dst) * f
      }

      this.velocities[i] = v2add(this.velocities[i], v2scale(colForce, dt))
    }
  }

  private updateBehavior(dt: number): void {
    const n = this.numParticles
    const searchRad = this.settings.sensorRadius

    for (let i = 0; i < n; i++) {
      const pos = this.predictedPositions[i]
      const myVel = this.velocities[i]

      const pressureForce = v2(0, 0)
      const viscosityForce = v2(0, 0)
      const avgPos = v2(0, 0)
      let neighborCount = 0

      for (let j = 0; j < n; j++) {
        if (j === i) continue
        const neighborPos = this.predictedPositions[j]
        const offset = v2sub(neighborPos, pos)
        const sqrDst = v2dot(offset, offset)

        if (sqrDst >= searchRad * searchRad) continue

        const dst = Math.sqrt(sqrDst)
        if (dst <= 0.0001) continue

        if (dst < this.settings.sensorRadius) {
          neighborCount++
          avgPos.x += neighborPos.x
          avgPos.y += neighborPos.y
          pressureForce.x -= (offset.x / dst) * (1 / dst)
          pressureForce.y -= (offset.y / dst) * (1 / dst)
          const nv = this.velocities[j]
          viscosityForce.x += (nv.x - myVel.x) / dst
          viscosityForce.y += (nv.y - myVel.y) / dst
        }
      }

      const { ideal, cohMult, speedMult } = this.getStateParams(this.states[i])
      const totalForce = v2(0, 0)

      if (neighborCount > 0) {
        const densityError = neighborCount - ideal
        if (densityError > 0) {
          totalForce.x += pressureForce.x * this.settings.pressureMultiplier * densityError
          totalForce.y += pressureForce.y * this.settings.pressureMultiplier * densityError
        }
        totalForce.x += viscosityForce.x * this.settings.viscosity
        totalForce.y += viscosityForce.y * this.settings.viscosity

        const ap = v2divs(avgPos, neighborCount)
        totalForce.x += (ap.x - pos.x) * this.settings.cohesionStrength * cohMult
        totalForce.y += (ap.y - pos.y) * this.settings.cohesionStrength * cohMult
      }

      totalForce.x -= myVel.x * this.settings.drag
      totalForce.y -= myVel.y * this.settings.drag

      const fLen = v2len(totalForce)
      if (fLen > this.settings.maxForce) {
        totalForce.x = (totalForce.x / fLen) * this.settings.maxForce
        totalForce.y = (totalForce.y / fLen) * this.settings.maxForce
      }

      this.velocities[i] = v2add(this.velocities[i], v2scale(totalForce, dt))

      const speedCap = this.settings.maxSpeed * speedMult
      const vLen = v2len(this.velocities[i])
      if (vLen > speedCap) {
        this.velocities[i] = v2scale(v2divs(this.velocities[i], vLen), speedCap)
      }

      this.densities[i] = v2(neighborCount, 0)
    }
  }

  // Pre-Task-A fused kernel (oracle for legacy baseline test).
  private legacyUpdateBoids(dt: number): void {
    const n = this.numParticles
    const searchRad = Math.max(this.settings.sensorRadius, this.settings.collisionRadius * 2.01)

    for (let i = 0; i < n; i++) {
      const pos = this.predictedPositions[i]
      const myVel = this.velocities[i]

      const pressureForce = v2(0, 0)
      const viscosityForce = v2(0, 0)
      const colForce = v2(0, 0)
      let neighborCount = 0

      for (let j = 0; j < n; j++) {
        if (j === i) continue
        const neighborPos = this.predictedPositions[j]
        const offset = v2sub(neighborPos, pos)
        const sqrDst = v2dot(offset, offset)

        if (sqrDst >= searchRad * searchRad) continue

        const dst = Math.sqrt(sqrDst)
        if (dst <= 0.0001) continue

        const minDst = this.settings.collisionRadius * 2
        if (dst < minDst) {
          const kick = (minDst - dst) / minDst
          colForce.x -= (offset.x / dst) * (kick * kick * this.settings.collisionStiffness)
          colForce.y -= (offset.y / dst) * (kick * kick * this.settings.collisionStiffness)
        }

        if (dst < this.settings.sensorRadius) {
          neighborCount++
          pressureForce.x -= (offset.x / dst) * (1 / dst)
          pressureForce.y -= (offset.y / dst) * (1 / dst)
          const nv = this.velocities[j]
          viscosityForce.x += (nv.x - myVel.x) / dst
          viscosityForce.y += (nv.y - myVel.y) / dst
        }
      }

      const totalForce = { ...colForce }

      if (neighborCount > 0) {
        const densityError = neighborCount - this.settings.idealNeighborCount
        if (densityError > 0) {
          totalForce.x += pressureForce.x * this.settings.pressureMultiplier * densityError
          totalForce.y += pressureForce.y * this.settings.pressureMultiplier * densityError
        }
        totalForce.x += viscosityForce.x * this.settings.viscosity
        totalForce.y += viscosityForce.y * this.settings.viscosity
      }

      totalForce.x -= myVel.x * this.settings.drag
      totalForce.y -= myVel.y * this.settings.drag

      const fLen = v2len(totalForce)
      if (fLen > this.settings.maxForce) {
        totalForce.x = (totalForce.x / fLen) * this.settings.maxForce
        totalForce.y = (totalForce.y / fLen) * this.settings.maxForce
      }

      this.velocities[i] = v2add(this.velocities[i], v2scale(totalForce, dt))

      const vLen = v2len(this.velocities[i])
      if (vLen > this.settings.maxSpeed) {
        this.velocities[i] = v2scale(v2divs(this.velocities[i], vLen), this.settings.maxSpeed)
      }

      this.densities[i] = v2(neighborCount, 0)
    }
  }

  private updatePositions(dt: number): void {
    const halfW = this.settings.boundsSize[0] * 0.5
    const halfH = this.settings.boundsSize[1] * 0.5
    const bounce = this.settings.wallBounciness
    const [osX, osY] = this.settings.obstacleSize
    const [ocX, ocY] = this.settings.obstacleCentre
    const ohX = osX * 0.5
    const ohY = osY * 0.5

    for (let i = 0; i < this.numParticles; i++) {
      this.positions[i] = v2add(this.positions[i], v2scale(this.velocities[i], dt))
      let { x: px, y: py } = this.positions[i]
      let { x: vx, y: vy } = this.velocities[i]

      if (Math.abs(px) > halfW) {
        px = halfW * Math.sign(px)
        vx *= -bounce
      }
      if (Math.abs(py) > halfH) {
        py = halfH * Math.sign(py)
        vy *= -bounce
      }

      const oedX = ohX - Math.abs(px - ocX)
      const oedY = ohY - Math.abs(py - ocY)
      if (oedX >= 0 && oedY >= 0) {
        if (oedX < oedY) {
          px = ohX * Math.sign(px - ocX) + ocX
          vx *= -bounce
        } else {
          py = ohY * Math.sign(py - ocY) + ocY
          vy *= -bounce
        }
      }

      this.positions[i] = v2(px, py)
      this.velocities[i] = v2(vx, vy)
    }
  }
}
