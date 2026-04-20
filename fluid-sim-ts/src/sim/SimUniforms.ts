// @ts-nocheck
import { uniform } from 'three/tsl'
import { Vector2 } from 'three'
import type { SimSettings } from '../types/SimSettings.js'

export function createSimUniforms() {
  return {
    numParticles:             uniform(0, 'uint'),
    gravity:                  uniform(0, 'float'),
    deltaTime:                uniform(0, 'float'),
    wallBounciness:           uniform(0, 'float'),
    sensorRadius:             uniform(0, 'float'),
    pressureMultiplier:       uniform(0, 'float'),
    viscosity:                uniform(0, 'float'),
    drag:                     uniform(0, 'float'),
    cohesionStrength:         uniform(0, 'float'),
    collisionStiffness:       uniform(0, 'float'),
    maxSpeed:                 uniform(0, 'float'),
    maxForce:                 uniform(0, 'float'),
    boundsSize:               uniform(new Vector2()),
    interactionInputPoint:    uniform(new Vector2()),
    interactionInputStrength: uniform(0, 'float'),
    interactionInputRadius:   uniform(0, 'float'),
    obstacleSize:             uniform(new Vector2()),
    obstacleCentre:           uniform(new Vector2()),
  }
}

export type SimUniforms = ReturnType<typeof createSimUniforms>

export function updateSimUniforms(
  u: SimUniforms, s: SimSettings, dt: number, N: number,
  interactionPoint: { x: number; y: number },
  interactionStrength: number,
): void {
  u.numParticles.value = N
  u.gravity.value = s.gravity
  u.deltaTime.value = dt
  u.wallBounciness.value = s.wallBounciness
  u.sensorRadius.value = s.sensorRadius
  u.pressureMultiplier.value = s.pressureMultiplier
  u.viscosity.value = s.viscosity
  u.drag.value = s.drag
  u.cohesionStrength.value = s.cohesionStrength
  u.collisionStiffness.value = s.collisionStiffness
  u.maxSpeed.value = s.maxSpeed
  u.maxForce.value = s.maxForce
  u.boundsSize.value.set(s.boundsSize[0], s.boundsSize[1])
  u.interactionInputPoint.value.set(interactionPoint.x, interactionPoint.y)
  u.interactionInputStrength.value = interactionStrength
  u.interactionInputRadius.value = s.interactionRadius
  u.obstacleSize.value.set(s.obstacleSize[0], s.obstacleSize[1])
  u.obstacleCentre.value.set(s.obstacleCentre[0], s.obstacleCentre[1])
}
