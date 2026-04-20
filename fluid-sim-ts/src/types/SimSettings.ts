export interface SimSettings {
  gravity: number
  wallBounciness: number
  boundsSize: [number, number]
  sensorRadius: number
  idealNeighborCount: number
  pressureMultiplier: number
  viscosity: number
  drag: number
  collisionRadius: number
  collisionStiffness: number
  cohesionStrength: number
  maxSpeed: number
  maxForce: number
  interactionRadius: number
  obstacleSize: [number, number]
  obstacleCentre: [number, number]
}

export const DEFAULT_SETTINGS: SimSettings = {
  gravity: -9.81,
  wallBounciness: 0.24,
  boundsSize: [17, 9],
  sensorRadius: 1.0,
  idealNeighborCount: 8,
  pressureMultiplier: 8,
  viscosity: 1.0,
  drag: 0.1,
  collisionRadius: 0.05,
  collisionStiffness: 20,
  cohesionStrength: 0.5,
  maxSpeed: 5,
  maxForce: 50,
  interactionRadius: 3,
  obstacleSize: [0, 0],
  obstacleCentre: [0, 0],
}
