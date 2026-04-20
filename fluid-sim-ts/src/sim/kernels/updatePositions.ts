// @ts-nocheck
import {
  Fn, If, Return, instanceIndex,
  float, abs, sign,
} from 'three/tsl'
import type { BufferManager } from '../BufferManager.js'
import type { SimUniforms } from '../SimUniforms.js'

export function buildUpdatePositionsKernel(buf: BufferManager, u: SimUniforms) {
  const { positions, velocities } = buf
  const { numParticles, deltaTime, boundsSize, wallBounciness, obstacleSize, obstacleCentre } = u

  return (Fn as any)((): void => {
    const i = instanceIndex
    If(i.greaterThanEqual(numParticles), () => Return())

    const vel = velocities.element(i).toVar()
    const pos = positions.element(i).add(vel.mul(deltaTime)).toVar()

    const halfW = boundsSize.x.mul(float(0.5)).toVar()
    const halfH = boundsSize.y.mul(float(0.5)).toVar()

    If(abs(pos.x).greaterThan(halfW), () => {
      pos.x.assign(halfW.mul(sign(pos.x)))
      vel.x.assign(vel.x.mul(float(-1).mul(wallBounciness)))
    })
    If(abs(pos.y).greaterThan(halfH), () => {
      pos.y.assign(halfH.mul(sign(pos.y)))
      vel.y.assign(vel.y.mul(float(-1).mul(wallBounciness)))
    })

    const ohX = obstacleSize.x.mul(float(0.5)).toVar()
    const ohY = obstacleSize.y.mul(float(0.5)).toVar()
    const oedX = ohX.sub(abs(pos.x.sub(obstacleCentre.x))).toVar()
    const oedY = ohY.sub(abs(pos.y.sub(obstacleCentre.y))).toVar()

    If(oedX.greaterThanEqual(float(0)).and(oedY.greaterThanEqual(float(0))), () => {
      If(oedX.lessThan(oedY), () => {
        pos.x.assign(ohX.mul(sign(pos.x.sub(obstacleCentre.x))).add(obstacleCentre.x))
        vel.x.assign(vel.x.mul(float(-1).mul(wallBounciness)))
      }).Else(() => {
        pos.y.assign(ohY.mul(sign(pos.y.sub(obstacleCentre.y))).add(obstacleCentre.y))
        vel.y.assign(vel.y.mul(float(-1).mul(wallBounciness)))
      })
    })

    positions.element(i).assign(pos)
    velocities.element(i).assign(vel)
  })
}
