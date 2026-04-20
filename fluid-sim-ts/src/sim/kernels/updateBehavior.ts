// @ts-nocheck
import {
  Fn, If, Return, Break, Continue, Loop, instanceIndex,
  vec2, float, int, uint, sqrt, dot, length, normalize, min,
} from 'three/tsl'
import { getCellX, getCellY, hashCell2D, keyFromHash } from '../spatialHash.js'
import type { BufferManager } from '../BufferManager.js'
import type { SimUniforms } from '../SimUniforms.js'

export function buildUpdateBehaviorKernel(buf: BufferManager, u: SimUniforms) {
  const { predictedPos, velocities, densities, states, stateParams,
    spatialIdx, spatialHash_, spatialKey, spatialOffsets } = buf
  const { numParticles, sensorRadius, pressureMultiplier, viscosity,
    drag, cohesionStrength, maxForce, maxSpeed, deltaTime } = u

  const checkCell = (
    pressureForce: any, viscosityForce: any, avgPos: any, neighborCount: any,
    originCellX: any, originCellY: any, posI: any, myVel: any, particleI: any,
    ox: number, oy: number,
  ) => {
    const cellX = originCellX.add(ox)
    const cellY = originCellY.add(oy)
    const hash = hashCell2D(cellX, cellY).toVar()
    const key = keyFromHash(hash, numParticles).toVar()
    Loop(
      { type: 'uint', start: spatialOffsets.element(key), end: numParticles },
      ({ i: idx }) => {
        const pIdx = spatialIdx.element(idx).toVar()
        const h = spatialHash_.element(idx).toVar()
        const k = spatialKey.element(idx).toVar()
        If(k.notEqual(key), () => Break())
        If(h.notEqual(hash), () => Continue())
        If(pIdx.equal(particleI), () => Continue())

        const offset = predictedPos.element(pIdx).sub(posI)
        const sqrDst = dot(offset, offset)
        If(sqrDst.greaterThanEqual(sensorRadius.mul(sensorRadius)), () => Continue())

        const dst = sqrt(sqrDst)
        If(dst.lessThanEqual(float(0.0001)), () => Continue())

        neighborCount.addAssign(int(1))
        avgPos.addAssign(predictedPos.element(pIdx))
        pressureForce.subAssign(offset.div(dst).mul(float(1).div(dst)))
        viscosityForce.addAssign(velocities.element(pIdx).sub(myVel).div(dst))
      }
    )
  }

  return (Fn as any)((): void => {
    const i = instanceIndex
    If(i.greaterThanEqual(numParticles), () => Return())

    const pos = predictedPos.element(i).toVar()
    const myVel = velocities.element(i).toVar()
    const originCellX = getCellX(pos.x, sensorRadius).toVar()
    const originCellY = getCellY(pos.y, sensorRadius).toVar()

    const stateIdx = min(states.element(i), uint(2)).toVar()
    const sp = stateParams.element(stateIdx)
    const ideal = sp.x.toVar()
    const cohMult = sp.y.toVar()
    const speedMult = sp.z.toVar()

    const pressureForce = vec2(float(0), float(0)).toVar()
    const viscosityForce = vec2(float(0), float(0)).toVar()
    const avgPos = vec2(float(0), float(0)).toVar()
    const neighborCount = int(0).toVar()

    checkCell(pressureForce, viscosityForce, avgPos, neighborCount, originCellX, originCellY, pos, myVel, i, -1, -1)
    checkCell(pressureForce, viscosityForce, avgPos, neighborCount, originCellX, originCellY, pos, myVel, i,  0, -1)
    checkCell(pressureForce, viscosityForce, avgPos, neighborCount, originCellX, originCellY, pos, myVel, i,  1, -1)
    checkCell(pressureForce, viscosityForce, avgPos, neighborCount, originCellX, originCellY, pos, myVel, i, -1,  0)
    checkCell(pressureForce, viscosityForce, avgPos, neighborCount, originCellX, originCellY, pos, myVel, i,  0,  0)
    checkCell(pressureForce, viscosityForce, avgPos, neighborCount, originCellX, originCellY, pos, myVel, i,  1,  0)
    checkCell(pressureForce, viscosityForce, avgPos, neighborCount, originCellX, originCellY, pos, myVel, i, -1,  1)
    checkCell(pressureForce, viscosityForce, avgPos, neighborCount, originCellX, originCellY, pos, myVel, i,  0,  1)
    checkCell(pressureForce, viscosityForce, avgPos, neighborCount, originCellX, originCellY, pos, myVel, i,  1,  1)

    const totalForce = vec2(float(0), float(0)).toVar()

    If(neighborCount.greaterThan(int(0)), () => {
      const densityError = float(neighborCount).sub(ideal).toVar()
      If(densityError.greaterThan(float(0)), () => {
        totalForce.addAssign(pressureForce.mul(pressureMultiplier).mul(densityError))
      })
      totalForce.addAssign(viscosityForce.mul(viscosity))
      totalForce.addAssign(avgPos.div(float(neighborCount)).sub(pos).mul(cohesionStrength).mul(cohMult))
    })

    totalForce.subAssign(myVel.mul(drag))

    const fLen = length(totalForce).toVar()
    If(fLen.greaterThan(maxForce), () => {
      totalForce.assign(normalize(totalForce).mul(maxForce))
    })

    const newVel = myVel.add(totalForce.mul(deltaTime)).toVar()

    const speedCap = maxSpeed.mul(speedMult).toVar()
    const vLen = length(newVel).toVar()
    If(vLen.greaterThan(speedCap), () => {
      newVel.assign(normalize(newVel).mul(speedCap))
    })

    velocities.element(i).assign(newVel)
    densities.element(i).assign(vec2(float(neighborCount), float(0)))
  })
}
