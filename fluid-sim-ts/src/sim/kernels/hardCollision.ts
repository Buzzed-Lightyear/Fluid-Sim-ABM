// @ts-nocheck
import {
  Fn, If, Return, Break, Continue, Loop, instanceIndex,
  vec2, float, sqrt, dot, uint,
} from 'three/tsl'
import { getCellX, getCellY, hashCell2D, keyFromHash } from '../spatialHash.js'
import type { BufferManager } from '../BufferManager.js'
import type { SimUniforms } from '../SimUniforms.js'

export function buildHardCollisionKernel(buf: BufferManager, u: SimUniforms) {
  const { predictedPos, velocities, collisionRadii,
    spatialIdx, spatialHash_, spatialKey, spatialOffsets } = buf
  const { numParticles, sensorRadius, collisionStiffness, deltaTime } = u

  const checkCell = (
    colForce: any, originCellX: any, originCellY: any,
    posI: any, myRadius: any, particleI: any, ox: number, oy: number,
  ) => {
    const cellX = originCellX.add(ox)
    const cellY = originCellY.add(oy)
    const hash = hashCell2D(cellX, cellY).toVar()
    const key = keyFromHash(hash, numParticles).toVar()

    // Loop from spatialOffsets[key] to numParticles; auto-increments each iteration.
    // Break when the key changes (sorted array property), Continue to skip hash/self.
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
        const minDst = myRadius.add(collisionRadii.element(pIdx))
        If(sqrDst.greaterThanEqual(minDst.mul(minDst)), () => Continue())

        const dst = sqrt(sqrDst)
        If(dst.lessThanEqual(float(0.0001)), () => Continue())

        const kick = minDst.sub(dst).div(minDst)
        colForce.subAssign(offset.div(dst).mul(kick.mul(kick).mul(collisionStiffness)))
      }
    )
  }

  return (Fn as any)((): void => {
    const i = instanceIndex
    If(i.greaterThanEqual(numParticles), () => Return())

    const pos = predictedPos.element(i).toVar()
    const myRadius = collisionRadii.element(i).toVar()
    const originCellX = getCellX(pos.x, sensorRadius).toVar()
    const originCellY = getCellY(pos.y, sensorRadius).toVar()
    const colForce = vec2(float(0), float(0)).toVar()

    checkCell(colForce, originCellX, originCellY, pos, myRadius, i, -1, -1)
    checkCell(colForce, originCellX, originCellY, pos, myRadius, i,  0, -1)
    checkCell(colForce, originCellX, originCellY, pos, myRadius, i,  1, -1)
    checkCell(colForce, originCellX, originCellY, pos, myRadius, i, -1,  0)
    checkCell(colForce, originCellX, originCellY, pos, myRadius, i,  0,  0)
    checkCell(colForce, originCellX, originCellY, pos, myRadius, i,  1,  0)
    checkCell(colForce, originCellX, originCellY, pos, myRadius, i, -1,  1)
    checkCell(colForce, originCellX, originCellY, pos, myRadius, i,  0,  1)
    checkCell(colForce, originCellX, originCellY, pos, myRadius, i,  1,  1)

    velocities.element(i).addAssign(colForce.mul(deltaTime))
  })
}
