// @ts-nocheck
import { Fn, If, Return, instanceIndex } from 'three/tsl'
import { getCellX, getCellY, hashCell2D, keyFromHash } from '../spatialHash.js'
import type { BufferManager } from '../BufferManager.js'
import type { SimUniforms } from '../SimUniforms.js'

export function buildUpdateSpatialHashKernel(buf: BufferManager, u: SimUniforms) {
  const { predictedPos, spatialIdx, spatialHash_, spatialKey, spatialOffsets } = buf
  const { numParticles, sensorRadius } = u

  return (Fn as any)((): void => {
    const i = instanceIndex
    If(i.greaterThanEqual(numParticles), () => Return())

    spatialOffsets.element(i).assign(numParticles)

    const pos = predictedPos.element(i)
    const cellX = getCellX(pos.x, sensorRadius)
    const cellY = getCellY(pos.y, sensorRadius)
    const hash = hashCell2D(cellX, cellY)
    const key = keyFromHash(hash, numParticles)

    spatialIdx.element(i).assign(i)
    spatialHash_.element(i).assign(hash)
    spatialKey.element(i).assign(key)
  })
}
