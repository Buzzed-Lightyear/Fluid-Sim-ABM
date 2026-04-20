// @ts-nocheck
import { Fn, If, Return, uint, int, instanceIndex } from 'three/tsl'
import { uniform } from 'three/tsl'
import type { WebGPURenderer } from 'three/webgpu'
import type { BufferManager } from './BufferManager.js'
import { nextPowerOfTwo } from '../types/BufferLayouts.js'

export class BitonicSort {
  private readonly numEntriesU = uniform(0, 'uint')
  private readonly groupWidthU = uniform(0, 'uint')
  private readonly groupHeightU = uniform(0, 'uint')
  private readonly stepIndexU = uniform(0, 'uint')

  private readonly sortFn: ReturnType<typeof Fn>
  private readonly offsetsFn: ReturnType<typeof Fn>
  // ComputeNodes created once; uniforms updated between dispatches
  private sortCompute: any
  private offsetsCompute: any

  constructor(buf: BufferManager, N: number) {
    const { spatialIdx, spatialHash_, spatialKey, spatialOffsets } = buf
    const numEntriesU = this.numEntriesU
    const groupWidthU = this.groupWidthU
    const groupHeightU = this.groupHeightU
    const stepIndexU = this.stepIndexU

    // Port of BitonicMergeSort.compute Sort kernel
    this.sortFn = (Fn as any)((): void => {
      const i = instanceIndex

      // Use uint(1) to avoid uint−float WGSL type error
      const hIndex = i.bitAnd(groupWidthU.sub(uint(1)))
      const indexLeft = hIndex.add(groupHeightU.add(uint(1)).mul(i.div(groupWidthU))).toVar()

      const rightStepSize = uint(0).toVar()
      If(stepIndexU.equal(uint(0)), () => {
        rightStepSize.assign(groupHeightU.sub(hIndex.mul(uint(2))))
      }).Else(() => {
        rightStepSize.assign(groupHeightU.add(uint(1)).div(uint(2)))
      })

      const indexRight = indexLeft.add(rightStepSize)
      If(indexRight.greaterThanEqual(numEntriesU), () => Return())

      const keyLeft  = spatialKey.element(indexLeft).toVar()
      const keyRight = spatialKey.element(indexRight).toVar()

      If(keyLeft.greaterThan(keyRight), () => {
        const tmpIdx  = spatialIdx.element(indexLeft).toVar()
        const tmpHash = spatialHash_.element(indexLeft).toVar()

        spatialIdx.element(indexLeft).assign(spatialIdx.element(indexRight))
        spatialHash_.element(indexLeft).assign(spatialHash_.element(indexRight))
        spatialKey.element(indexLeft).assign(keyRight)

        spatialIdx.element(indexRight).assign(tmpIdx)
        spatialHash_.element(indexRight).assign(tmpHash)
        spatialKey.element(indexRight).assign(keyLeft)
      })
    })

    // Port of BitonicMergeSort.compute CalculateOffsets kernel
    this.offsetsFn = (Fn as any)((): void => {
      const i = instanceIndex
      If(i.greaterThanEqual(numEntriesU), () => Return())

      const key = spatialKey.element(i).toVar()
      const keyPrev = uint(0).toVar()
      If(i.equal(uint(0)), () => {
        keyPrev.assign(numEntriesU)
      }).Else(() => {
        keyPrev.assign(spatialKey.element(i.sub(uint(1))))
      })

      If(key.notEqual(keyPrev), () => {
        spatialOffsets.element(key).assign(i)
      })
    })

    // Create ComputeNodes once — reused every dispatch, uniforms updated between passes
    const nextPow2 = nextPowerOfTwo(N)
    this.sortCompute    = (this.sortFn    as any).compute(nextPow2 / 2)
    this.offsetsCompute = (this.offsetsFn as any).compute(N)
  }

  async sortAndCalculateOffsets(renderer: WebGPURenderer, N: number): Promise<void> {
    const nextPow2 = nextPowerOfTwo(N)
    const numStages = Math.round(Math.log2(nextPow2))
    this.numEntriesU.value = N

    for (let stage = 0; stage < numStages; stage++) {
      for (let step = 0; step <= stage; step++) {
        const groupWidth = 1 << (stage - step)
        const groupHeight = 2 * groupWidth - 1
        this.groupWidthU.value = groupWidth
        this.groupHeightU.value = groupHeight
        this.stepIndexU.value = step
        // Reuse the same ComputeNode — uniform values are uploaded before each dispatch
        await renderer.computeAsync(this.sortCompute)
      }
    }

    await renderer.computeAsync(this.offsetsCompute)
  }
}
