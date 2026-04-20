// @ts-nocheck
import { storage } from 'three/tsl'
import { StorageBufferAttribute } from 'three/webgpu'

export class BufferManager {
  readonly N: number

  // CPU-side typed arrays (mirrors of GPU buffers)
  readonly positionsCPU: Float32Array
  readonly velocitiesCPU: Float32Array
  readonly predictedPosCPU: Float32Array
  readonly densitiesCPU: Float32Array
  readonly collisionRadiiCPU: Float32Array
  readonly statesCPU: Uint32Array
  readonly spatialIdxCPU: Uint32Array
  readonly spatialHashCPU: Uint32Array
  readonly spatialKeyCPU: Uint32Array
  readonly spatialOffsetsCPU: Uint32Array

  // GPU-side StorageBufferAttribute instances
  readonly positionsAttr: StorageBufferAttribute
  readonly velocitiesAttr: StorageBufferAttribute
  readonly predictedPosAttr: StorageBufferAttribute
  readonly densitiesAttr: StorageBufferAttribute
  readonly collisionRadiiAttr: StorageBufferAttribute
  readonly statesAttr: StorageBufferAttribute
  readonly spatialIdxAttr: StorageBufferAttribute
  readonly spatialHashAttr: StorageBufferAttribute
  readonly spatialKeyAttr: StorageBufferAttribute
  readonly spatialOffsetsAttr: StorageBufferAttribute

  // TSL storage nodes (use in kernels and materials)
  readonly positions: ReturnType<typeof storage>
  readonly velocities: ReturnType<typeof storage>
  readonly predictedPos: ReturnType<typeof storage>
  readonly densities: ReturnType<typeof storage>
  readonly collisionRadii: ReturnType<typeof storage>
  readonly states: ReturnType<typeof storage>
  readonly spatialIdx: ReturnType<typeof storage>
  readonly spatialHash_: ReturnType<typeof storage>
  readonly spatialKey: ReturnType<typeof storage>
  readonly spatialOffsets: ReturnType<typeof storage>
  // State params: 3 × vec4 = [Calm, Scared, Huddle], each (ideal, cohMult, speedMult, 0)
  readonly stateParamsCPU: Float32Array
  readonly stateParamsAttr: StorageBufferAttribute
  readonly stateParams: ReturnType<typeof storage>

  constructor(N: number) {
    this.N = N

    // Allocate CPU arrays
    this.positionsCPU = new Float32Array(N * 2)
    this.velocitiesCPU = new Float32Array(N * 2)
    this.predictedPosCPU = new Float32Array(N * 2)
    this.densitiesCPU = new Float32Array(N * 2)
    this.collisionRadiiCPU = new Float32Array(N)
    this.statesCPU = new Uint32Array(N)
    this.spatialIdxCPU = new Uint32Array(N)
    this.spatialHashCPU = new Uint32Array(N)
    this.spatialKeyCPU = new Uint32Array(N)
    this.spatialOffsetsCPU = new Uint32Array(N)

    // Create GPU StorageBufferAttributes backed by CPU arrays
    this.positionsAttr = new StorageBufferAttribute(this.positionsCPU, 2)
    this.velocitiesAttr = new StorageBufferAttribute(this.velocitiesCPU, 2)
    this.predictedPosAttr = new StorageBufferAttribute(this.predictedPosCPU, 2)
    this.densitiesAttr = new StorageBufferAttribute(this.densitiesCPU, 2)
    this.collisionRadiiAttr = new StorageBufferAttribute(this.collisionRadiiCPU, 1)
    this.statesAttr = new StorageBufferAttribute(this.statesCPU, 1)
    this.spatialIdxAttr = new StorageBufferAttribute(this.spatialIdxCPU, 1)
    this.spatialHashAttr = new StorageBufferAttribute(this.spatialHashCPU, 1)
    this.spatialKeyAttr = new StorageBufferAttribute(this.spatialKeyCPU, 1)
    this.spatialOffsetsAttr = new StorageBufferAttribute(this.spatialOffsetsCPU, 1)

    // Create TSL storage nodes
    this.positions = storage(this.positionsAttr, 'vec2', N)
    this.velocities = storage(this.velocitiesAttr, 'vec2', N)
    this.predictedPos = storage(this.predictedPosAttr, 'vec2', N)
    this.densities = storage(this.densitiesAttr, 'vec2', N)
    this.collisionRadii = storage(this.collisionRadiiAttr, 'float', N)
    this.states = storage(this.statesAttr, 'uint', N)
    this.spatialIdx = storage(this.spatialIdxAttr, 'uint', N)
    this.spatialHash_ = storage(this.spatialHashAttr, 'uint', N)
    this.spatialKey = storage(this.spatialKeyAttr, 'uint', N)
    this.spatialOffsets = storage(this.spatialOffsetsAttr, 'uint', N)

    // State params: 3 entries × 4 floats (ideal, cohMult, speedMult, padding)
    this.stateParamsCPU = new Float32Array(12)
    this.stateParamsAttr = new StorageBufferAttribute(this.stateParamsCPU, 4)
    this.stateParams = storage(this.stateParamsAttr, 'vec4', 3)
  }

  updateStateParams(idealNeighborCount: number): void {
    const d = this.stateParamsCPU
    // Calm
    d[0] = idealNeighborCount; d[1] = 1.0; d[2] = 1.0; d[3] = 0
    // Scared
    d[4] = 0; d[5] = 0.0; d[6] = 1.5; d[7] = 0
    // Huddle
    d[8] = 20; d[9] = 3.0; d[10] = 1.0; d[11] = 0
    this.stateParamsAttr.needsUpdate = true
  }

  markStatesForUpload(): void {
    this.statesAttr.needsUpdate = true
  }

  markPositionsForUpload(): void {
    this.positionsAttr.needsUpdate = true
    this.velocitiesAttr.needsUpdate = true
    this.collisionRadiiAttr.needsUpdate = true
  }
}
