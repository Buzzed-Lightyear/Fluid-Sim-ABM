// @ts-nocheck
import { Fn, floor, int, uint } from 'three/tsl'

const HASH_K1 = 15823
const HASH_K2 = 9737333

// --- GPU (TSL) ---
// Returns int2 cell coordinate for a 2D position.
export const getCellX = Fn(([posX, radius]: [any, any]) =>
  int(floor(posX.div(radius)))
)
export const getCellY = Fn(([posY, radius]: [any, any]) =>
  int(floor(posY.div(radius)))
)

// Hash int2 cell to a uint. Replicates HLSL: cell=(uint2)cell; return cell.x*K1 + cell.y*K2
export const hashCell2D = Fn(([cellX, cellY]: [any, any]) => {
  const ucx = uint(cellX)  // wrapping cast (negative int → large uint, matching HLSL)
  const ucy = uint(cellY)
  return ucx.mul(HASH_K1).add(ucy.mul(HASH_K2))  // uint wrapping arithmetic
})

// Reduce hash to a table key.
export const keyFromHash = Fn(([hash, tableSize]: [any, any]) =>
  hash.mod(tableSize)
)

// --- CPU equivalents (for tests and CPU oracle compatibility) ---
export function getCellCPU(v: number, radius: number): number {
  return Math.floor(v / radius) | 0  // signed int
}

export function hashCell2DCPU(cx: number, cy: number): number {
  // Replicate HLSL (uint)cell wrapping: JS bitwise ops work on int32 range
  const ucx = cx >>> 0   // zero-fill right shift → uint32
  const ucy = cy >>> 0
  return ((Math.imul(ucx, HASH_K1) + Math.imul(ucy, HASH_K2)) >>> 0)
}

export function keyFromHashCPU(hash: number, tableSize: number): number {
  return (hash >>> 0) % tableSize
}
