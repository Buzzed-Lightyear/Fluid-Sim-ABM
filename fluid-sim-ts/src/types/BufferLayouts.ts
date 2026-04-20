export const THREAD_GROUP_SIZE = 64
export const SORT_THREAD_GROUP_SIZE = 128

export function nextPowerOfTwo(n: number): number {
  let p = 1
  while (p < n) p <<= 1
  return p
}

export function numGroups(count: number, groupSize = THREAD_GROUP_SIZE): number {
  return Math.ceil(count / groupSize)
}
