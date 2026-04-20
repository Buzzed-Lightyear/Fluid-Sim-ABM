// @ts-nocheck
import {
  Fn, If, Return, instanceIndex,
  vec2, float, sqrt, dot,
} from 'three/tsl'
import type { BufferManager } from '../BufferManager.js'
import type { SimUniforms } from '../SimUniforms.js'

// Port of SwarmSim.compute ExternalForces kernel.
export function buildExternalForcesKernel(buf: BufferManager, u: SimUniforms) {
  const { positions, velocities, predictedPos } = buf
  const { numParticles, gravity, deltaTime,
    interactionInputPoint, interactionInputStrength, interactionInputRadius } = u

  return (Fn as any)((): void => {
    const i = instanceIndex
    If(i.greaterThanEqual(numParticles), () => Return())

    const force = vec2(float(0), gravity).toVar()

    If(interactionInputStrength.notEqual(float(0)), () => {
      const offset = interactionInputPoint.sub(positions.element(i))
      const sqrDst = dot(offset, offset)
      const r = interactionInputRadius
      If(sqrDst.lessThan(r.mul(r)), () => {
        const dst = sqrt(sqrDst)
        const dir = offset.div(dst)
        const t = float(1).sub(dst.div(r))
        force.addAssign(dir.mul(interactionInputStrength.mul(t)))
      })
    })

    velocities.element(i).addAssign(force.mul(deltaTime))
    predictedPos.element(i).assign(
      positions.element(i).add(velocities.element(i).mul(float(1 / 120)))
    )
  })
}
