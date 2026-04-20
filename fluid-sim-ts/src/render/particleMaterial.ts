// @ts-nocheck
import {
  Fn, uniform, vertexIndex,
  vec2, vec3, vec4, float, uint, int,
  length, clamp, dot, mix, smoothstep, min,
  If, uv, texture,
} from 'three/tsl'
import { PointsNodeMaterial } from 'three/webgpu'
import { DataTexture, RGBAFormat, UnsignedByteType, LinearFilter } from 'three'
import type { BufferManager } from '../sim/BufferManager.js'

// Velocity color gradient texture (cool → warm)
function buildGradientTexture(width = 256): DataTexture {
  const data = new Uint8Array(width * 4)
  for (let i = 0; i < width; i++) {
    const t = i / (width - 1)
    let r = 0, g = 0, b = 0
    if (t < 0.25) {
      r = 0; g = t * 4; b = 1
    } else if (t < 0.5) {
      r = 0; g = 1; b = 1 - (t - 0.25) * 4
    } else if (t < 0.75) {
      r = (t - 0.5) * 4; g = 1; b = 0
    } else {
      r = 1; g = 1 - (t - 0.75) * 4; b = 0
    }
    data[i * 4 + 0] = Math.round(r * 255)
    data[i * 4 + 1] = Math.round(g * 255)
    data[i * 4 + 2] = Math.round(b * 255)
    data[i * 4 + 3] = 255
  }
  const tex = new DataTexture(data, width, 1, RGBAFormat, UnsignedByteType)
  tex.magFilter = LinearFilter
  tex.minFilter = LinearFilter
  tex.needsUpdate = true
  return tex
}

export function buildParticleMaterial(buf: BufferManager, maxSpeed = 5, pointSize = 6) {
  const { positions, velocities, states } = buf

  const maxSpeedU = uniform(maxSpeed, 'float')
  const pointSizeU = uniform(pointSize, 'float')
  const gradientTexture = buildGradientTexture()

  const material = new PointsNodeMaterial()
  material.transparent = true
  material.depthWrite = false
  material.sizeAttenuation = false

  // Read particle world position from storage buffer
  material.positionNode = Fn(() => {
    const p = positions.element(vertexIndex)
    return vec3(p.x, p.y, float(0))
  })()

  ;(material as any).pointWidth = pointSizeU

  // Color: velocity gradient + state tinting + circle alpha
  material.colorNode = Fn(() => {
    const i = vertexIndex
    const vel = velocities.element(i)
    const speedT = clamp(length(vel).div(maxSpeedU), float(0), float(1))

    // Sample velocity gradient (1D strip texture)
    const gradColor = texture(gradientTexture, vec2(speedT, float(0.5))).rgb

    // State-based tint: 0=Calm(none), 1=Scared(red), 2=Huddle(blue)
    const stateIdx = min(states.element(i), uint(2))
    const tintColor = vec3(float(1), float(1), float(1)).toVar()
    const tintAlpha = float(0).toVar()

    If(stateIdx.equal(uint(1)), () => {
      tintColor.assign(vec3(float(1), float(0.1), float(0.1)))
      tintAlpha.assign(float(0.85))
    }).ElseIf(stateIdx.equal(uint(2)), () => {
      tintColor.assign(vec3(float(0.2), float(0.4), float(1)))
      tintAlpha.assign(float(0.85))
    })

    const finalRGB = mix(gradColor, tintColor, tintAlpha)

    // Anti-aliased circle from UV (0→1 over point sprite)
    const centreOffset = uv().sub(vec2(float(0.5), float(0.5))).mul(float(2))
    const sqrDst = dot(centreOffset, centreOffset)
    const alpha = smoothstep(float(1.1), float(0.8), sqrDst)

    return vec4(finalRGB, alpha)
  })()

  return { material, maxSpeedU, pointSizeU }
}
