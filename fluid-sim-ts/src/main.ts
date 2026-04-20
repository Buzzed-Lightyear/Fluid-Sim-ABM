// @ts-nocheck
import { WebGPURenderer } from 'three/webgpu'
import { Scene, OrthographicCamera, Color } from 'three'
import { SwarmSimulation } from './sim/SwarmSimulation.js'
import { ParticleRenderer } from './render/ParticleRenderer.js'

const NUM_PARTICLES = 4096
const BOUNDS = [17, 9] as [number, number]  // world units, matches DEFAULT_SETTINGS.boundsSize

async function checkWebGPU(): Promise<boolean> {
  if (!('gpu' in navigator)) return false
  try {
    const adapter = await (navigator as any).gpu.requestAdapter()
    return adapter !== null
  } catch {
    return false
  }
}

async function main() {
  // --- WebGPU availability check ---
  if (!(await checkWebGPU())) {
    const el = document.getElementById('webgpu-error')!
    el.style.display = 'flex'
    el.innerHTML = `
      <p><strong>WebGPU is not available in this browser.</strong></p>
      <p>Please use Chrome 113+ or Edge 113+ with WebGPU enabled.</p>
      <p>On Chrome: enable via <code>chrome://flags/#enable-unsafe-webgpu</code></p>
    `
    return
  }

  // [DIAG-1] Renderer init — log backend and surface any init errors
  console.log('[DIAG-1] WebGPURenderer init...')
  const renderer = new WebGPURenderer({ antialias: true })
  renderer.setPixelRatio(window.devicePixelRatio)
  renderer.setSize(window.innerWidth, window.innerHeight)
  // [DIAG-3] Magenta clear: if the canvas presents this color, the renderer is alive
  renderer.setClearColor(new Color(0xff00ff))
  document.body.appendChild(renderer.domElement)

  try {
    await renderer.init()
  } catch (err) {
    console.error('[DIAG-1] renderer.init() FAILED:', err)
    return
  }
  console.log('[DIAG-1] renderer.init() OK  backend:', (renderer as any).backend?.constructor?.name,
    ' isWebGPU:', (renderer as any).backend?.isWebGPUBackend)

  // --- Scene & Camera ---
  const scene = new Scene()
  const aspect = window.innerWidth / window.innerHeight
  const worldH = BOUNDS[1]
  const worldW = worldH * aspect
  const camera = new OrthographicCamera(
    -worldW / 2, worldW / 2,
     worldH / 2, -worldH / 2,
    0.1, 10,
  )
  camera.position.z = 1

  // --- Simulation ---
  const sim = new SwarmSimulation(renderer, NUM_PARTICLES, {
    boundsSize: BOUNDS,
  })

  // --- Renderer ---
  new ParticleRenderer(scene, sim.buf, NUM_PARTICLES, sim.settings.maxSpeed)

  // --- Input handling ---
  let rmb = false
  let huddleKey = false

  const toWorld = (clientX: number, clientY: number) => {
    const x = (clientX / window.innerWidth * 2 - 1) * (worldW / 2)
    const y = -(clientY / window.innerHeight * 2 - 1) * (worldH / 2)
    return { x, y }
  }

  window.addEventListener('contextmenu', e => e.preventDefault())

  window.addEventListener('mousedown', e => {
    if (e.button === 2) { rmb = true; e.preventDefault() }
  })
  window.addEventListener('mouseup', e => {
    if (e.button === 2) { rmb = false; sim.clearInteraction() }
  })
  window.addEventListener('mousemove', e => {
    const { x, y } = toWorld(e.clientX, e.clientY)
    if (rmb) {
      sim.setInteraction(x, y, -15)  // attract/repel (negative = repel)
      sim.triggerPanic(x, y, 2)
    }
  })

  window.addEventListener('keydown', e => {
    if (e.key === 'h' || e.key === 'H') {
      if (!huddleKey) { huddleKey = true; sim.setAllHuddle(true) }
    }
    if (e.key === ' ') sim.paused = !sim.paused
    if (e.key === 'r' || e.key === 'R') location.reload()
  })
  window.addEventListener('keyup', e => {
    if (e.key === 'h' || e.key === 'H') {
      huddleKey = false; sim.setAllHuddle(false)
    }
  })

  // --- Resize ---
  window.addEventListener('resize', () => {
    renderer.setSize(window.innerWidth, window.innerHeight)
    const a = window.innerWidth / window.innerHeight
    const w = worldH * a
    camera.left = -w / 2; camera.right = w / 2
    camera.updateProjectionMatrix()
  })

  // --- Render loop ---
  // [DIAG-2] Frame counter — confirm the loop is alive
  let frameCount = 0
  let lastTime = performance.now()

  async function frame() {
    const now = performance.now()
    const dt = Math.min((now - lastTime) / 1000, 0.05)  // cap at 50ms
    lastTime = now
    frameCount++

    if (frameCount === 1 || frameCount === 60 || frameCount === 300) {
      console.log(`[DIAG-2] frame ${frameCount}`)
    }

    sim.tickStateTimers()

    // Protect: compute errors must not kill the render loop.
    // If sim.update throws, we still render the last valid frame and schedule
    // the next one — this keeps the magenta/particles visible for bisection.
    try {
      await sim.update(dt)
    } catch (err) {
      if (frameCount <= 5 || frameCount % 300 === 0) {
        console.error(`[DIAG] sim.update error (frame ${frameCount}):`, err)
      }
    }

    renderer.render(scene, camera)
    requestAnimationFrame(frame)
  }

  requestAnimationFrame(frame)
}

main().catch(console.error)
