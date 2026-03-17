import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import GUI from 'lil-gui'

// ─── Config ──────────────────────────────────────────────────────────────────

// Falls back gracefully when accessed from a remote host (e.g. Docker).
const WS_URL = `ws://${location.hostname}:9002`

// Upper bound on points held in the GPU buffer at once.
// Each point costs 6 floats (3 pos + 3 color) = 24 bytes → 200 K pts ≈ 4.8 MB.
const MAX_POINTS = 200_000

// ─── Scene ───────────────────────────────────────────────────────────────────

const scene = new THREE.Scene()
scene.background = new THREE.Color(0x0a0a0a)

// Grid in the XY plane (Z-up coordinate system to match the LiDAR frame)
const grid = new THREE.GridHelper(60, 60, 0x2a2a2a, 0x1a1a1a)
grid.rotation.x = Math.PI / 2   // GridHelper is XZ by default; rotate to XY
scene.add(grid)

// Thin axes at origin for orientation reference
scene.add(new THREE.AxesHelper(2))

// ─── Camera ───────────────────────────────────────────────────────────────────

const camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.01, 500)
camera.up.set(0, 0, 1)            // Z-up to match LiDAR coordinate frame
camera.position.set(0, -18, 14)   // Isometric-ish bird's-eye starting view

const renderer = new THREE.WebGLRenderer({ antialias: true })
renderer.setPixelRatio(Math.min(devicePixelRatio, 2))
renderer.setSize(innerWidth, innerHeight)
document.body.appendChild(renderer.domElement)

const controls = new OrbitControls(camera, renderer.domElement)
controls.enableDamping = true
controls.dampingFactor = 0.06
controls.target.set(0, 0, 0)

// ─── Point cloud geometry (pre-allocated, never resized) ─────────────────────

const positions  = new Float32Array(MAX_POINTS * 3)
const colors     = new Float32Array(MAX_POINTS * 3)

const geometry  = new THREE.BufferGeometry()
const posAttr   = new THREE.BufferAttribute(positions, 3)
const colAttr   = new THREE.BufferAttribute(colors, 3)

// DynamicDrawUsage hints to WebGL that these buffers will be updated often,
// allowing the driver to place them in faster memory.
posAttr.setUsage(THREE.DynamicDrawUsage)
colAttr.setUsage(THREE.DynamicDrawUsage)

geometry.setAttribute('position', posAttr)
geometry.setAttribute('color',    colAttr)
geometry.setDrawRange(0, 0)

const params = {
  pointSize:  0.08,
  colorMode:  'cluster',   // 'cluster' | 'height' | 'intensity'
  maxFrames:  1,           // how many consecutive clouds to overlay
}

const cloudMaterial = new THREE.PointsMaterial({
  size: params.pointSize,
  vertexColors: true,
  sizeAttenuation: true,
})
scene.add(new THREE.Points(geometry, cloudMaterial))

// ─── Frame queue ─────────────────────────────────────────────────────────────
// Each entry is { pos, clusterIds, intensities, count } — typed arrays
// transferred from the worker (no copy overhead).

const frames = []

// ─── GUI ─────────────────────────────────────────────────────────────────────

const gui = new GUI({ title: 'LiDAR Viewer', width: 220 })

gui.add(params, 'pointSize', 0.01, 0.5, 0.005)
   .name('Point size')
   .onChange(v => { cloudMaterial.size = v })

gui.add(params, 'colorMode', ['cluster', 'height', 'intensity'])
   .name('Color mode')

gui.add(params, 'maxFrames', 1, 30, 1)
   .name('Frames to keep')

// ─── Worker setup ────────────────────────────────────────────────────────────

const worker = new Worker(new URL('./worker.js', import.meta.url), { type: 'module' })
worker.postMessage({ type: 'init', wsUrl: WS_URL })

const connLabel = document.getElementById('conn-label')
const infoLabel = document.getElementById('info-label')
const fpsDiv    = document.getElementById('fps-counter')

let lastFrameId = -1

worker.onmessage = ({ data: msg }) => {
  if (msg.type === 'status') {
    if (msg.connected) {
      connLabel.textContent = '⬤ Connected'
      connLabel.className   = 'connected'
    } else {
      connLabel.textContent = '⬤ Disconnected'
      connLabel.className   = 'disconnected'
    }
  } else if (msg.type === 'cloud') {
    frames.push({ pos: msg.pos, clusterIds: msg.clusterIds,
                  intensities: msg.intensities, count: msg.count })
    // Evict oldest frames beyond the configured limit
    while (frames.length > params.maxFrames) frames.shift()
    lastFrameId = msg.id
  }
}

// ─── Color helpers ───────────────────────────────────────────────────────────

// Reusable scratch array — avoids allocating [r,g,b] arrays in hot path
const _rgb = new Float32Array(3)

function hslToRgb(h, s, l, out) {
  const c = (1 - Math.abs(2 * l - 1)) * s
  const x = c * (1 - Math.abs((h * 6) % 2 - 1))
  const m = l - c / 2
  let r = 0, g = 0, b = 0
  if      (h < 1/6) { r = c; g = x }
  else if (h < 2/6) { r = x; g = c }
  else if (h < 3/6) { g = c; b = x }
  else if (h < 4/6) { g = x; b = c }
  else if (h < 5/6) { r = x; b = c }
  else              { r = c; b = x }
  out[0] = r + m; out[1] = g + m; out[2] = b + m
}

// Golden-angle hue hash — produces visually distinct colors for any cluster ID.
function clusterColor(id, out) {
  if (id < 0) { out[0] = out[1] = out[2] = 0.35; return }
  hslToRgb(((id * 137.508) % 360) / 360, 0.85, 0.55, out)
}

// ─── Geometry rebuild (called once per animation frame) ──────────────────────

function buildGeometry() {
  if (frames.length === 0) { geometry.setDrawRange(0, 0); return }

  // Compute Z range for height colormap in a single pass over active frames
  let zMin = Infinity, zMax = -Infinity
  if (params.colorMode === 'height') {
    for (const f of frames) {
      for (let i = 0; i < f.count; i++) {
        const z = f.pos[3 * i + 2]
        if (z < zMin) zMin = z
        if (z > zMax) zMax = z
      }
    }
    if (zMin === zMax) zMax = zMin + 1
  }

  let offset = 0
  done:
  for (const f of frames) {
    for (let i = 0; i < f.count; i++) {
      if (offset >= MAX_POINTS) break done

      const b = offset * 3
      positions[b]     = f.pos[3 * i]
      positions[b + 1] = f.pos[3 * i + 1]
      positions[b + 2] = f.pos[3 * i + 2]

      if (params.colorMode === 'cluster') {
        clusterColor(f.clusterIds[i], _rgb)
      } else if (params.colorMode === 'height') {
        // Blue (cold) → red (warm) gradient across observed Z range
        hslToRgb((1 - (f.pos[3 * i + 2] - zMin) / (zMax - zMin)) * 0.67, 1.0, 0.5, _rgb)
      } else {
        // Intensity: white when intensity = 1, black when 0
        const v = f.intensities[i]
        _rgb[0] = _rgb[1] = _rgb[2] = v
      }
      colors[b] = _rgb[0]; colors[b + 1] = _rgb[1]; colors[b + 2] = _rgb[2]

      offset++
    }
  }

  posAttr.needsUpdate = true
  colAttr.needsUpdate = true
  geometry.setDrawRange(0, offset)
  return offset
}

// ─── FPS counter ─────────────────────────────────────────────────────────────

let fpsCount = 0, fpsTimer = performance.now(), fpsDisplay = 0

function tickFps() {
  fpsCount++
  const now = performance.now()
  if (now - fpsTimer >= 500) {
    fpsDisplay = Math.round(fpsCount * 1000 / (now - fpsTimer))
    fpsCount = 0
    fpsTimer = now
  }
}

// ─── Render loop ─────────────────────────────────────────────────────────────

function animate() {
  requestAnimationFrame(animate)

  const drawn = buildGeometry() ?? 0

  tickFps()
  fpsDiv.textContent    = `${fpsDisplay} fps`
  infoLabel.textContent = `${drawn.toLocaleString()} pts  ·  frame ${lastFrameId}`

  controls.update()
  renderer.render(scene, camera)
}

window.addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight
  camera.updateProjectionMatrix()
  renderer.setSize(innerWidth, innerHeight)
})

animate()
