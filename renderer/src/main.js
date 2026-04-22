import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js'
import GUI from 'lil-gui'

// ─── Config ──────────────────────────────────────────────────────────────────

const WS_URL             = `ws://${location.hostname}:9002`
const CLASSIFIER_WS_URL  = `ws://${location.hostname}:9003`

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

// ─── WebGL renderer ───────────────────────────────────────────────────────────

const renderer = new THREE.WebGLRenderer({ antialias: true })
renderer.setPixelRatio(Math.min(devicePixelRatio, 2))
renderer.setSize(innerWidth, innerHeight)
document.body.appendChild(renderer.domElement)

// ─── CSS2D renderer (used for text labels) ────────────────────────────────────

const labelRenderer = new CSS2DRenderer()
labelRenderer.setSize(innerWidth, innerHeight)
labelRenderer.domElement.style.position = 'absolute'
labelRenderer.domElement.style.top      = '0px'
labelRenderer.domElement.style.pointerEvents = 'none'
document.body.appendChild(labelRenderer.domElement)

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

// DynamicDrawUsage hints to WebGL that these buffers will be updated often.
posAttr.setUsage(THREE.DynamicDrawUsage)
colAttr.setUsage(THREE.DynamicDrawUsage)

geometry.setAttribute('position', posAttr)
geometry.setAttribute('color',    colAttr)
geometry.setDrawRange(0, 0)

const params = {
  pointSize:    0.08,
  colorMode:    'cluster',   // 'cluster' | 'height' | 'intensity'
  maxFrames:    1,
  showBoxes:    true,
  showLabels:   true,
  rotX:         0,           // degrees — pitch (rotation around X axis)
  rotY:         0,           // degrees — roll  (rotation around Y axis)
  rotZ:         0,           // degrees — yaw   (rotation around Z axis)
  groundLevel:  -10,         // metres — points below this (post-rotation) are hidden
}

const cloudMaterial = new THREE.PointsMaterial({
  size: params.pointSize,
  vertexColors: true,
  sizeAttenuation: true,
})
scene.add(new THREE.Points(geometry, cloudMaterial))

// Group that holds all classification boxes.  Rotating this group matches the
// per-point rotation applied in buildGeometry() so boxes stay aligned with the
// point cloud without needing to transform each centroid individually.
const boxGroup = new THREE.Group()
scene.add(boxGroup)

// ─── GUI ─────────────────────────────────────────────────────────────────────

const gui = new GUI({ title: 'LiDAR Viewer', width: 240 })

gui.add(params, 'pointSize', 0.01, 0.5, 0.005)
   .name('Point size')
   .onChange(v => { cloudMaterial.size = v })

gui.add(params, 'colorMode', ['cluster', 'height', 'intensity'])
   .name('Color mode')

gui.add(params, 'maxFrames', 1, 30, 1)
   .name('Frames to keep')

gui.add(params, 'showBoxes')
   .name('Bounding boxes')
   .onChange(v => { clusterObjects.forEach(o => { o.box.visible = v }) })

gui.add(params, 'showLabels')
   .name('Labels')
   .onChange(v => { clusterObjects.forEach(o => { o.labelObj.visible = v }) })

gui.add(params, 'groundLevel', -10, 10, 0.05)
   .name('Ground level (m)')

const rotFolder = gui.addFolder('Rotation (deg)')
rotFolder.add(params, 'rotX', -180, 180, 0.5).name('X  (pitch)')
rotFolder.add(params, 'rotY', -180, 180, 0.5).name('Y  (roll)')
rotFolder.add(params, 'rotZ', -180, 180, 0.5).name('Z  (yaw)')

// ─── Frame queue ─────────────────────────────────────────────────────────────

const frames = []

// ─── Filtered cluster IDs (pedestrians suppressed by the classifier) ──────────

const filteredClusterIds = new Set()

// ─── Classification overlay ──────────────────────────────────────────────────
// Maps cluster_id → { box: THREE.LineSegments, labelObj: CSS2DObject }

const clusterObjects = new Map()

// Wireframe box material per label type
const LABEL_COLORS = {
  car:     0x00bfff,
  truck:   0xff8c00,
  ground:  0x888888,
  unknown: 0xffffff,
}

function labelColor(label) {
  return LABEL_COLORS[label] ?? LABEL_COLORS.unknown
}

// Create or update the 3D box + CSS label for a single classification entry.
function upsertCluster(c) {
  const [dx, dy, dz] = c.bbox
  const [cx, cy, cz] = c.centroid

  if (clusterObjects.has(c.cluster_id)) {
    const { box, labelObj, labelDiv } = clusterObjects.get(c.cluster_id)

    // Reuse existing objects — update geometry and position in-place.
    box.geometry.dispose()
    box.geometry = new THREE.EdgesGeometry(new THREE.BoxGeometry(dx, dy, dz))
    box.material.color.setHex(labelColor(c.label))
    box.position.set(cx, cy, cz)
    box.visible = params.showBoxes

    labelObj.position.set(0, 0, dz / 2 + 0.4)
    labelDiv.textContent = `${c.label} ${(c.confidence * 100).toFixed(0)}%`
    labelDiv.className   = `cluster-label label-${c.label}`
    labelObj.visible     = params.showLabels

  } else {
    // Build box wireframe
    const boxGeom = new THREE.EdgesGeometry(new THREE.BoxGeometry(dx, dy, dz))
    const boxMat  = new THREE.LineBasicMaterial({ color: labelColor(c.label) })
    const box     = new THREE.LineSegments(boxGeom, boxMat)
    box.position.set(cx, cy, cz)
    box.visible = params.showBoxes

    // Build CSS2D label — child of the box so it moves with it
    const div = document.createElement('div')
    div.textContent = `${c.label} ${(c.confidence * 100).toFixed(0)}%`
    div.className   = `cluster-label label-${c.label}`

    const labelObj = new CSS2DObject(div)
    // Offset upward by half the box height + a small margin
    labelObj.position.set(0, 0, dz / 2 + 0.4)
    labelObj.visible = params.showLabels
    box.add(labelObj)

    boxGroup.add(box)
    clusterObjects.set(c.cluster_id, { box, labelObj, labelDiv: div })
  }
}

function applyClassifications(classifications, filteredIds) {
  // Refresh the pedestrian (filtered) cluster ID set so buildGeometry() can
  // suppress those points from the rendered cloud.
  filteredClusterIds.clear()
  for (const id of filteredIds) filteredClusterIds.add(id)

  const activeIds = new Set(classifications.map(c => c.cluster_id))

  // Remove boxes + labels for clusters no longer present.
  // labelDiv.remove() is required: CSS2DRenderer creates DOM elements and does
  // not remove them when the parent Object3D is removed from the scene graph.
  for (const [id, { box, labelDiv }] of clusterObjects) {
    if (!activeIds.has(id)) {
      box.geometry.dispose()
      box.material.dispose()
      boxGroup.remove(box)
      labelDiv.remove()
      clusterObjects.delete(id)
    }
  }

  // Create or update each active cluster
  for (const c of classifications) {
    upsertCluster(c)
  }
}

// ─── Worker setup ────────────────────────────────────────────────────────────

const worker = new Worker(new URL('./worker.js', import.meta.url), { type: 'module' })
worker.postMessage({ type: 'init', wsUrl: WS_URL, classifierUrl: CLASSIFIER_WS_URL })

const connLabel       = document.getElementById('conn-label')
const classifierLabel = document.getElementById('classifier-label')
const infoLabel       = document.getElementById('info-label')
const fpsDiv          = document.getElementById('fps-counter')

let lastFrameId = -1

worker.onmessage = ({ data: msg }) => {
  switch (msg.type) {
    case 'status':
      if (msg.connected) {
        connLabel.textContent = '⬤ Lidar'
        connLabel.className   = 'connected'
      } else {
        connLabel.textContent = '⬤ Lidar'
        connLabel.className   = 'disconnected'
      }
      break

    case 'classifierStatus':
      if (msg.connected) {
        classifierLabel.textContent = '⬤ Classifier'
        classifierLabel.className   = 'connected'
      } else {
        classifierLabel.textContent = '⬤ Classifier'
        classifierLabel.className   = 'disconnected'
      }
      break

    case 'cloud':
      frames.push({ pos: msg.pos, clusterIds: msg.clusterIds,
                    intensities: msg.intensities, count: msg.count })
      while (frames.length > params.maxFrames) frames.shift()
      lastFrameId = msg.id
      break

    case 'classifications':
      applyClassifications(
        msg.data.classifications ?? [],
        msg.data.filtered_cluster_ids ?? [],
      )
      break
  }
}

// ─── Color helpers ───────────────────────────────────────────────────────────

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

function clusterColor(id, out) {
  if (id < 0) { out[0] = out[1] = out[2] = 0.35; return }
  hslToRgb(((id * 137.508) % 360) / 360, 0.85, 0.55, out)
}

// ─── Rotation helpers (pre-allocated to avoid per-frame heap pressure) ────────

const _euler  = new THREE.Euler()
const _rotMat = new THREE.Matrix4()
const _vec    = new THREE.Vector3()

// ─── Geometry rebuild (called once per animation frame) ──────────────────────

function buildGeometry() {
  if (frames.length === 0) { geometry.setDrawRange(0, 0); return }

  // Build rotation matrix from current GUI params (once per frame).
  _euler.set(
    THREE.MathUtils.degToRad(params.rotX),
    THREE.MathUtils.degToRad(params.rotY),
    THREE.MathUtils.degToRad(params.rotZ),
    'XYZ',
  )
  _rotMat.makeRotationFromEuler(_euler)

  // Sync box group rotation so boxes stay aligned with the rotated point cloud.
  // Box centroids are stored in sensor-frame coordinates; rotating the group
  // transforms them to world frame without touching each box individually.
  boxGroup.rotation.copy(_euler)

  let zMin = Infinity, zMax = -Infinity
  if (params.colorMode === 'height') {
    for (const f of frames) {
      for (let i = 0; i < f.count; i++) {
        if (filteredClusterIds.has(f.clusterIds[i])) continue
        _vec.set(f.pos[3 * i], f.pos[3 * i + 1], f.pos[3 * i + 2])
        _vec.applyMatrix4(_rotMat)
        if (_vec.z < params.groundLevel) continue
        if (_vec.z < zMin) zMin = _vec.z
        if (_vec.z > zMax) zMax = _vec.z
      }
    }
    if (zMin === zMax) zMax = zMin + 1
  }

  let offset = 0
  done:
  for (const f of frames) {
    for (let i = 0; i < f.count; i++) {
      if (offset >= MAX_POINTS) break done

      const cid = f.clusterIds[i]

      // Skip points belonging to filtered (pedestrian) clusters.
      if (filteredClusterIds.has(cid)) continue

      _vec.set(f.pos[3 * i], f.pos[3 * i + 1], f.pos[3 * i + 2])
      _vec.applyMatrix4(_rotMat)

      // Skip points below the ground level threshold (post-rotation world space).
      if (_vec.z < params.groundLevel) continue

      const b = offset * 3
      positions[b]     = _vec.x
      positions[b + 1] = _vec.y
      positions[b + 2] = _vec.z

      if (params.colorMode === 'cluster') {
        clusterColor(cid, _rgb)
      } else if (params.colorMode === 'height') {
        hslToRgb((1 - (_vec.z - zMin) / (zMax - zMin)) * 0.67, 1.0, 0.5, _rgb)
      } else {
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
  labelRenderer.render(scene, camera)
}

window.addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight
  camera.updateProjectionMatrix()
  renderer.setSize(innerWidth, innerHeight)
  labelRenderer.setSize(innerWidth, innerHeight)
})

animate()
