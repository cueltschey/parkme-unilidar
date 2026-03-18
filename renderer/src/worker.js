// WebSocket worker — runs entirely off the main thread.
// Manages two connections:
//   1. lidar (port 9002) — point cloud frames
//   2. classifier (port 9003) — cluster classification results

let ws = null
let classifierWs = null
let wsUrl = null
let classifierUrl = null

self.onmessage = (e) => {
  if (e.data.type === 'init') {
    wsUrl           = e.data.wsUrl
    classifierUrl   = e.data.classifierUrl
    connect()
    connectClassifier()
  }
}

// ─── Lidar point-cloud connection ────────────────────────────────────────────

function connect() {
  try { ws = new WebSocket(wsUrl) }
  catch (_) { retry(); return }

  ws.onopen  = () => self.postMessage({ type: 'status', connected: true })
  ws.onerror = () => ws.close()
  ws.onclose = () => {
    self.postMessage({ type: 'status', connected: false })
    retry()
  }

  ws.onmessage = (e) => {
    let cloud
    try { cloud = JSON.parse(e.data) } catch (_) { return }

    const pts = cloud.points
    if (!pts || pts.length === 0) return

    const n = pts.length

    // Pack into typed arrays so we can transfer the underlying ArrayBuffers
    // to the main thread without copying.
    const pos         = new Float32Array(n * 3)
    const clusterIds  = new Int32Array(n)
    const intensities = new Float32Array(n)

    for (let i = 0; i < n; i++) {
      const p = pts[i]
      pos[3 * i]     = p.x
      pos[3 * i + 1] = p.y
      pos[3 * i + 2] = p.z
      clusterIds[i]  = p.cluster_id ?? -1
      intensities[i] = p.intensity  ?? 1.0
    }

    self.postMessage(
      { type: 'cloud', pos, clusterIds, intensities, count: n,
        stamp: cloud.stamp, id: cloud.id },
      [pos.buffer, clusterIds.buffer, intensities.buffer]
    )
  }
}

function retry() { setTimeout(connect, 3000) }

// ─── Classifier connection ────────────────────────────────────────────────────

function connectClassifier() {
  try { classifierWs = new WebSocket(classifierUrl) }
  catch (_) { retryClassifier(); return }

  classifierWs.onopen  = () =>
    self.postMessage({ type: 'classifierStatus', connected: true })
  classifierWs.onerror = () => classifierWs.close()
  classifierWs.onclose = () => {
    self.postMessage({ type: 'classifierStatus', connected: false })
    retryClassifier()
  }

  classifierWs.onmessage = (e) => {
    let data
    try { data = JSON.parse(e.data) } catch (_) { return }
    self.postMessage({ type: 'classifications', data })
  }
}

function retryClassifier() { setTimeout(connectClassifier, 3000) }
