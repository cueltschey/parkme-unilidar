// WebSocket worker — runs entirely off the main thread.
// Parses JSON, packs point data into typed arrays, and transfers
// them (zero-copy) to the main thread via Transferable ArrayBuffers.

let ws = null
let wsUrl = null

self.onmessage = (e) => {
  if (e.data.type === 'init') {
    wsUrl = e.data.wsUrl
    connect()
  }
}

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
    const pos        = new Float32Array(n * 3)
    const clusterIds = new Int32Array(n)
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
