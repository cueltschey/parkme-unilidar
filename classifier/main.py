"""
LiDAR cluster classifier service.

Connects to the lidar WebSocket (port 9002) as a client, receives JSON
point clouds with cluster_id labels, extracts geometric features per cluster,
and classifies each cluster as car / truck / pedestrian / ground / unknown.

Results are broadcast on port 9003 as JSON:
  {
    "stamp": <float>,
    "id":    <int>,
    "classifications": [
      {
        "cluster_id":  <int>,
        "label":       <str>,
        "confidence":  <float>,
        "centroid":    [x, y, z],
        "bbox":        [dx, dy, dz],
        "point_count": <int>
      }, ...
    ]
  }
"""

import asyncio
import json
import logging
import os

import numpy as np
import websockets

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)
log = logging.getLogger(__name__)

WS_IN = os.getenv("LIDAR_WS_URL", "ws://localhost:9002")
WS_OUT_PORT = int(os.getenv("CLASSIFIER_PORT", "9003"))

# ---------------------------------------------------------------------------
# Feature extraction
# ---------------------------------------------------------------------------

def extract_features(points: list[dict]) -> dict:
    """Return a flat dict of geometric features for a cluster."""
    pts = np.array([[p["x"], p["y"], p["z"]] for p in points], dtype=np.float32)

    min_pt = pts.min(axis=0)
    max_pt = pts.max(axis=0)
    dims = max_pt - min_pt          # [dx, dy, dz]
    centroid = pts.mean(axis=0)

    count = len(pts)
    dx, dy, dz = float(dims[0]), float(dims[1]), float(dims[2])
    long_side  = max(dx, dy)
    short_side = min(dx, dy)
    footprint  = dx * dy
    volume     = footprint * dz if footprint * dz > 1e-6 else 1e-6
    density    = count / volume
    aspect_xy  = long_side / (short_side + 1e-6)
    # Fraction of the bounding-box volume actually occupied (rough solidity).
    # Dense → compact solid object; sparse → open mesh or noise.
    solidity   = min(count / (density * volume + 1e-6), 1.0)

    return {
        "count":      count,
        "dx":         dx,
        "dy":         dy,
        "dz":         dz,
        "long_side":  long_side,
        "short_side": short_side,
        "footprint":  footprint,
        "volume":     volume,
        "density":    float(density),
        "aspect_xy":  float(aspect_xy),
        "solidity":   float(solidity),
        "centroid":   centroid.tolist(),
    }


# ---------------------------------------------------------------------------
# Classifier
# ---------------------------------------------------------------------------
# Each prototype is (feature_vector, label).  At runtime we compute the
# Mahalanobis-inspired cosine distance to each and pick the nearest winner,
# falling back to "unknown" when the best score is below a threshold.
#
# Feature order: [long_side, short_side, dz, log10(count), aspect_xy]

_FEATURE_NAMES = ["long_side", "short_side", "dz", "log_count", "aspect_xy"]

# Typical values for each class derived from a Mid-360 scan at 1 m height:
#   car        ~4.5 m long, ~2 m wide, ~1.4 m tall, 150-600 pts, aspect ~2.2
#   truck/suv  ~6.5 m long, ~2.5 m wide, ~2.0 m tall, 300-1200 pts
#   pedestrian ~0.5 m wide, ~0.5 m wide, ~1.7 m tall, 10-60 pts
#   ground     large flat plane — large footprint, dz < 0.3

_PROTOTYPES: list[tuple[np.ndarray, str]] = [
    (np.array([4.5, 2.0, 1.4, np.log10(300), 2.2]),  "car"),
    (np.array([6.5, 2.5, 2.0, np.log10(600), 2.6]),  "truck"),
    (np.array([0.5, 0.5, 1.7, np.log10(30),  1.1]),  "pedestrian"),
    (np.array([8.0, 8.0, 0.2, np.log10(800), 1.0]),  "ground"),
]

# Per-feature standard deviations used to normalise distances.
_SCALES = np.array([3.0, 1.0, 0.8, 0.8, 1.5])


def _feature_vec(feat: dict) -> np.ndarray:
    return np.array([
        feat["long_side"],
        feat["short_side"],
        feat["dz"],
        np.log10(max(feat["count"], 1)),
        feat["aspect_xy"],
    ], dtype=np.float64)


def classify_cluster(feat: dict) -> tuple[str, float]:
    """Return (label, confidence) for a cluster described by *feat*."""
    # Hard rules that dominate the distance scoring.
    if feat["footprint"] > 30 and feat["dz"] < 0.3:
        return "ground", 0.95
    if feat["count"] < 5:
        return "unknown", 0.50

    fv = _feature_vec(feat)
    best_dist = float("inf")
    best_label = "unknown"

    for proto_vec, label in _PROTOTYPES:
        diff = (fv - proto_vec) / _SCALES
        dist = float(np.dot(diff, diff))  # squared Mahalanobis
        if dist < best_dist:
            best_dist = dist
            best_label = label

    # Convert distance to a confidence score via a soft-max-like mapping.
    # dist == 0 → 1.0; dist == 9 (3-sigma) → ~0.5
    confidence = float(1.0 / (1.0 + best_dist / 3.0))

    if confidence < 0.35:
        return "unknown", confidence
    return best_label, round(confidence, 3)


# ---------------------------------------------------------------------------
# Frame processing
# ---------------------------------------------------------------------------

def process_frame(raw: str) -> str:
    data = json.loads(raw)
    by_cluster: dict[int, list] = {}

    for p in data.get("points", []):
        cid = p.get("cluster_id", -1)
        if cid < 0:
            continue
        by_cluster.setdefault(cid, []).append(p)

    classifications = []
    for cid, points in by_cluster.items():
        if len(points) < 5:
            continue
        feat = extract_features(points)
        label, confidence = classify_cluster(feat)
        classifications.append({
            "cluster_id":  cid,
            "label":       label,
            "confidence":  confidence,
            "centroid":    feat["centroid"],
            "bbox":        [feat["dx"], feat["dy"], feat["dz"]],
            "point_count": feat["count"],
        })

    return json.dumps({
        "stamp":           data.get("stamp", 0.0),
        "id":              data.get("id", 0),
        "classifications": classifications,
    })


# ---------------------------------------------------------------------------
# WebSocket server (outbound — sends classifications to renderer / others)
# ---------------------------------------------------------------------------

_clients: set = set()


async def _ws_handler(websocket):
    _clients.add(websocket)
    log.info("Client connected  — %d active", len(_clients))
    try:
        await websocket.wait_closed()
    finally:
        _clients.discard(websocket)
        log.info("Client disconnected — %d active", len(_clients))


async def _broadcast(message: str) -> None:
    if not _clients:
        return
    await asyncio.gather(
        *(ws.send(message) for ws in list(_clients)),
        return_exceptions=True,
    )


# ---------------------------------------------------------------------------
# WebSocket client (inbound — reads from lidar producer)
# ---------------------------------------------------------------------------

async def _consume() -> None:
    while True:
        try:
            async with websockets.connect(WS_IN, ping_interval=20, max_size=16 * 1024 * 1024) as ws:
                log.info("Connected to lidar at %s", WS_IN)
                async for message in ws:
                    try:
                        result = process_frame(message)
                        await _broadcast(result)
                    except Exception as exc:
                        log.warning("Frame processing error: %s", exc)
        except Exception as exc:
            log.warning("Lidar connection lost: %s — retrying in 2 s", exc)
            await asyncio.sleep(2)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

async def _main() -> None:
    server = await websockets.serve(_ws_handler, "0.0.0.0", WS_OUT_PORT)
    log.info("Classifier WebSocket server listening on port %d", WS_OUT_PORT)
    await _consume()
    server.close()


if __name__ == "__main__":
    asyncio.run(_main())
