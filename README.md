# Parkme UniLiDAR producer

This program collects, processes and transmits simulated or real point cloud data using the UniLiDAR SDK.

It can be configured to use either a ray tracing simulation, or a real LiDAR communicating over UDP.

The data is sent to a configured destination over WebSockets.

## Dependencies

### Docker build
- Docker

### Bare-metal build

| Dependency | Why it is needed |
|---|---|
| CMake ≥ 3.10, Make, G++/Clang++ | Build toolchain |
| **yaml-cpp** | Parses the YAML config file (`emulation` / `testbed` sections) |
| **Boost** (thread, regex, filesystem) | Boost.Beast (header-only, ships with Boost) provides the WebSocket server; Boost.Thread/Regex are required by Beast's internals |
| **PCL** (common, io, search, segmentation) | Point Cloud Library — used for Euclidean cluster extraction that groups LiDAR hit points into labelled objects before they are broadcast |
| **lidarsim.h** (header-only, included in `include/`) | Fast ray-scene intersection with per-object BVH acceleration trees; used in emulation mode to cast rays against OBJ/STL meshes |
| **UniLiDAR SDK** (`lib/libunilidar_sdk2.a`, pre-built) | Low-level hardware interface for the real Unitree Mid-360 LiDAR over UDP; used in testbed mode |

## Building

To build with Docker:
```bash
docker build ./lidar -f Dockerfile -t ghcr.io/cueltschey/parkme-unilidar
docker build ./renderer -f Dockerfile -t ghcr.io/cueltschey/parkme-renderer
```

To run the full stack:
```bash
docker-compose up
```

To build the lidar component bare-metal:
```bash
cd lidar
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Running

```bash
parkme_unilidar --config /path/to/config.yaml
# or
parkme_unilidar -c /path/to/config.yaml
```

The renderer is a static HTML/JS file served by a plain HTTP server:
```bash
cd renderer && python3 -m http.server 8080
```

## Configuration

See `lidar/README.md` for full configuration reference, including LiDAR position (`lidar_x/y/z`) and other emulation options.
