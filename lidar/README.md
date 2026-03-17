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
docker build . -f Dockerfile -t ghcr.io/cueltschey/parkme-unilidar
```

To build bare-metal:
```bash
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

## Configuration

The config file must contain either an `emulation` or `testbed` top-level key.

### Emulation mode

```yaml
emulation:
  websocket_port: 9002
  cloud_generation_interval: 0.05

  # LiDAR world-space position (metres). The sensor is placed here before
  # scene_update transforms all objects into the lidar-relative frame.
  # z: 1.0 puts the sensor 1 m above the ground plane, which is the minimum
  # needed for downward rays to intersect objects sitting at z = 0.
  lidar_x: 0.0
  lidar_y: 0.0
  lidar_z: 1.0

  # Unused — ray pattern comes from csv/mid360.csv
  nof_rings: 132
  points_per_ring: 220
  # Unused — meshes are hardcoded in emulation_producer.cpp
  obj_file: "car.obj"
```

### Testbed (UDP) mode

```yaml
testbed:
  udp_ip: "192.168.1.100"
  udp_port: 2368
```
