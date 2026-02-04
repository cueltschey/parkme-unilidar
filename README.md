# Parkme UniLiDAR producer

This program collects, processes and tranmits simulated or real point cloud data using the UniLiDAR SDK.

It can be configured to use either a ray tracing simulation, or a real LiDAR communicating over UDP.

The data is sent to a configured destination over websockets.

## Dependencies

If building with docker:
- Docker

If building baremetal:
- CMake
- Make
- G++ (or other C++ compiler)

## Building

To build with docker:
```bash
docker build . -f Dockerfile.alpine -t ghcr.io/cueltschey/parkme-unitree
```

To build baremetal:
```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Running

Run the following:
```bash
parkme-unitree --config /path/to/config.yaml
```

or

```bash
parkme-unitree -c /path/to/config.yaml
```

## Configuration


