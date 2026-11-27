# reach-core

Rust companion-computer service for the **REACH** autonomous drone platform. Bridges a Pixhawk flight controller (MAVLink2 over serial) to ground-station clients over gRPC.

## Architecture

- **MAVLink bridge** (`src/mavlink_conn.rs`) — Auto-reconnecting serial link to ArduPilot (`ardupilotmega`), continuous heartbeat loop, and telemetry broadcast channel
- **Telemetry extraction** (`src/telemetry.rs`) — Extracts fused EKF `GLOBAL_POSITION_INT` updates into Protobuf `TelemetryData`
- **gRPC server** (`src/service.rs`) — Implements `drone.DroneControl` from [`reach-proto`](https://github.com/oc7o/reach-proto) (`StreamTelemetry`, `Arm`, `Disarm`, `SendRCOverride`) via `tonic` on port `50051`

## Running

```bash
git submodule update --init --recursive
cargo run --release
```

## Related Repositories

- [`reach-proto`](https://github.com/oc7o/reach-proto) — Shared Protocol Buffers definitions
- [`reach-control`](https://github.com/oc7o/reach-control) — SwiftUI ground station client
- [`reach-auto-fly`](https://github.com/oc7o/reach-auto-fly) — Archived v1 autonomous Python + computer vision stack
