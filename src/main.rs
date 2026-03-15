use tonic::transport::Server;

mod drone {
    tonic::include_proto!("drone");
}

mod mavlink_conn;
mod service;
mod telemetry;

use drone::drone_control_server::DroneControlServer;
use mavlink_conn::MavlinkBridge;
use service::DroneService;

/// Serial port of the Pixhawk USB connection.
/// On Linux this is typically /dev/ttyACM0 or /dev/ttyUSB0.
/// On macOS: /dev/tty.usbmodem<...>
/// On Windows: COM3 (or similar) — format: "serial:COM3:57600"
const SERIAL_URL: &str = "serial:/dev/ttyACM0:57600";

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr = "0.0.0.0:50051".parse()?;

    // Start the MAVLink bridge. This spawns background threads for the recv
    // loop and heartbeat — it returns immediately.
    let bridge = MavlinkBridge::start(SERIAL_URL);

    let service = DroneService {
        telemetry_tx: bridge.telemetry_tx,
        command_tx: bridge.command_tx,
    };

    println!("Server listening on {addr}");

    Server::builder()
        .add_service(DroneControlServer::new(service))
        .serve(addr)
        .await?;

    Ok(())
}
