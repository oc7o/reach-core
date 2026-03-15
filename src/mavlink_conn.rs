use std::sync::{Arc, Mutex};
use std::time::Duration;

use mavlink::ardupilotmega::{
    MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, HEARTBEAT_DATA,
};
use mavlink::{MavConnection, MavHeader};
use tokio::sync::{broadcast, mpsc};

use crate::drone::TelemetryData;
use crate::telemetry::extract_telemetry;

const TELEMETRY_CAPACITY: usize = 64;
const COMMAND_CAPACITY: usize = 32;

/// Shorthand for the shared current connection type.
/// `Option` because there may be no active connection (e.g. during reconnect).
type SharedConn = Arc<Mutex<Option<Arc<dyn MavConnection<MavMessage> + Send + Sync>>>>;

pub struct MavlinkBridge {
    pub telemetry_tx: broadcast::Sender<TelemetryData>,
    pub command_tx: mpsc::Sender<MavMessage>,
}

impl MavlinkBridge {
    pub fn start(serial_url: &str) -> Self {
        let (telemetry_tx, _) = broadcast::channel(TELEMETRY_CAPACITY);
        let (command_tx, command_rx) = mpsc::channel(COMMAND_CAPACITY);

        // Shared slot for the active connection. Both the reconnect loop and
        // the command thread point to this same value.
        let shared_conn: SharedConn = Arc::new(Mutex::new(None));

        // Command thread — spawned once, lives forever.
        // It reads from the channel and writes to whatever connection is
        // currently active (or skips if there is none).
        let cmd_shared = Arc::clone(&shared_conn);
        std::thread::spawn(move || {
            run_command_loop(cmd_shared, command_rx);
        });

        let tx = telemetry_tx.clone();
        let url = serial_url.to_string();

        tokio::task::spawn_blocking(move || {
            run_mavlink_loop(&url, tx, shared_conn);
        });

        MavlinkBridge {
            telemetry_tx,
            command_tx,
        }
    }
}

fn run_mavlink_loop(
    serial_url: &str,
    telemetry_tx: broadcast::Sender<TelemetryData>,
    shared_conn: SharedConn,
) {
    loop {
        eprintln!("MAVLink: connecting to {serial_url}...");

        match mavlink::connect::<MavMessage>(serial_url) {
            Err(e) => {
                eprintln!("MAVLink: connection failed: {e}. Retrying in 3s...");
                std::thread::sleep(Duration::from_secs(3));
            }
            Ok(conn) => {
                // Coerce the concrete Connection type to the trait object so it
                // can be stored in SharedConn and shared across threads.
                let conn: Arc<dyn MavConnection<MavMessage> + Send + Sync> = Arc::new(conn);

                // Publish the new connection so the command thread can use it.
                *shared_conn.lock().unwrap() = Some(Arc::clone(&conn));

                // Heartbeat thread.
                let hb_conn = Arc::clone(&conn);
                std::thread::spawn(move || {
                    run_heartbeat_loop(hb_conn);
                });

                eprintln!("MAVLink: connected. Waiting for telemetry...");

                loop {
                    match conn.recv() {
                        Ok((_header, msg)) => {
                            if let Some(telemetry) = extract_telemetry(&msg) {
                                let _ = telemetry_tx.send(telemetry);
                            }
                        }
                        Err(e) => {
                            eprintln!("MAVLink: recv error: {e}. Reconnecting...");
                            // Clear the shared connection so the command thread
                            // doesn't try to send on a dead connection.
                            *shared_conn.lock().unwrap() = None;
                            break;
                        }
                    }
                }
            }
        }
    }
}

/// Reads outbound commands and sends them on the current active connection.
/// If no connection is active the message is silently dropped.
fn run_command_loop(shared_conn: SharedConn, mut command_rx: mpsc::Receiver<MavMessage>) {
    let header = MavHeader::default();

    while let Some(msg) = command_rx.blocking_recv() {
        // Grab the current connection (if any) and release the lock immediately
        // so we don't hold it while doing I/O.
        let conn = shared_conn.lock().unwrap().clone();

        if let Some(conn) = conn {
            if conn.send(&header, &msg).is_err() {
                // Send failed — the recv loop will handle reconnection.
                eprintln!("MAVLink: command send failed (connection lost)");
            }
        } else {
            eprintln!("MAVLink: command dropped (no active connection)");
        }
    }
}

fn run_heartbeat_loop(conn: Arc<dyn MavConnection<MavMessage> + Send + Sync>) {
    let heartbeat = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });

    let header = MavHeader::default();

    loop {
        if conn.send(&header, &heartbeat).is_err() {
            break;
        }
        std::thread::sleep(Duration::from_secs(1));
    }
}
