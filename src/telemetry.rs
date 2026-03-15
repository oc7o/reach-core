use mavlink::ardupilotmega::MavMessage;

use crate::drone::TelemetryData;

/// Tries to extract a TelemetryData from a MAVLink message.
/// Returns None for messages we don't care about (heartbeats, attitude, etc.).
pub fn extract_telemetry(msg: &MavMessage) -> Option<TelemetryData> {
    match msg {
        // GLOBAL_POSITION_INT is the fused EKF position — preferred over raw GPS.
        // MAVLink units: lat/lon in 1e-7 degrees, alt in millimeters.
        MavMessage::GLOBAL_POSITION_INT(data) => Some(TelemetryData {
            latitude: data.lat as f64 / 1e7,
            longitude: data.lon as f64 / 1e7,
            altitude: data.alt as f32 / 1000.0, // mm -> meters
            timestamp_ms: chrono::Utc::now().timestamp_millis(),
        }),
        _ => None,
    }
}
