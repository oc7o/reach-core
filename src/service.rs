use mavlink::ardupilotmega::{
    MavCmd, MavMessage, RC_CHANNELS_OVERRIDE_DATA, COMMAND_LONG_DATA,
};
use tokio::sync::{broadcast, mpsc};
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status};

use crate::drone::{
    drone_control_server::DroneControl,
    CommandResult, Empty, RcChannels, TelemetryData,
};

pub struct DroneService {
    pub telemetry_tx: broadcast::Sender<TelemetryData>,
    pub command_tx: mpsc::Sender<MavMessage>,
}

#[tonic::async_trait]
impl DroneControl for DroneService {
    type StreamTelemetryStream = ReceiverStream<Result<TelemetryData, Status>>;

    async fn stream_telemetry(
        &self,
        _request: Request<Empty>,
    ) -> Result<Response<Self::StreamTelemetryStream>, Status> {
        let mut rx = self.telemetry_tx.subscribe();
        let (tx, grpc_rx) = mpsc::channel(32);

        tokio::spawn(async move {
            loop {
                match rx.recv().await {
                    Ok(telemetry) => {
                        if tx.send(Ok(telemetry)).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(skipped)) => {
                        eprintln!("gRPC client lagged, skipped {skipped} telemetry frames");
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
        });

        Ok(Response::new(ReceiverStream::new(grpc_rx)))
    }

    async fn arm(&self, _request: Request<Empty>) -> Result<Response<CommandResult>, Status> {
        let msg = arm_disarm_message(true);
        self.send_command(msg).await
    }

    async fn disarm(&self, _request: Request<Empty>) -> Result<Response<CommandResult>, Status> {
        let msg = arm_disarm_message(false);
        self.send_command(msg).await
    }

    async fn send_rc_override(
        &self,
        request: Request<RcChannels>,
    ) -> Result<Response<CommandResult>, Status> {
        let ch = request.into_inner();

        // Clamp all values to the valid PWM range 1000–2000.
        let msg = MavMessage::RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA {
            target_system: 1,
            target_component: 1,
            chan1_raw: clamp_pwm(ch.roll),
            chan2_raw: clamp_pwm(ch.pitch),
            chan3_raw: clamp_pwm(ch.throttle),
            chan4_raw: clamp_pwm(ch.yaw),
            // Channels 5–8: 0 means "do not override".
            chan5_raw: 0,
            chan6_raw: 0,
            chan7_raw: 0,
            chan8_raw: 0,
        });

        self.send_command(msg).await
    }
}

impl DroneService {
    /// Sends a MAVLink message to the command channel and returns a
    /// CommandResult. If the channel is full or closed, returns an error.
    async fn send_command(&self, msg: MavMessage) -> Result<Response<CommandResult>, Status> {
        match self.command_tx.send(msg).await {
            Ok(_) => Ok(Response::new(CommandResult {
                success: true,
                message: String::new(),
            })),
            Err(_) => Ok(Response::new(CommandResult {
                success: false,
                message: "MAVLink connection unavailable".to_string(),
            })),
        }
    }
}

/// Builds a COMMAND_LONG message for arming or disarming.
fn arm_disarm_message(arm: bool) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: if arm { 1.0 } else { 0.0 }, // 1.0 = arm, 0.0 = disarm
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    })
}

/// Clamps a PWM value to the valid 1000–2000 µs range.
fn clamp_pwm(value: u32) -> u16 {
    value.clamp(1000, 2000) as u16
}
