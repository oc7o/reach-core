use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{transport::Server, Request, Response, Status};

mod drone {
    tonic::include_proto!("drone");
}

use drone::{
    drone_control_server::{DroneControl, DroneControlServer},
    Empty, TelemetryData,
};

#[derive(Default)]
pub struct DroneService {}

#[tonic::async_trait]
impl DroneControl for DroneService {
    type StreamTelemetryStream = ReceiverStream<Result<TelemetryData, Status>>;

    async fn stream_telemetry(
        &self,
        _request: Request<Empty>,
    ) -> Result<Response<Self::StreamTelemetryStream>, Status> {
        let (tx, rx) = mpsc::channel(128);

        tokio::spawn(async move {
            let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(100)); // 10Hz

            loop {
                interval.tick().await;

                // TODO: Get real GPS data here
                let telemetry = TelemetryData {
                    latitude: 52.520008, // Berlin coordinates as example
                    longitude: 13.404954,
                    altitude: 34.0,
                    timestamp_ms: chrono::Utc::now().timestamp_millis(),
                };

                if tx.send(Ok(telemetry)).await.is_err() {
                    break; // Client disconnected
                }
            }
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr = "0.0.0.0:50051".parse()?;
    let service = DroneService::default();

    println!("Server listening on {}", addr);

    Server::builder()
        .add_service(DroneControlServer::new(service))
        .serve(addr)
        .await?;

    Ok(())
}
