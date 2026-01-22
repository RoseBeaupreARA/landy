use dashmap::DashMap;
use serde::{Deserialize, Serialize};
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::net::UdpSocket;
use tokio::sync::oneshot;
use tokio::time::timeout;
use std::future::Future;
use chrono::Utc;
use serde_json;
use serde_json::json;
use crate::LLA;

#[derive(Serialize, Debug)]
struct RequestPacket {
    req: u32,
    id: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    data: Option<serde_json::Value>,
}

#[derive(Deserialize, Debug, Clone)]
pub struct ResponsePacket {
    pub req: u32,
    pub id: u64,
    pub res: i32,
    pub data: Option<serde_json::Value>,
}

#[derive(thiserror::Error, Debug)]
pub enum DeviceError {
    #[error("IO Error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Serialization Error: {0}")]
    Serialization(#[from] rmp_serde::encode::Error),
    #[error("Deserialization Error: {0}")]
    Deserialization(#[from] rmp_serde::decode::Error),
    #[error("Request timed out after max retries")]
    Timeout,
    #[error("Internal channel closed")]
    InternalError,
}

pub struct Skypack {
    socket: Arc<UdpSocket>,
    target_addr: SocketAddr,
    pending_requests: Arc<DashMap<(u32, u64), oneshot::Sender<ResponsePacket>>>,
    next_id: AtomicU64,
}

/// A handle returned to the user.
/// Can be awaited directly, or polled manually via `is_finished()`.
pub struct RequestHandle {
    inner: tokio::task::JoinHandle<Result<ResponsePacket, DeviceError>>,
}

impl RequestHandle {
    /// Non-blocking check to see if the request is done.
    pub fn is_finished(&self) -> bool {
        self.inner.is_finished()
    }

    /// Wait for the result asynchronously.
    pub async fn wait(self) -> Result<ResponsePacket, DeviceError> {
        // Flatten the JoinError into our DeviceError
        self.inner.await.unwrap_or(Err(DeviceError::InternalError))
    }
}

// Allow the handle to be awaited directly
impl Future for RequestHandle {
    type Output = Result<ResponsePacket, DeviceError>;

    fn poll(mut self: std::pin::Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> std::task::Poll<Self::Output> {
        use std::task::Poll;
        match std::pin::Pin::new(&mut self.inner).poll(cx) {
            Poll::Ready(Ok(res)) => Poll::Ready(res),
            Poll::Ready(Err(_)) => Poll::Ready(Err(DeviceError::InternalError)),
            Poll::Pending => Poll::Pending,
        }
    }
}

impl Skypack {
    pub async fn new(bind_addr: &str, target_addr: &str) -> Result<Arc<Self>, DeviceError> {
        let socket = UdpSocket::bind(bind_addr).await?;
        let target: SocketAddr = target_addr.parse().expect("Invalid target address");
        let device = Arc::new(Self {
            socket: Arc::new(socket),
            target_addr: target,
            pending_requests: Arc::new(DashMap::new()),
            next_id: AtomicU64::new(rand::random()),
        });

        device.start_background_listener();

        Ok(device)
    }

    fn start_background_listener(self: &Arc<Self>) {
        let socket = self.socket.clone();
        let pending_requests = self.pending_requests.clone();

        tokio::spawn(async move {
            let mut buf = [0u8; 65536];
            loop {
                match socket.recv_from(&mut buf).await {
                    Ok((size, _src)) => {
                        // Attempt to deserialize generic response to get ID
                        if let Ok(response) = rmp_serde::from_slice::<ResponsePacket>(&buf[..size]) {
                            // If we have a waiter for this ID, send the response and remove from a map
                            if let Some((_, sender)) = pending_requests.remove(&(response.req, response.id)) {
                                let _ = sender.send(response);
                            }
                        }
                    }
                    Err(e) => eprintln!("UDP Receive Error: {}", e),
                }
            }
        });
    }

    /// The core logic handles ID generation, retry loops, and timeouts.
    /// It returns a Future that resolves when the whole process is done.
    async fn perform_request(&self, req: u32, data: Option<serde_json::Value>) -> Result<ResponsePacket, DeviceError> {
        // 1. Generate ID (increments automatically)
        let id = self.next_id.fetch_add(1, Ordering::SeqCst);

        let packet = RequestPacket { req, id, data };
        let buf = rmp_serde::to_vec_named(&packet)?;

        // Register a listener channel (done once for all retries to avoid race conditions)
        let (tx, mut rx) = oneshot::channel();
        self.pending_requests.insert((req, id), tx);

        // Ensure we remove the entry from the map when we're done, regardless of the outcome
        let _cleanup = scopeguard::guard(self.pending_requests.clone(), |pending| {
            pending.remove(&(req, id));
        });

        // Retry config
        let max_retries = 3;
        let timeout_duration = Duration::from_secs(1);

        for _attempt in 1..=max_retries {
            // Send request
            self.socket.send_to(&buf, self.target_addr).await?;

            // Wait for response OR timeout
            match timeout(timeout_duration, &mut rx).await {
                Ok(Ok(response)) => {
                    // Success
                    return Ok(response);
                }
                Ok(Err(_)) => {
                    // Channel closed unexpectedly (internal error)
                    return Err(DeviceError::InternalError);
                }
                Err(_) => {
                    // Timeout occurred, retry
                }
            }
        }

        Err(DeviceError::Timeout)
    }

    // --- User API ---

    /// Async call. Returns a Handle that can be polled or awaited.
    /// Spawns the work on the runtime so it proceeds even if not immediately awaited.
    pub fn get_telemetry(self: &Arc<Self>) -> RequestHandle {
        let self_clone = self.clone();
        let handle = tokio::spawn(async move {
            self_clone.perform_request(9, None).await
        });

        RequestHandle { inner: handle }
    }

    /// Synchronous blocking call.
    /// Creates a temporary runtime environment if one doesn't exist, or blocks the thread.
    pub fn get_telemetry_sync(self: &Arc<Self>) -> Result<ResponsePacket, DeviceError> {
        // Handle::block_on is the standard way to bridge sync -> async
        tokio::task::block_in_place(|| {
            tokio::runtime::Handle::current().block_on(async {
                self.perform_request(9, None).await
            })
        })
    }

    pub fn set_precision_landing_zone(self: &Arc<Self>, lla: LLA, vel: nalgebra::Vector3<f32>, timestamp: f64) -> RequestHandle {
        let data = json!({ "items": [{
            "id": 1,
            "frame": "lla".to_owned(),
            "pos": [lla.latitude.to_degrees(), lla.longitude.to_degrees(), lla.altitude],
            "vel":  [vel[0], vel[1], vel[2]],
            "rpy": [0., 0., 0.],
            "ts": timestamp
        }] });

        let self_clone = self.clone();
        let handle = tokio::spawn(async move {
            self_clone.perform_request(46, Some(data)).await
        });

        RequestHandle { inner: handle }
    }
}
