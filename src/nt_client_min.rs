//! Minimal NT4 client — just what the orchestrator bridge needs: connect to a
//! robot's NT server, subscribe to actuator topics, publish sensor topics.
//! Written against the same wire format as `nt_server` / the NT4 spec:
//! WS + JSON text control msgs + msgpack binary data frames [uid, ts, type, val].
use futures_util::{SinkExt, StreamExt};
use rmpv::Value;
use serde_json::{json, Value as J};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::http::HeaderValue;
use tokio_tungstenite::tungstenite::Message;

pub const NT_SUBPROTOCOL: &str = "v4.1.networktables.first.wpi.edu";

fn now_us() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_micros() as i64
}

type TopicMap = Arc<Mutex<HashMap<u32, String>>>; // server topic uid -> name

/// Split halves: call `publish` for announced client topics, `set` to send data.
#[derive(Clone)]
pub struct NtClient {
    /// uid -> name for server-announced topics (subscription deliveries)
    names: TopicMap,
    /// topic name -> update values, latest only
    values: Arc<Mutex<HashMap<String, Value>>>,
    sender: tokio::sync::mpsc::UnboundedSender<Message>,
    next_pubuid: Arc<Mutex<u32>>,
    /// announced pubuids (we just send frames; no server ack needed for M0)
    pub_uids: Arc<Mutex<HashMap<String, (u32, u32)>>>, // name -> (pubuid, type_id)
    alive: Arc<AtomicBool>,
}

impl NtClient {
    pub async fn connect(port: u16) -> anyhow::Result<Self> {
        let url = format!("ws://127.0.0.1:{}/nt/frc-sim", port);
        let mut req = url.into_client_request()?;
        req.headers_mut().insert(
            "Sec-WebSocket-Protocol",
            HeaderValue::from_static(NT_SUBPROTOCOL),
        );
        let (ws, _) = tokio_tungstenite::connect_async(req).await?;
        let (mut sink, mut stream) = ws.split();

        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<Message>();
        tokio::spawn(async move {
            while let Some(m) = rx.recv().await {
                if sink.send(m).await.is_err() {
                    break;
                }
            }
        });

        // time sync message (spec: client sends its timestamp once)
        let _ = tx.send(Message::Text(
            serde_json::to_string(&J::Array(vec![json!({
                "method": "timestamp", "params": {"time": now_us()}
            })]))
            .unwrap()
            .into(),
        ));

        let client = Self {
            names: Arc::new(Mutex::new(HashMap::new())),
            values: Arc::new(Mutex::new(HashMap::new())),
            sender: tx.clone(),
            next_pubuid: Arc::new(Mutex::new(100)),
            pub_uids: Arc::new(Mutex::new(HashMap::new())),
            alive: Arc::new(AtomicBool::new(true)),
        };

        // subscribe to everything under /sim/cmd_
        let _ = tx.send(Message::Text(
            serde_json::to_string(&J::Array(vec![json!({
                "method": "subscribe",
                "params": {"topics": ["/sim/"], "subuid": 1,
                           "options": {"prefix": true, "all": true}}
            })]))
            .unwrap()
            .into(),
        ));

        let names = client.names.clone();
        let values = client.values.clone();
        let alive = client.alive.clone();
        tokio::spawn(async move {
            while let Some(Ok(msg)) = stream.next().await {
                match msg {
                    Message::Text(txt) => {
                        if let Ok(msgs) = serde_json::from_str::<Vec<J>>(&txt) {
                            for m in msgs {
                                if m["method"].as_str() == Some("announce") {
                                    let p = &m["params"];
                                    if let (Some(id), Some(name)) =
                                        (p["id"].as_u64(), p["name"].as_str())
                                    {
                                        names
                                            .lock()
                                            .unwrap()
                                            .insert(id as u32, name.to_string());
                                    }
                                }
                            }
                        }
                    }
                    Message::Binary(bin) => {
                        let mut cur = &bin[..];
                        while let Ok(v) = rmpv::decode::read_value(&mut cur) {
                            if let Value::Array(arr) = v {
                                if arr.len() >= 4 {
                                    if let Some(uid) = arr[0].as_i64() {
                                        let name = names
                                            .lock()
                                            .unwrap()
                                            .get(&(uid as u32))
                                            .cloned();
                                        if let Some(n) = name {
                                            values
                                                .lock()
                                                .unwrap()
                                                .insert(n, arr[3].clone());
                                        }
                                    }
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
            alive.store(false, Ordering::Relaxed);
        });

        Ok(client)
    }

    /// Client-publish a topic (announce it to the server).
    pub fn publish(&self, name: &str, type_id: u32) {
        let type_name = match type_id {
            0x00 => "boolean",
            0x01 => "int",
            0x03 => "double",
            _ => "raw",
        };
        let mut pu = self.next_pubuid.lock().unwrap();
        let pubuid = *pu;
        *pu += 1;
        self.pub_uids
            .lock()
            .unwrap()
            .insert(name.to_string(), (pubuid, type_id));
        let _ = self.sender.send(Message::Text(
            serde_json::to_string(&J::Array(vec![json!({
                "method": "publish",
                "params": {"name": name, "pubuid": pubuid,
                           "type": type_name, "properties": {}}
            })]))
            .unwrap()
            .into(),
        ));
    }

    /// Send a data frame for a client-published topic.
    pub fn set_f64(&self, name: &str, v: f64) {
        let Some(&(pubuid, tid)) = self.pub_uids.lock().unwrap().get(name) else {
            return;
        };
        let frame = Value::Array(vec![
            Value::from(pubuid as i64),
            Value::from(now_us()),
            Value::from(tid as i64),
            Value::from(v),
        ]);
        let mut buf = Vec::new();
        rmpv::encode::write_value(&mut buf, &frame).unwrap();
        let _ = self.sender.send(Message::Binary(buf.into()));
    }

    pub fn alive(&self) -> bool {
        self.alive.load(Ordering::Relaxed)
    }

    /// Latest value received for a subscribed topic.
    pub fn get_f64(&self, name: &str) -> Option<f64> {
        self.values
            .lock()
            .unwrap()
            .get(name)
            .and_then(|v| v.as_f64())
    }
}
