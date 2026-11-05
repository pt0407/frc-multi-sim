//! Minimal NT4 (NetworkTables 4.0) server — enough of the spec for a
//! standards-compliant NT4 client (nt_client / wpilib ntcore) to connect,
//! subscribe, publish, and exchange binary data messages.
//!
//! Spec: https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc
use futures_util::{SinkExt, StreamExt};
use rmpv::Value;
use serde_json::{json, Value as J};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::http::HeaderValue;
use tokio_tungstenite::tungstenite::Message;

pub const NT_SUBPROTOCOL: &str = "networktables.first.wpi.edu";

#[derive(Debug, Clone)]
pub struct Topic {
    pub name: String,
    pub id: u32,          // server-side uid sent in server->client data frames
    pub type_id: u32,     // NT4 type id (double=3, int=1, bool=0)
    pub type_name: String,
    pub value: Value,
    /// pubuid -> conn index for client-published topics
    pub client_pub: Option<(usize, u32)>,
}

struct State {
    topics: HashMap<String, Topic>,
    /// conn index -> ws sender
    conns: HashMap<usize, tokio::sync::mpsc::UnboundedSender<Message>>,
    /// subscriptions: conn -> list of (subuid, prefix match option)
    next_conn: AtomicU32,
    next_uid: AtomicU32,
    next_pubuid: AtomicU32,
}

/// Shared handle a host program (e.g. mock robot code) uses to read/write topics.
#[derive(Clone)]
pub struct NtServer {
    state: Arc<Mutex<State>>,
    time_us: Arc<AtomicU32>,
}

fn type_name(id: u32) -> &'static str {
    match id {
        0x00 => "boolean",
        0x01 => "int",
        0x02 => "float",
        0x03 => "double",
        0x04 => "string",
        0x05 => "json",
        0x10 => "boolean[]",
        0x11 => "int[]",
        0x12 => "float[]",
        0x13 => "double[]",
        0x14 => "string[]",
        _ => "raw",
    }
}

fn type_id(name: &str) -> u32 {
    match name {
        "boolean" => 0x00,
        "int" => 0x01,
        "float" => 0x02,
        "double" => 0x03,
        "string" => 0x04,
        "json" => 0x05,
        "boolean[]" => 0x10,
        "int[]" => 0x11,
        "float[]" => 0x12,
        "double[]" => 0x13,
        "string[]" => 0x14,
        _ => 0x06,
    }
}

fn now_us() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_micros() as i64
}

impl NtServer {
    pub async fn start(addr: &str) -> std::io::Result<Self> {
        let listener = TcpListener::bind(addr).await?;
        let server = NtServer {
            state: Arc::new(Mutex::new(State {
                topics: HashMap::new(),
                conns: HashMap::new(),
                next_conn: AtomicU32::new(1),
                next_uid: AtomicU32::new(1),
                next_pubuid: AtomicU32::new(1),
            })),
            time_us: Arc::new(AtomicU32::new(0)),
        };
        let s = server.clone();
        tokio::spawn(async move { s.accept_loop(listener).await });
        Ok(server)
    }

    /// Server-side publish (like robot code publishing a topic).
    pub fn publish(&self, name: &str, type_id: u32, value: Value) -> u32 {
        let mut st = self.state.lock().unwrap();
        let id = st.next_uid.fetch_add(1, Ordering::Relaxed);
        st.topics.insert(
            name.to_string(),
            Topic {
                name: name.to_string(),
                id,
                type_id,
                type_name: type_name(type_id).to_string(),
                value: value.clone(),
                client_pub: None,
            },
        );
        let announce = serde_json::to_string(&J::Array(vec![json!({
            "method": "announce",
            "params": {"name": name, "id": id, "type": type_name(type_id), "properties": {}}
        })]))
        .unwrap();
        for tx in st.conns.values() {
            let _ = tx.send(Message::Text(announce.clone().into()));
        }
        // send initial value
        drop(st);
        self.set(name, value);
        id
    }

    /// Update a topic's value and broadcast the binary data message.
    pub fn set(&self, name: &str, value: Value) {
        let mut st = self.state.lock().unwrap();
        let Some(topic) = st.topics.get_mut(name) else { return };
        topic.value = value.clone();
        let frame = Value::Array(vec![
            Value::from(topic.id as i64),
            Value::from(now_us()),
            Value::from(topic.type_id as i64),
            value,
        ]);
        let mut buf = Vec::new();
        rmpv::encode::write_value(&mut buf, &frame).unwrap();
        let bin = Message::Binary(buf.into());
        for tx in st.conns.values() {
            let _ = tx.send(bin.clone());
        }
    }

    pub fn get(&self, name: &str) -> Option<Value> {
        self.state
            .lock()
            .unwrap()
            .topics
            .get(name)
            .map(|t| t.value.clone())
    }

    pub fn get_f64(&self, name: &str) -> Option<f64> {
        self.get(name).and_then(|v| v.as_f64())
    }

    async fn accept_loop(self, listener: TcpListener) {
        while let Ok((stream, _)) = listener.accept().await {
            let me = self.clone();
            tokio::spawn(async move {
                // NT4 requires the subprotocol on the response
                let cb = |req: &tokio_tungstenite::tungstenite::handshake::server::Request,
                          mut resp: tokio_tungstenite::tungstenite::handshake::server::Response| {
                    // echo back a subprotocol the client actually offered
                    let offered = req
                        .headers()
                        .get("Sec-WebSocket-Protocol")
                        .and_then(|h| h.to_str().ok())
                        .unwrap_or("")
                        .to_string();
                    eprintln!("client offered subprotocols: {:?}", offered);
                    // echo the first offered token — we support the base NT4 protocol
                    if let Some(first) = offered.split(',').next() {
                        let first = first.trim();
                        if !first.is_empty() {
                            resp.headers_mut().insert(
                                "Sec-WebSocket-Protocol",
                                HeaderValue::from_str(first).unwrap(),
                            );
                        }
                    }
                    Ok(resp)
                };
                let Ok(ws) = tokio_tungstenite::accept_hdr_async(stream, cb).await else {
                    return;
                };
                me.handle_conn(ws).await;
            });
        }
    }

    async fn handle_conn(self, ws: tokio_tungstenite::WebSocketStream<tokio::net::TcpStream>) {
        let (mut sink, mut stream) = ws.split();
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<Message>();
        let conn_id = {
            let mut st = self.state.lock().unwrap();
            let id = st.next_conn.fetch_add(1, Ordering::Relaxed) as usize;
            st.conns.insert(id, tx.clone());
            id
        };

        // send "server started" / announce all existing topics
        {
            let msgs: Vec<Message> = {
                let st = self.state.lock().unwrap();
                let mut v = Vec::new();
                let anns: Vec<J> = st
                    .topics
                    .values()
                    .map(|t| {
                        json!({"method":"announce","params":{"name":t.name,"id":t.id,"type":t.type_name,"properties":{}}})
                    })
                    .collect();
                if !anns.is_empty() {
                    v.push(Message::Text(
                        serde_json::to_string(&J::Array(anns)).unwrap().into(),
                    ));
                }
                for t in st.topics.values() {
                    let frame = Value::Array(vec![
                        Value::from(t.id as i64),
                        Value::from(now_us()),
                        Value::from(t.type_id as i64),
                        t.value.clone(),
                    ]);
                    let mut buf = Vec::new();
                    rmpv::encode::write_value(&mut buf, &frame).unwrap();
                    v.push(Message::Binary(buf.into()));
                }
                v
            };
            for m in msgs {
                if sink.send(m).await.is_err() {
                    return;
                }
            }
        }

        let state = self.state.clone();
        let mut writer = tokio::spawn(async move {
            while let Some(msg) = rx.recv().await {
                if sink.send(msg).await.is_err() {
                    break;
                }
            }
        });

        // pubuid -> topic name map for this conn
        let mut my_pubs: HashMap<u32, u32> = HashMap::new(); // pubuid -> topic uid

        while let Some(Ok(msg)) = stream.next().await {
            match msg {
                Message::Text(txt) => {
                    let Ok(msgs) = serde_json::from_str::<Vec<J>>(&txt) else {
                        continue;
                    };
                    for m in msgs {
                        let method = m["method"].as_str().unwrap_or("");
                        let p = &m["params"];
                        match method {
                            "publish" => {
                                eprintln!("PUBLISH msg: {}", p);
                                let name = p["name"].as_str().unwrap_or("").to_string();
                                let ty = p["type"].as_str().unwrap_or("double").to_string();
                                let pubuid = p["pubuid"].as_i64().unwrap_or(0) as u32;
                                let mut st = state.lock().unwrap();
                                let uid = st.next_uid.fetch_add(1, Ordering::Relaxed);
                                st.topics.insert(
                                    name.clone(),
                                    Topic {
                                        name: name.clone(),
                                        id: uid,
                                        type_id: type_id(&ty),
                                        type_name: ty.clone(),
                                        value: Value::Nil,
                                        client_pub: Some((conn_id, pubuid)),
                                    },
                                );
                                my_pubs.insert(pubuid, uid);
                                // announce to ALL conns including this one
                                let ann = serde_json::to_string(&J::Array(vec![json!({
                                    "method":"announce",
                                    "params":{"name":name,"id":uid,"type":ty,"pubuid":pubuid,"properties":{}}
                                })]))
                                .unwrap();
                                for c in st.conns.values() {
                                    let _ = c.send(Message::Text(ann.clone().into()));
                                }
                            }
                            "subscribe" | "timestamp" | "unsubscribe" | "setproperties" => {}
                            _ => {}
                        }
                    }
                }
                Message::Binary(bin) => {
                    let mut cur = &bin[..];
                    // data frames are concatenated msgpack arrays
                    while let Ok(v) = rmpv::decode::read_value(&mut cur) {
                        if let Value::Array(arr) = v {
                            if arr.len() >= 4 {
                                let pubuid = arr[0].as_i64().unwrap_or(-1) as u32;
                                let val = arr[3].clone();
                                let mut st = state.lock().unwrap();
                                eprintln!("frame pubuid={} known={:?} val={}", pubuid, my_pubs.get(&pubuid).is_some(), arr[3]);
                                if let Some(&uid) = my_pubs.get(&pubuid) {
                                    // find topic by uid and update + rebroadcast
                                    let mut tname = None;
                                    for t in st.topics.values_mut() {
                                        if t.id == uid {
                                            t.value = val.clone();
                                            tname = Some(t.name.clone());
                                            break;
                                        }
                                    }
                                    let _ = tname;
                                    let frame = Value::Array(vec![
                                        Value::from(uid as i64),
                                        Value::from(now_us()),
                                        arr[2].clone(),
                                        val,
                                    ]);
                                    let mut buf = Vec::new();
                                    rmpv::encode::write_value(&mut buf, &frame).unwrap();
                                    for c in st.conns.values() {
                                        let _ = c.send(Message::Binary(buf.clone().into()));
                                    }
                                }
                            }
                        }
                    }
                }
                _ => {}
            }
        }
        writer.abort();
        self.state.lock().unwrap().conns.remove(&conn_id);
    }
}
