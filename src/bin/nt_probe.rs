use nt_client::subscribe::ReceivedMessage;
use nt_client::{Client, NewClientOptions};

#[tokio::main]
async fn main() {
    let port: u16 = std::env::args().nth(1).unwrap().parse().unwrap();
    let client = Client::new(NewClientOptions {
        unsecure_port: port,
        secure_port: None,
        ..Default::default()
    });
    client
        .connect_setup(|c| {
            let t = c.topic("");
            tokio::spawn(async move {
                let mut opts = nt_client::subscribe::SubscriptionOptions::default();
                opts.all = Some(true); opts.prefix = Some(true);
                let mut sub = t.subscribe(opts).await.unwrap();
                while let Ok(m) = sub.recv().await {
                    match m {
                        ReceivedMessage::Announced(t) => println!("announce: {}", t.name()),
                        ReceivedMessage::Updated((t, v)) => println!("update: {} = {}", t.name(), v),
                        _ => {}
                    }
                }
            });
        })
        .await
        .unwrap();
}
