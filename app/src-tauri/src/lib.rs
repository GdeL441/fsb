use std::{
    io::BufRead,
    process::{Command, Stdio},
    sync::{Arc, Mutex},
};

use tauri::{Emitter, Manager, State};
use wifi_rs::{prelude::*, WiFi};

#[tauri::command]
async fn scan() -> Vec<String> {
    println!("scan");
    let wifi = tokio_wifiscanner::scan()
        .await
        .expect("Cannot scan network");
    println!("{wifi:?}");
    wifi.into_iter().map(|w| w.ssid).collect()
}

#[tauri::command]
fn get_url() -> String {
    "ws://192.168.4.1/ws".to_string()
}

#[tauri::command]
fn connect(state: State<'_, AppData>, ssid: String) -> Option<String> {
    println!("Connect to {ssid}");
    let mut wifi = state.wifi.lock().unwrap();
    match wifi.connect(&ssid, "") {
        Ok(result) => {
            if result == true {
                println!("Connection Successful.");
                // TODO: Discover with mDNS
                return Some("ws://192.168.4.1/ws".to_string());
            } else {
                println!("Invalid password.");
            }
        }
        Err(err) => println!("The following error occurred: {:?}", err),
    }
    return None;
}

#[derive(Clone, Debug)]
struct AppData {
    wifi: Arc<Mutex<WiFi>>,
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let config = Some(Config {
        interface: Some("en0"),
    });

    let wifi = Arc::new(Mutex::new(WiFi::new(config)));

    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(tauri::generate_handler![scan, connect, get_url])
        .setup(|app| {
            #[cfg(debug_assertions)]
            app.get_webview_window("main").unwrap().open_devtools();

            app.manage(AppData { wifi });

            let handle = app.handle().clone();

            let _handler = std::thread::spawn(move || {
                // let child = Command::new(". ./src-tauri/my-venv/bin/activate")
                //     .spawn()
                //     .expect("failed to activate python venv");
                let mut child = Command::new("./run.sh")
                    // .arg("read_ds4.py")
                    .stdout(Stdio::piped()) // Capture the output
                    .spawn()
                    .expect("Failed to start Python process");

                let mut reader =
                    std::io::BufReader::new(child.stdout.take().expect("Failed to capture stdout"));

                let mut buffer = String::new();
                loop {
                    match reader.read_line(&mut buffer) {
                        Ok(0) => break, // End of output, exit the loop
                        Ok(_) => {
                            if !buffer.trim().is_empty() {
                                handle.emit("ds4-data", buffer.clone()).unwrap();
                            }

                            buffer.clear(); // Clear buffer for the next line
                        }
                        Err(e) => {
                            eprintln!("Failed to read Python output: {}", e);
                            break;
                        }
                    }
                }
                println!("break");
            });
            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
