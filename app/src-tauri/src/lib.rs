use std::sync::{Arc, Mutex};

use tauri::{Emitter, Manager, State};
use wifi_rs::{prelude::*, WiFi};

mod ds4;

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
    // Using hardcoded "password" for WPA2 authentication
    match wifi.connect(&ssid, "password") {
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
    None
}

// Added new command to get controller data
#[tauri::command]
fn get_controller_data(state: State<'_, AppData>) -> Option<ds4::ControllerData> {
    let ds4 = state.ds4.lock().unwrap();
    ds4.receive()
}

#[derive(Clone, Debug)]
struct AppData {
    wifi: Arc<Mutex<WiFi>>,
    ds4: Arc<Mutex<ds4::DS4Controller>>,
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let config = Some(Config {
        interface: Some("en0"),
    });

    let wifi = Arc::new(Mutex::new(WiFi::new(config)));
    let ds4 = Arc::new(Mutex::new(ds4::DS4Controller::new()));

    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(tauri::generate_handler![
            scan,
            connect,
            get_url,
            get_controller_data // Added new command to handler
        ])
        .setup(|app| {
            #[cfg(debug_assertions)]
            app.get_webview_window("main").unwrap().open_devtools();

            app.manage(AppData {
                wifi,
                ds4: ds4.clone(),
            });

            let handle = app.handle().clone();

            std::thread::spawn(move || {
                let ds4 = ds4.lock().unwrap();
                loop {
                    if let Some(data) = ds4.receive() {
                        handle.emit("ds4-data", data).unwrap();
                    }
                    // Increase sleep time to reduce data rate
                    std::thread::sleep(std::time::Duration::from_millis(50));
                }
            });

            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
