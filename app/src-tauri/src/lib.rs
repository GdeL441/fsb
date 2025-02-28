use std::sync::{Arc, Mutex};

use tauri::{Manager, State};
use wifi_rs::{
    prelude::{Config, Connectivity},
    WiFi,
};

#[tauri::command]
fn scan() -> Vec<String> {
    let wifi = wifiscanner::scan().unwrap_or_default();
    println!("{wifi:?}");
    wifi.into_iter().map(|w| w.ssid).collect()
}

#[tauri::command]
fn connect(state: State<'_, AppData>, ssid: String) {
    println!("Connect to {ssid}");
    let mut wifi = state.wifi.lock().unwrap();
    match wifi.connect("ssid", "") {
        Ok(result) => println!(
            "{}",
            if result == true {
                "Connection Successful."
            } else {
                "Invalid password."
            }
        ),
        Err(err) => println!("The following error occurred: {:?}", err),
    }
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
        .invoke_handler(tauri::generate_handler![scan, connect])
        .setup(|app| {
            #[cfg(debug_assertions)]
            app.get_webview_window("main").unwrap().open_devtools();

            app.manage(AppData { wifi });
            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
