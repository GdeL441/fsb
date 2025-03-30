use dual_shock4_controller::joystick::{DeviceInfo, Joystick};
use std::sync::{Arc, Mutex};

use tauri::{Manager, State};
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

    let handler = std::thread::spawn(|| {
        let joystick = Joystick::new();
        let device_info = DeviceInfo {
            vid: 0x054c,
            pid: 0x05c4,
        }; //HID\VID_054C&PID_05C4\7&3869AC07&0&0000
        let device = joystick.connect(device_info).expect("can't find device!"); //
        loop {
            let mut buf = [0u8; 64];
            device.read_timeout(&mut buf[..], 1000).unwrap();
            let gamepad = joystick.get_gamepad().get_state(&buf);
            if gamepad.x_button.pressed {
                println!("× button is pressed");
                break;
            }
        }
    });
    // tokio::spawn(ds4());

    let wifi = Arc::new(Mutex::new(WiFi::new(config)));

    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(tauri::generate_handler![scan, connect, get_url])
        .setup(|app| {
            #[cfg(debug_assertions)]
            app.get_webview_window("main").unwrap().open_devtools();

            app.manage(AppData { wifi });
            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}

// async fn ds4() {
//     let joystick = Joystick::new();
//     let device_info = DeviceInfo {
//         vid: 0x054c,
//         pid: 0x05c4,
//     }; //HID\VID_054C&PID_05C4\7&3869AC07&0&0000
//     let device = joystick.connect(device_info).expect("can't find device!"); //
//     loop {
//         let mut buf = [0u8; 64];
//         device.read_timeout(&mut buf[..], 1000).unwrap();
//         let gamepad = joystick.get_gamepad().get_state(&buf);
//         if gamepad.x_button.pressed {
//             println!("× button is pressed");
//             break;
//         }
//     }
// }
