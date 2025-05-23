use gilrs::{Axis, Button, Event, EventType, Gilrs};
use serde::Serialize;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::Duration;

#[derive(Serialize, Clone, Debug)]
pub struct ControllerData {
    left_x: f32,
    left_y: f32,
    right_x: f32,
    right_y: f32,
    up: bool,
    down: bool,
}

#[derive(Debug)]
pub struct DS4Controller {
    rx: Receiver<ControllerData>,
}

impl DS4Controller {
    pub fn new() -> Self {
        let (tx, rx) = mpsc::channel();
        Self::spawn_controller_thread(tx);
        Self { rx }
    }

    pub fn receive(&self) -> Option<ControllerData> {
        // Drain any buffered messages to get the most recent one
        let mut latest = None;
        while let Ok(data) = self.rx.try_recv() {
            latest = Some(data);
        }
        latest
    }

    fn spawn_controller_thread(tx: Sender<ControllerData>) {
        thread::spawn(move || {
            let mut gilrs = match Gilrs::new() {
                Ok(g) => g,
                Err(e) => {
                    eprintln!("Failed to initialize controller: {}", e);
                    return;
                }
            };

            let mut data = ControllerData {
                left_x: 0.0,
                left_y: 0.0,
                right_x: 0.0,
                right_y: 0.0,
                up: false,
                down: false,
            };

            loop {
                data.down = false;
                data.up = false;
                while let Some(Event { event, .. }) = gilrs.next_event() {
                    match event {
                        EventType::AxisChanged(axis, value, _) => {
                            match axis {
                                Axis::LeftStickX => data.left_x = value,
                                Axis::LeftStickY => data.left_y = value,
                                Axis::RightStickX => data.right_x = value,
                                Axis::RightStickY => data.right_y = value,
                                _ => {}
                            }
                        }
                        EventType::ButtonPressed(btn, _) => match btn {
                            Button::DPadUp => {
                                println!("arrow up");
                                data.up = true;
                            }
                            Button::DPadDown => {
                                println!("arrow down");
                                data.down = true
                            }
                            _ => {}
                        },
                        _ => {}
                    }
                }

                if tx.send(data.clone()).is_err() {
                    break;
                }

                // Increase sleep time to reduce data rate
                thread::sleep(Duration::from_millis(100));
            }
        });
    }
}
