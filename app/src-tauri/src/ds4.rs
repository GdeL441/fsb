use gilrs::{Gilrs, Event, EventType, Axis};
use std::sync::mpsc::{self, Sender, Receiver};
use std::thread;
use std::time::Duration;
use serde::Serialize;

#[derive(Serialize, Clone, Debug)]
pub struct ControllerData {
    left_x: f32,
    left_y: f32,
    right_x: f32,
    right_y: f32,
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
        self.rx.try_recv().ok()
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
            };

            loop {
                while let Some(Event { event, .. }) = gilrs.next_event() {
                    match event {
                        EventType::AxisChanged(axis, value, _) => {
                            match axis {
                                Axis::LeftStickX => data.left_x = value,
                                Axis::LeftStickY => data.left_y = value, // Remove the inversion
                                Axis::RightStickX => data.right_x = value,
                                Axis::RightStickY => data.right_y = value, // Remove the inversion
                                _ => {}
                            }
                        }
                        _ => {}
                    }
                }

                if tx.send(data.clone()).is_err() {
                    break;
                }

                thread::sleep(Duration::from_millis(30));
            }
        });
    }
}
