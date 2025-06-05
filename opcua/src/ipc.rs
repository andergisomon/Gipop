use iceoryx2::prelude::*;

#[derive(Debug, Default, PlacementDefault, ZeroCopySend)]
#[type_name("IpcDataFromPlc")]
#[repr(C)]
pub struct IpcDataFromPlc {
    pub temperature: f32,
    pub humidity: f32,
    pub status: u32,
    pub area_1_lights: u32,
    pub area_2_lights: u32,
}

#[derive(Debug, Default, PlacementDefault, ZeroCopySend)]
#[type_name("IpcDataToPlc")]
#[repr(C)]
pub struct IpcDataToPlc {
    pub area_1_lights_hmi_cmd: u32, // incoming to PLC
}

// impl IpcDataFromPlc {
//     pub fn new() -> Self {
//         Self {
//             temperature: 0.0,
//             humidity: 0.0,
//             status: 0,
//             area_1_lights: 0,
//             area_2_lights: 0,
//         }
//     }
// }

// impl IpcDataToPlc {
//     pub fn new() -> Self {
//         Self {
//             area_1_lights_hmi_cmd: 0
//         }
//     }
// }