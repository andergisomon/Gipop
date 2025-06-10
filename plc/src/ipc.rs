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