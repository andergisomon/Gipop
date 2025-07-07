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
    pub rmt_rag: u32,
    pub rmt_area_2_lights: u32,
}

// Solely for OPC UA server, makes piping data easier
pub struct IpcData {
    pub temperature: f32,
    pub humidity: f32,
    pub status: u32,
    pub area_1_lights: u32,
    pub area_2_lights: u32,
    pub area_1_lights_hmi_cmd: u32, // incoming to PLC
    pub modbus_ai_0: f32,
    pub modbus_di_0: u32,
    pub rmt_rag: u32,
    pub rmt_area_2_lights: u32,
}

// A bit of an oddball. "Plc" here refers to the EtherCAT MainDevice as well, but the Modbus driver is run as a separate non-RT process
// use ToLogic/FromLogic to indicate direction of data
// Tx: IRIV IO -> Anywhere else
// Rx: Anywhere else -> IRIV IO
#[derive(Debug, Default, PlacementDefault, ZeroCopySend)]
#[type_name("ModbusIpcDataTx")]
#[repr(C)]
pub struct ModbusIpcDataTx {
    pub modbus_ai_0: f32,
    pub modbus_di_0: u32,
}