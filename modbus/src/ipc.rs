use iceoryx2::prelude::*;

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

#[derive(Debug, Default, PlacementDefault, ZeroCopySend)]
#[type_name("ModbusIpcDataRx")]
#[repr(C)]
pub struct ModbusIpcDataRx {
    pub modbus_do_0: u32,
}

pub struct IpcData {
    pub modbus_ai_0: f32,
    pub modbus_di_0: u32,
    pub modbus_do_0: u32,
}