// this file should be a carbon copy in both ./opcua/src/ and ./plc/src/
use bytemuck::{Pod, Zeroable};
use std::{mem, fs::File};
use memmap2::MmapMut;

pub const SHM_PATH: &str = "/dev/shm/shared_plc_data";

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)] // Plain Old Data; zeroed bytes are valid
pub struct SharedData {
    pub temperature: f32,
    pub humidity: f32,
    pub status: u32,
    pub area_1_lights: u32,
    pub area_2_lights: u32,
}

pub fn map_shared_memory(file: &File) -> memmap2::MmapMut {
    unsafe { MmapMut::map_mut(file).expect("Failed to mmap") } // unsafe because of potential UB if file is modified
}

pub fn read_data(mmap: &memmap2::MmapMut) -> SharedData {
    bytemuck::from_bytes::<SharedData>(&mmap[..mem::size_of::<SharedData>()]).clone()
}

pub fn write_data(mmap: &mut memmap2::MmapMut, data: SharedData) {
    let bytes = bytemuck::bytes_of(&data);
    mmap[..bytes.len()].copy_from_slice(bytes);
    mmap.flush().unwrap(); // make changes visible
}