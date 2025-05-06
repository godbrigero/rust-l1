use futures_util::Stream;
use nalgebra::Vector3;
use std::ffi::{CString, c_void};
use std::pin::Pin;
use std::task::{Context, Poll};

use crate::{
    LidarWorkingMode, MessageType, PointCloudUnitree, createUnitreeLidarReaderCpp, delete_reader,
    getCloud, getVersionOfFirmware, getVersionOfSDK, initialize, runParse, setLidarWorkingMode,
};

pub struct LidarReader {
    reader: *mut c_void,
}

impl LidarReader {
    pub fn new_with_initialize(
        port: &str,
        baudrate: u32,
        cloud_scan_num: u16,
        max_distance_meters: f32,
        min_distance_meters: f32,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let reader = unsafe { createUnitreeLidarReaderCpp() };
        if reader.is_null() {
            return Err("Failed to create LiDAR reader".into());
        }

        let port = CString::new(port)?;
        let result = unsafe {
            initialize(
                reader,
                cloud_scan_num,
                port.as_ptr(),
                baudrate,
                0.0,
                0.001,
                0.0,
                max_distance_meters as f32,
                min_distance_meters as f32,
            )
        };

        if result != 0 {
            return Err("Failed to initialize LiDAR".into());
        }

        Ok(Self { reader })
    }

    pub fn into_stream(self) -> LidarStream {
        LidarStream { reader: self }
    }

    pub fn run_parse(&self) -> MessageType {
        unsafe { runParse(self.reader) }
    }

    pub fn get_cloud(&self) -> PointCloudUnitree {
        let mut cloud = PointCloudUnitree::default();
        unsafe { getCloud(self.reader, &mut cloud) };
        cloud
    }

    pub fn get_version_of_firmware(&self) -> [u8; 128] {
        let mut buffer = [0; 128];
        unsafe { getVersionOfFirmware(self.reader, buffer.as_mut_ptr() as *mut i8, buffer.len()) };
        buffer
    }

    pub fn get_version_of_sdk(&self) -> [u8; 128] {
        let mut buffer = [0; 128];
        unsafe { getVersionOfSDK(self.reader, buffer.as_mut_ptr() as *mut i8, buffer.len()) };
        buffer
    }

    pub fn set_lidar_working_mode(&self, mode: LidarWorkingMode) {
        unsafe { setLidarWorkingMode(self.reader, mode) };
    }
}

pub struct LidarStream {
    reader: LidarReader,
}

impl Stream for LidarStream {
    type Item = Vec<Vector3<f64>>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        unsafe {
            match runParse(self.reader.reader) {
                MessageType::POINTCLOUD => {
                    let mut cloud = PointCloudUnitree::default();
                    getCloud(self.reader.reader, &mut cloud);
                    let points = cloud.points();
                    if points.is_empty() {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    } else {
                        Poll::Ready(Some(
                            points
                                .iter()
                                .map(|p| Vector3::new(p.x as f64, p.y as f64, p.z as f64))
                                .collect(),
                        ))
                    }
                }
                _ => {
                    // If no data, wake up after a short delay
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            }
        }
    }
}

impl Drop for LidarReader {
    fn drop(&mut self) {
        unsafe {
            delete_reader(self.reader);
        }
    }
}
