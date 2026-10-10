//! # `geometry_msgs` Data Types (types/geometry.rs)

use crate::cdr::{CdrReader, CdrWriter};

/// 3차원 기하 벡터 (`geometry_msgs/msg/Vector3`)
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn write_cdr(&self, writer: &mut CdrWriter) {
        writer.write_f64(self.x);
        writer.write_f64(self.y);
        writer.write_f64(self.z);
    }

    pub fn read_cdr(reader: &mut CdrReader) -> Option<Self> {
        let x = reader.read_f64()?;
        let y = reader.read_f64()?;
        let z = reader.read_f64()?;
        Some(Self { x, y, z })
    }
}

/// 4원수 자세 (`geometry_msgs/msg/Quaternion`)
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Quaternion {
    pub const fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    pub fn write_cdr(&self, writer: &mut CdrWriter) {
        writer.write_f64(self.x);
        writer.write_f64(self.y);
        writer.write_f64(self.z);
        writer.write_f64(self.w);
    }
}

/// `geometry_msgs/msg/Twist`
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

impl Twist {
    pub const fn new(linear: Vector3, angular: Vector3) -> Self {
        Self { linear, angular }
    }

    pub fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        let mut writer = CdrWriter::new(buf);
        self.linear.write_cdr(&mut writer);
        self.angular.write_cdr(&mut writer);
        writer.position()
    }

    pub fn decode_cdr(buf: &[u8]) -> Option<Self> {
        let mut reader = CdrReader::new(buf)?;
        let linear = Vector3::read_cdr(&mut reader)?;
        let angular = Vector3::read_cdr(&mut reader)?;
        Some(Self { linear, angular })
    }
}

impl crate::traits::RosMessage for Twist {
    const TOPIC_KEY: &'static str =
        "0/nucleo/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a";
    const LIVELINESS_TOKEN: &'static str =
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/16/MS/%/%/nucleo_h743zi2/%nucleo%cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a/::,:,:,:,,";

    fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        self.encode_cdr(buf)
    }

    fn decode_cdr(buf: &[u8]) -> Option<Self> {
        Self::decode_cdr(buf)
    }
}
