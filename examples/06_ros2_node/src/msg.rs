//! # ROS 2 Data Objects & Topic Descriptors (msg.rs)
//!
//! ROS 2 표준 메시지 데이터 오브젝트(Imu, Twist, MagneticField 등)와
//! Zenoh 와이어 프로토콜용 토픽 메타데이터(Key, DDS Type Hash, Liveliness Token)를
//! 일원화하여 관리하는 SSOT 모듈이다.

#![allow(dead_code)]

use crate::cdr::{CdrReader, CdrWriter};

/// `std_msgs/msg/Header`
#[derive(Copy, Clone, Debug, Default)]
pub struct Header {
    pub sec: i32,
    pub nanosec: u32,
    pub frame_id: &'static str,
}

impl Header {
    pub const fn new(sec: i32, nanosec: u32, frame_id: &'static str) -> Self {
        Self { sec, nanosec, frame_id }
    }

    pub fn write_cdr(&self, writer: &mut CdrWriter) {
        writer.write_header(self.sec, self.nanosec, self.frame_id);
    }
}

/// 3차원 기하 벡터 (`geometry_msgs/msg/Vector3`)
#[derive(Copy, Clone, Debug, Default)]
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
#[derive(Copy, Clone, Debug, Default)]
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
#[derive(Copy, Clone, Debug, Default)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

impl Twist {
    pub const fn new(linear: Vector3, angular: Vector3) -> Self {
        Self { linear, angular }
    }

    pub fn decode_cdr(buf: &[u8]) -> Option<Self> {
        let mut reader = CdrReader::new(buf)?;
        let linear = Vector3::read_cdr(&mut reader)?;
        let angular = Vector3::read_cdr(&mut reader)?;
        Some(Self { linear, angular })
    }
}

/// `sensor_msgs/msg/Imu`
#[derive(Copy, Clone, Debug)]
pub struct Imu {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}

impl Imu {
    pub fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        let mut writer = CdrWriter::new(buf);
        self.header.write_cdr(&mut writer);
        self.orientation.write_cdr(&mut writer);
        writer.write_f64_array(&self.orientation_covariance);
        self.angular_velocity.write_cdr(&mut writer);
        writer.write_f64_array(&self.angular_velocity_covariance);
        self.linear_acceleration.write_cdr(&mut writer);
        writer.write_f64_array(&self.linear_acceleration_covariance);
        writer.position()
    }
}

/// `sensor_msgs/msg/MagneticField`
#[derive(Copy, Clone, Debug)]
pub struct MagneticField {
    pub header: Header,
    pub magnetic_field: Vector3,
    pub magnetic_field_covariance: [f64; 9],
}

impl MagneticField {
    pub const fn new(header: Header, magnetic_field: Vector3, magnetic_field_covariance: [f64; 9]) -> Self {
        Self { header, magnetic_field, magnetic_field_covariance }
    }

    pub fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        let mut writer = CdrWriter::new(buf);
        self.header.write_cdr(&mut writer);
        self.magnetic_field.write_cdr(&mut writer);
        writer.write_f64_array(&self.magnetic_field_covariance);
        writer.position()
    }
}

/// `sensor_msgs/msg/FluidPressure`
#[derive(Copy, Clone, Debug)]
pub struct FluidPressure {
    pub header: Header,
    pub fluid_pressure: f64,
    pub variance: f64,
}

impl FluidPressure {
    pub const fn new(header: Header, fluid_pressure: f64, variance: f64) -> Self {
        Self { header, fluid_pressure, variance }
    }

    pub fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        let mut writer = CdrWriter::new(buf);
        self.header.write_cdr(&mut writer);
        writer.write_f64(self.fluid_pressure);
        writer.write_f64(self.variance);
        writer.position()
    }
}

/// `sensor_msgs/msg/Temperature`
#[derive(Copy, Clone, Debug)]
pub struct Temperature {
    pub header: Header,
    pub temperature: f64,
    pub variance: f64,
}

impl Temperature {
    pub const fn new(header: Header, temperature: f64, variance: f64) -> Self {
        Self { header, temperature, variance }
    }

    pub fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        let mut writer = CdrWriter::new(buf);
        self.header.write_cdr(&mut writer);
        writer.write_f64(self.temperature);
        writer.write_f64(self.variance);
        writer.position()
    }
}

/// `sensor_msgs/msg/RelativeHumidity`
#[derive(Copy, Clone, Debug)]
pub struct RelativeHumidity {
    pub header: Header,
    pub relative_humidity: f64,
    pub variance: f64,
}

impl RelativeHumidity {
    pub const fn new(header: Header, relative_humidity: f64, variance: f64) -> Self {
        Self { header, relative_humidity, variance }
    }

    pub fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        let mut writer = CdrWriter::new(buf);
        self.header.write_cdr(&mut writer);
        writer.write_f64(self.relative_humidity);
        writer.write_f64(self.variance);
        writer.position()
    }
}

// ----------------------------------------------------------------------------
// Topic Endpoint Metadata (SSOT)
// ----------------------------------------------------------------------------
pub mod endpoints {
    pub const KEY_IMU_DATA: &str =
        "0/nucleo/imu/data/sensor_msgs::msg::dds_::Imu_/RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b";

    pub const KEY_IMU_MAG: &str =
        "0/nucleo/imu/mag/sensor_msgs::msg::dds_::MagneticField_/RIHS01_e80f32f56a20486c9923008fc1a1db07bbb273cbbf6a5b3bfa00835ee00e4dff";

    pub const KEY_PRESSURE: &str =
        "0/nucleo/pressure/sensor_msgs::msg::dds_::FluidPressure_/RIHS01_22dfb2b145a0bd5a31a1ac3882a1b32148b51d9b2f3bab250290d66f3595bc32";

    pub const KEY_TEMPERATURE: &str =
        "0/nucleo/temperature/sensor_msgs::msg::dds_::Temperature_/RIHS01_72514a14126ab9f8a9abec974c78e5610a367b59db5da355ff1fb982d5bad4b8";

    pub const KEY_HUMIDITY: &str =
        "0/nucleo/humidity/sensor_msgs::msg::dds_::RelativeHumidity_/RIHS01_8687c99b4fb393cb2e545e407b5ea7fd0b5d8960bcd849a0f86c544740138839";

    pub const KEY_CMD_VEL: &str =
        "0/nucleo/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a";

    pub const TOKEN_CMD_VEL: &str =
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/16/MS/%/%/nucleo_h743zi2/%nucleo%cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a/::,:,:,:,,";

    pub const TOPIC_LIVELINESS_TOKENS: [&str; 7] = [
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/0/NN/%/%/nucleo_h743zi2",
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/11/MP/%/%/nucleo_h743zi2/%nucleo%imu%data/sensor_msgs::msg::dds_::Imu_/RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b/::,:,:,:,,",
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/12/MP/%/%/nucleo_h743zi2/%nucleo%imu%mag/sensor_msgs::msg::dds_::MagneticField_/RIHS01_e80f32f56a20486c9923008fc1a1db07bbb273cbbf6a5b3bfa00835ee00e4dff/::,:,:,:,,",
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/13/MP/%/%/nucleo_h743zi2/%nucleo%pressure/sensor_msgs::msg::dds_::FluidPressure_/RIHS01_22dfb2b145a0bd5a31a1ac3882a1b32148b51d9b2f3bab250290d66f3595bc32/::,:,:,:,,",
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/14/MP/%/%/nucleo_h743zi2/%nucleo%temperature/sensor_msgs::msg::dds_::Temperature_/RIHS01_72514a14126ab9f8a9abec974c78e5610a367b59db5da355ff1fb982d5bad4b8/::,:,:,:,,",
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/15/MP/%/%/nucleo_h743zi2/%nucleo%humidity/sensor_msgs::msg::dds_::RelativeHumidity_/RIHS01_8687c99b4fb393cb2e545e407b5ea7fd0b5d8960bcd849a0f86c544740138839/::,:,:,:,,",
        TOKEN_CMD_VEL,
    ];
}
