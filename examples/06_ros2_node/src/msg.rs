//! # ROS 2 Data Objects & Topic Descriptors (msg.rs)
//!
//! NUCLEO 센서 전용 ROS 2 데이터 오브젝트(Imu, MagneticField 등)와
//! Zenoh 와이어 프로토콜용 토픽 메타데이터(Key, DDS Type Hash, Liveliness Token)를 관리한다.

#![allow(dead_code)]

use zenoh_ros2::{CdrWriter, RosMessage};
pub use zenoh_ros2::types::{Header, Quaternion, Twist, Vector3};

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
    pub const TOKEN_NODE_NAME: &str =
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/0/NN/%/%/nucleo_h743zi2";

    pub const KEY_IMU_DATA: &str =
        "0/nucleo/imu/data/sensor_msgs::msg::dds_::Imu_/RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b";
    pub const TOKEN_IMU_DATA: &str =
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/11/MP/%/%/nucleo_h743zi2/%nucleo%imu%data/sensor_msgs::msg::dds_::Imu_/RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b/::,:,:,:,,";

    pub const KEY_IMU_MAG: &str =
        "0/nucleo/imu/mag/sensor_msgs::msg::dds_::MagneticField_/RIHS01_e80f32f56a20486c9923008fc1a1db07bbb273cbbf6a5b3bfa00835ee00e4dff";
    pub const TOKEN_IMU_MAG: &str =
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/12/MP/%/%/nucleo_h743zi2/%nucleo%imu%mag/sensor_msgs::msg::dds_::MagneticField_/RIHS01_e80f32f56a20486c9923008fc1a1db07bbb273cbbf6a5b3bfa00835ee00e4dff/::,:,:,:,,";

    pub const KEY_PRESSURE: &str =
        "0/nucleo/pressure/sensor_msgs::msg::dds_::FluidPressure_/RIHS01_22dfb2b145a0bd5a31a1ac3882a1b32148b51d9b2f3bab250290d66f3595bc32";
    pub const TOKEN_PRESSURE: &str =
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/13/MP/%/%/nucleo_h743zi2/%nucleo%pressure/sensor_msgs::msg::dds_::FluidPressure_/RIHS01_22dfb2b145a0bd5a31a1ac3882a1b32148b51d9b2f3bab250290d66f3595bc32/::,:,:,:,,";

    pub const KEY_TEMPERATURE: &str =
        "0/nucleo/temperature/sensor_msgs::msg::dds_::Temperature_/RIHS01_72514a14126ab9f8a9abec974c78e5610a367b59db5da355ff1fb982d5bad4b8";
    pub const TOKEN_TEMPERATURE: &str =
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/14/MP/%/%/nucleo_h743zi2/%nucleo%temperature/sensor_msgs::msg::dds_::Temperature_/RIHS01_72514a14126ab9f8a9abec974c78e5610a367b59db5da355ff1fb982d5bad4b8/::,:,:,:,,";

    pub const KEY_HUMIDITY: &str =
        "0/nucleo/humidity/sensor_msgs::msg::dds_::RelativeHumidity_/RIHS01_473fa732e600572e9d2243d6860d5bfa4d76241a4980a3ee67bf841eb1d7f35b";
    pub const TOKEN_HUMIDITY: &str =
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/15/MP/%/%/nucleo_h743zi2/%nucleo%humidity/sensor_msgs::msg::dds_::RelativeHumidity_/RIHS01_473fa732e600572e9d2243d6860d5bfa4d76241a4980a3ee67bf841eb1d7f35b/::,:,:,:,,";

    pub const KEY_CMD_VEL: &str =
        "0/nucleo/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a";
    pub const TOKEN_CMD_VEL: &str =
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/16/MS/%/%/nucleo_h743zi2/%nucleo%cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a/::,:,:,:,,";

    pub const TOPIC_LIVELINESS_TOKENS: [&str; 7] = [
        TOKEN_NODE_NAME,
        TOKEN_IMU_DATA,
        TOKEN_IMU_MAG,
        TOKEN_PRESSURE,
        TOKEN_TEMPERATURE,
        TOKEN_HUMIDITY,
        TOKEN_CMD_VEL,
    ];
}

impl RosMessage for Imu {
    const TOPIC_KEY: &'static str = endpoints::KEY_IMU_DATA;
    const LIVELINESS_TOKEN: &'static str = endpoints::TOKEN_IMU_DATA;

    fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        self.encode_cdr(buf)
    }
}

impl RosMessage for MagneticField {
    const TOPIC_KEY: &'static str = endpoints::KEY_IMU_MAG;
    const LIVELINESS_TOKEN: &'static str = endpoints::TOKEN_IMU_MAG;

    fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        self.encode_cdr(buf)
    }
}

impl RosMessage for FluidPressure {
    const TOPIC_KEY: &'static str = endpoints::KEY_PRESSURE;
    const LIVELINESS_TOKEN: &'static str = endpoints::TOKEN_PRESSURE;

    fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        self.encode_cdr(buf)
    }
}

impl RosMessage for Temperature {
    const TOPIC_KEY: &'static str = endpoints::KEY_TEMPERATURE;
    const LIVELINESS_TOKEN: &'static str = endpoints::TOKEN_TEMPERATURE;

    fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        self.encode_cdr(buf)
    }
}

impl RosMessage for RelativeHumidity {
    const TOPIC_KEY: &'static str = endpoints::KEY_HUMIDITY;
    const LIVELINESS_TOKEN: &'static str = endpoints::TOKEN_HUMIDITY;

    fn encode_cdr(&self, buf: &mut [u8]) -> usize {
        self.encode_cdr(buf)
    }
}

