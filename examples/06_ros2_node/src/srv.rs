//! # ROS 2 Service Objects & Descriptors (srv.rs)
//!
//! example_interfaces/srv/SetBool 등 ROS 2 서비스 Request/Response 객체 및
//! Zenoh Queryable 서비스 엔드포인트 SSOT 메타데이터 정의.

#![allow(dead_code)]

use crate::cdr::{CdrReader, CdrWriter};

/// `example_interfaces/srv/SetBool`
pub mod set_bool {
    use super::*;

    /// 서비스 요청 (Request)
    #[derive(Copy, Clone, Debug)]
    pub struct Request {
        pub data: bool,
    }

    impl Request {
        pub const fn new(data: bool) -> Self {
            Self { data }
        }

        pub fn decode_cdr(buf: &[u8]) -> Option<Self> {
            let mut reader = CdrReader::new(buf)?;
            let data = reader.read_bool()?;
            Some(Self { data })
        }
    }

    /// 서비스 응답 (Response)
    #[derive(Copy, Clone, Debug)]
    pub struct Response {
        pub success: bool,
        pub message: &'static str,
    }

    impl Response {
        pub const fn new(success: bool, message: &'static str) -> Self {
            Self { success, message }
        }

        pub fn encode_cdr(&self, buf: &mut [u8]) -> usize {
            let mut writer = CdrWriter::new(buf);
            writer.write_bool(self.success);
            writer.write_string(self.message);
            writer.position()
        }
    }
}

// ----------------------------------------------------------------------------
// Service Endpoint Metadata (SSOT)
// ----------------------------------------------------------------------------
pub mod endpoints {
    /// `/nucleo/set_led` Zenoh 1.0 Queryable 키 (example_interfaces/srv/SetBool)
    pub const KEY_SET_LED: &str =
        "0/nucleo/set_led/example_interfaces::srv::dds_::SetBool_/RIHS01_a69782e5631b12e15c8e218410de1685bbf13e382718295adad14037a24afbe8";

    /// `/nucleo/set_led` ROS 2 Jazzy Liveliness Token (Service Server: SS)
    pub const TOKEN_SET_LED: &str =
        "@ros2_lv/0/100f0e0d0c0b0a090807060504030201/0/21/SS/%/%/nucleo_h743zi2/%nucleo%set_led/example_interfaces::srv::dds_::SetBool_/RIHS01_a69782e5631b12e15c8e218410de1685bbf13e382718295adad14037a24afbe8/::,:,:,:,,";
}
