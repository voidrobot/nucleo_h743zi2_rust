//! # ROS 2 Service Objects & Descriptors (srv.rs)
//!
//! example_interfaces/srv/SetBool 등 ROS 2 서비스 Request/Response 객체 및
//! Zenoh Queryable 서비스 엔드포인트 SSOT 메타데이터 정의.

#![allow(dead_code)]

pub use zenoh_ros2::types::srv::set_bool;
use zenoh_ros2::RosService;

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

pub struct SetBoolService;

impl RosService for SetBoolService {
    type Request = set_bool::Request;
    type Response = set_bool::Response;

    const SERVICE_KEY: &'static str = endpoints::KEY_SET_LED;
    const LIVELINESS_TOKEN: &'static str = endpoints::TOKEN_SET_LED;

    fn decode_request(buf: &[u8]) -> Option<Self::Request> {
        set_bool::Request::decode_cdr(buf)
    }

    fn encode_response(res: &Self::Response, buf: &mut [u8]) -> usize {
        res.encode_cdr(buf)
    }
}
