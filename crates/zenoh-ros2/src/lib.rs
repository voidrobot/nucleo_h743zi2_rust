//! # zenoh-ros2: Embedded Pure Rust ROS 2 Client via Zenoh
//!
//! 하드웨어 및 운영체제에 독립적인 `no_std`, Zero-allocation ROS 2 클라이언트 미들웨어 라이브러리.
//! ROS 2 Jazzy `rmw_zenoh_cpp` 와이어 프로토콜과 100% 호환된다.

#![no_std]
#![allow(dead_code)]

pub mod cdr;
pub mod publisher;
pub mod registry;
pub mod service;
pub mod subscriber;
pub mod traits;
pub mod types;
pub mod wire;

// 주요 심볼 최상위 재수출
pub use cdr::{CdrReader, CdrWriter, CDR_HEADER_LE};
pub use publisher::Publisher;
pub use registry::DiscoveryRegistry;
pub use service::ServiceServer;
pub use subscriber::Subscriber;
pub use traits::{RosMessage, RosService};
pub use wire::{msg_id, ZenohWire};
