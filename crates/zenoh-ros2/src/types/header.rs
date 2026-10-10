//! # `std_msgs/msg/Header` (types/header.rs)

use crate::cdr::CdrWriter;

#[derive(Copy, Clone, Debug, Default, PartialEq)]
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
