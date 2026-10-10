//! # Standard Service Data Types (types/srv.rs)

use crate::cdr::{CdrReader, CdrWriter};

/// `example_interfaces/srv/SetBool`
pub mod set_bool {
    use super::*;

    /// 서비스 요청 (Request)
    #[derive(Copy, Clone, Debug, Default, PartialEq)]
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
    #[derive(Copy, Clone, Debug, PartialEq)]
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
