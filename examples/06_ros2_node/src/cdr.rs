//! # no_std ROS 2 정규 CDR(Common Data Representation) 직렬화기 및 역직렬화기
//!
//! ROS 2의 표준 CDR 리틀 엔디안(CDR Little-Endian) 규격에 맞추어
//! 힙 동적 할당 없이(Zero-Heap Allocation) 고정 바이트 슬라이스에 고속으로
//! 인코딩 및 디코딩을 수행한다.


/// CDR 리틀 엔디안 인코딩 헤더 (버전 1, Little Endian, Flags 0)
pub const CDR_HEADER_LE: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// 고정 버퍼 기반 CDR 라이터
pub struct CdrWriter<'a> {
    buf: &'a mut [u8],
    offset: usize,
}

impl<'a> CdrWriter<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        let mut writer = Self { buf, offset: 0 };
        writer.write_bytes(&CDR_HEADER_LE);
        writer
    }

    pub fn position(&self) -> usize {
        self.offset
    }

    pub fn align(&mut self, alignment: usize) {
        // CDR 정렬은 CDR 헤더 4바이트 이후의 상대 오프셋 기준 정렬
        let relative_offset = self.offset - 4;
        let remainder = relative_offset % alignment;
        if remainder != 0 {
            let padding = alignment - remainder;
            for _ in 0..padding {
                self.write_u8(0);
            }
        }
    }

    pub fn write_u8(&mut self, val: u8) {
        if self.offset < self.buf.len() {
            self.buf[self.offset] = val;
            self.offset += 1;
        }
    }

    pub fn write_bool(&mut self, val: bool) {
        self.write_u8(if val { 1 } else { 0 });
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) {
        let len = bytes.len();
        if self.offset + len <= self.buf.len() {
            self.buf[self.offset..self.offset + len].copy_from_slice(bytes);
            self.offset += len;
        }
    }

    pub fn write_i32(&mut self, val: i32) {
        self.align(4);
        self.write_bytes(&val.to_le_bytes());
    }

    pub fn write_u32(&mut self, val: u32) {
        self.align(4);
        self.write_bytes(&val.to_le_bytes());
    }

    pub fn write_f64(&mut self, val: f64) {
        self.align(8);
        self.write_bytes(&val.to_le_bytes());
    }

    pub fn write_f64_array(&mut self, arr: &[f64]) {
        for &val in arr {
            self.write_f64(val);
        }
    }

    /// null 종단을 포함하는 ROS 2 문자열 작성
    pub fn write_string(&mut self, s: &str) {
        let len_with_null = (s.len() + 1) as u32;
        self.write_u32(len_with_null);
        self.write_bytes(s.as_bytes());
        self.write_u8(0x00); // Null terminator
    }

    /// std_msgs/msg/Header 직렬화
    pub fn write_header(&mut self, sec: i32, nanosec: u32, frame_id: &str) {
        self.write_i32(sec);
        self.write_u32(nanosec);
        self.write_string(frame_id);
    }
}

/// 고정 버퍼 기반 CDR 리더
pub struct CdrReader<'a> {
    buf: &'a [u8],
    offset: usize,
}

impl<'a> CdrReader<'a> {
    pub fn new(buf: &'a [u8]) -> Option<Self> {
        if buf.len() < 4 || buf[0..4] != CDR_HEADER_LE {
            return None;
        }
        Some(Self { buf, offset: 4 })
    }

    pub fn align(&mut self, alignment: usize) {
        let relative_offset = self.offset - 4;
        let remainder = relative_offset % alignment;
        if remainder != 0 {
            self.offset += alignment - remainder;
        }
    }

    pub fn read_u8(&mut self) -> Option<u8> {
        if self.offset < self.buf.len() {
            let val = self.buf[self.offset];
            self.offset += 1;
            Some(val)
        } else {
            None
        }
    }

    pub fn read_bool(&mut self) -> Option<bool> {
        self.read_u8().map(|b| b != 0)
    }

    pub fn read_f64(&mut self) -> Option<f64> {
        self.align(8);
        if self.offset + 8 <= self.buf.len() {
            let bytes: [u8; 8] = self.buf[self.offset..self.offset + 8].try_into().ok()?;
            self.offset += 8;
            Some(f64::from_le_bytes(bytes))
        } else {
            None
        }
    }
}


