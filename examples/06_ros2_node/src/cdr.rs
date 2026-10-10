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

/// ROS 2 텔레메트리 CDR 인코더 모음
pub struct Ros2Cdr;

impl Ros2Cdr {
    /// `sensor_msgs/msg/Imu` 인코딩 (100 Hz)
    /// ROS 2 표준 쿼터니언 순서: (x, y, z, w)
    pub fn encode_imu(
        buf: &mut [u8],
        sec: i32,
        nanosec: u32,
        quat_xyzw: [f64; 4],
        cov_orient: &[f64; 9],
        ang_vel: [f64; 3],
        cov_ang_vel: &[f64; 9],
        linear_accel: [f64; 3],
        cov_linear_accel: &[f64; 9],
    ) -> usize {
        let mut writer = CdrWriter::new(buf);
        writer.write_header(sec, nanosec, "imu_link");

        // orientation (geometry_msgs/msg/Quaternion: x, y, z, w)
        writer.write_f64(quat_xyzw[0]);
        writer.write_f64(quat_xyzw[1]);
        writer.write_f64(quat_xyzw[2]);
        writer.write_f64(quat_xyzw[3]);
        writer.write_f64_array(cov_orient);

        // angular_velocity (geometry_msgs/msg/Vector3: x, y, z)
        writer.write_f64(ang_vel[0]);
        writer.write_f64(ang_vel[1]);
        writer.write_f64(ang_vel[2]);
        writer.write_f64_array(cov_ang_vel);

        // linear_acceleration (geometry_msgs/msg/Vector3: x, y, z)
        writer.write_f64(linear_accel[0]);
        writer.write_f64(linear_accel[1]);
        writer.write_f64(linear_accel[2]);
        writer.write_f64_array(cov_linear_accel);

        writer.position()
    }

    /// `sensor_msgs/msg/MagneticField` 인코딩 (10 Hz)
    pub fn encode_mag(
        buf: &mut [u8],
        sec: i32,
        nanosec: u32,
        mag_tesla: [f64; 3],
        covariance: &[f64; 9],
    ) -> usize {
        let mut writer = CdrWriter::new(buf);
        writer.write_header(sec, nanosec, "imu_link");
        writer.write_f64(mag_tesla[0]);
        writer.write_f64(mag_tesla[1]);
        writer.write_f64(mag_tesla[2]);
        writer.write_f64_array(covariance);
        writer.position()
    }

    /// `sensor_msgs/msg/FluidPressure` 인코딩 (1 Hz)
    pub fn encode_pressure(
        buf: &mut [u8],
        sec: i32,
        nanosec: u32,
        pressure_pa: f64,
        variance: f64,
    ) -> usize {
        let mut writer = CdrWriter::new(buf);
        writer.write_header(sec, nanosec, "imu_link");
        writer.write_f64(pressure_pa);
        writer.write_f64(variance);
        writer.position()
    }

    /// `sensor_msgs/msg/Temperature` 인코딩 (1 Hz)
    pub fn encode_temperature(
        buf: &mut [u8],
        sec: i32,
        nanosec: u32,
        temperature_c: f64,
        variance: f64,
    ) -> usize {
        let mut writer = CdrWriter::new(buf);
        writer.write_header(sec, nanosec, "imu_link");
        writer.write_f64(temperature_c);
        writer.write_f64(variance);
        writer.position()
    }

    /// `sensor_msgs/msg/RelativeHumidity` 인코딩 (1 Hz)
    pub fn encode_humidity(
        buf: &mut [u8],
        sec: i32,
        nanosec: u32,
        humidity_ratio: f64,
        variance: f64,
    ) -> usize {
        let mut writer = CdrWriter::new(buf);
        writer.write_header(sec, nanosec, "imu_link");
        writer.write_f64(humidity_ratio);
        writer.write_f64(variance);
        writer.position()
    }

    /// `example_interfaces/srv/SetBool` 응답 인코딩
    pub fn encode_set_bool_response(buf: &mut [u8], success: bool, message: &str) -> usize {
        let mut writer = CdrWriter::new(buf);
        writer.write_bool(success);
        writer.write_string(message);
        writer.position()
    }

    /// `geometry_msgs/msg/Twist` 디코딩
    pub fn decode_twist(buf: &[u8]) -> Option<([f64; 3], [f64; 3])> {
        let mut reader = CdrReader::new(buf)?;
        let lx = reader.read_f64()?;
        let ly = reader.read_f64()?;
        let lz = reader.read_f64()?;
        let ax = reader.read_f64()?;
        let ay = reader.read_f64()?;
        let az = reader.read_f64()?;
        Some(([lx, ly, lz], [ax, ay, az]))
    }

    /// `example_interfaces/srv/SetBool` 요청 디코딩
    pub fn decode_set_bool_request(buf: &[u8]) -> Option<bool> {
        let mut reader = CdrReader::new(buf)?;
        reader.read_bool()
    }
}
