//! # HTS221 온습도 센서 공장 출하 OTP 캘리브레이션 모듈
//!
//! HTS221 센서 내부 OTP 캘리브레이션 레지스터(0x30~0x3F)로부터 계수를 파싱하고,
//! 1차 선형 보간(Linear Interpolation)을 통해 정확한 상대습도(% rH)와 온도(°C)를 산출한다.

/// HTS221 공장 캘리브레이션 계수 구조체
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Hts221Calibration {
    // 습도 보간 파라미터
    pub h0_rh_x2: u8,
    pub h1_rh_x2: u8,
    pub h0_t0_out: i16,
    pub h1_t0_out: i16,

    // 온도 보간 파라미터 (1/8 °C 단위)
    pub t0_degc_x8: u16,
    pub t1_degc_x8: u16,
    pub t0_out: i16,
    pub t1_out: i16,
}

impl Hts221Calibration {
    /// HTS221의 0x30~0x3F 레지스터 16바이트 버퍼로부터 캘리브레이션 계수 파싱
    ///
    /// 버퍼 오프셋 기준:
    /// - 0x00 (0x30): H0_rH_x2
    /// - 0x01 (0x31): H1_rH_x2
    /// - 0x02 (0x32): T0_degC_x8 (하위 8비트)
    /// - 0x03 (0x33): T1_degC_x8 (하위 8비트)
    /// - 0x05 (0x35): T1/T0 MSB (bits 1:0 -> T0_msb, bits 3:2 -> T1_msb)
    /// - 0x06~0x07 (0x36~0x37): H0_T0_OUT (i16)
    /// - 0x0A~0x0B (0x3A~0x3B): H1_T0_OUT (i16)
    /// - 0x0C~0x0D (0x3C~0x3D): T0_OUT (i16)
    /// - 0x0E~0x0F (0x3E~0x3F): T1_OUT (i16)
    pub fn from_raw_registers(buf: &[u8; 16]) -> Self {
        let h0_rh_x2 = buf[0];
        let h1_rh_x2 = buf[1];

        let t0_lsb = buf[2] as u16;
        let t1_lsb = buf[3] as u16;
        let msb = buf[5];

        let t0_msb = (msb & 0x03) as u16;
        let t1_msb = ((msb & 0x0C) >> 2) as u16;

        let t0_degc_x8 = (t0_msb << 8) | t0_lsb;
        let t1_degc_x8 = (t1_msb << 8) | t1_lsb;

        let h0_t0_out = i16::from_le_bytes([buf[6], buf[7]]);
        let h1_t0_out = i16::from_le_bytes([buf[10], buf[11]]);
        let t0_out = i16::from_le_bytes([buf[12], buf[13]]);
        let t1_out = i16::from_le_bytes([buf[14], buf[15]]);

        Self {
            h0_rh_x2,
            h1_rh_x2,
            h0_t0_out,
            h1_t0_out,
            t0_degc_x8,
            t1_degc_x8,
            t0_out,
            t1_out,
        }
    }

    /// 원시 온도 ADC 값(i16)을 1차 선형 보간하여 °C * 10 (소수점 1자리 정수)으로 변환
    pub fn compensate_temp_x10(&self, raw_temp: i16) -> i16 {
        let delta_out = (self.t1_out as i32) - (self.t0_out as i32);
        if delta_out == 0 {
            return 0;
        }

        // T1 - T0 (단위: 1/8 °C)
        let delta_t_x8 = (self.t1_degc_x8 as i32) - (self.t0_degc_x8 as i32);
        let diff_out = (raw_temp as i32) - (self.t0_out as i32);

        // temp_x8 = t0_x8 + (delta_t_x8 * diff_out) / delta_out
        let temp_x8 = (self.t0_degc_x8 as i32) + (delta_t_x8 * diff_out) / delta_out;

        // °C * 10 = (temp_x8 * 10) / 8
        ((temp_x8 * 10) / 8) as i16
    }

    /// 원시 온도 ADC 값(i16)을 °C 부동소수점(f32)으로 변환
    pub fn compensate_temp_f32(&self, raw_temp: i16) -> f32 {
        self.compensate_temp_x10(raw_temp) as f32 / 10.0
    }

    /// 원시 습도 ADC 값(i16)을 1차 선형 보간하여 % rH * 10 (소수점 1자리 정수, 0~1000)으로 변환
    pub fn compensate_humidity_x10(&self, raw_humidity: i16) -> u16 {
        let delta_out = (self.h1_t0_out as i32) - (self.h0_t0_out as i32);
        if delta_out == 0 {
            return 0;
        }

        // H1 - H0 (단위: 1/2 % rH)
        let delta_h_x2 = (self.h1_rh_x2 as i32) - (self.h0_rh_x2 as i32);
        let diff_out = (raw_humidity as i32) - (self.h0_t0_out as i32);

        // hum_x2 = h0_x2 + (delta_h_x2 * diff_out) / delta_out
        let hum_x2 = (self.h0_rh_x2 as i32) + (delta_h_x2 * diff_out) / delta_out;

        // % rH * 10 = (hum_x2 * 10) / 2 = hum_x2 * 5
        let hum_x10 = (hum_x2 * 5).clamp(0, 1000);
        hum_x10 as u16
    }

    /// 원시 습도 ADC 값(i16)을 % rH 부동소수점(f32, 0.0~100.0)으로 변환
    pub fn compensate_humidity_f32(&self, raw_humidity: i16) -> f32 {
        self.compensate_humidity_x10(raw_humidity) as f32 / 10.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hts221_calibration_parsing_and_interpolation() {
        // 임의의 정규 공장 출하 레지스터 목업 데이터
        // H0_rH_x2 = 60 (30.0% rH), H1_rH_x2 = 140 (70.0% rH)
        // T0_degC_x8 = 200 (25.0 °C), T1_degC_x8 = 320 (40.0 °C)
        let mut buf = [0u8; 16];
        buf[0] = 60; // H0_rH_x2
        buf[1] = 140; // H1_rH_x2
        buf[2] = 200; // T0_degC LSB
        buf[3] = 64; // T1_degC LSB (320 = 1 << 8 + 64)
        buf[5] = 0x04; // T1 MSB = 1 (bits 3:2 = 01b -> 0x04)

        // H0_T0_OUT = 1000, H1_T0_OUT = 5000
        let h0_out: i16 = 1000;
        let h1_out: i16 = 5000;
        buf[6] = (h0_out & 0xFF) as u8;
        buf[7] = ((h0_out >> 8) & 0xFF) as u8;
        buf[10] = (h1_out & 0xFF) as u8;
        buf[11] = ((h1_out >> 8) & 0xFF) as u8;

        // T0_OUT = 500, T1_OUT = 2500
        let t0_out: i16 = 500;
        let t1_out: i16 = 2500;
        buf[12] = (t0_out & 0xFF) as u8;
        buf[13] = ((t0_out >> 8) & 0xFF) as u8;
        buf[14] = (t1_out & 0xFF) as u8;
        buf[15] = ((t1_out >> 8) & 0xFF) as u8;

        let calib = Hts221Calibration::from_raw_registers(&buf);

        assert_eq!(calib.h0_rh_x2, 60);
        assert_eq!(calib.h1_rh_x2, 140);
        assert_eq!(calib.t0_degc_x8, 200);
        assert_eq!(calib.t1_degc_x8, 320);

        // 기준점 1 (T0 지점): 500 -> 25.0 °C (250)
        assert_eq!(calib.compensate_temp_x10(500), 250);
        // 기준점 2 (T1 지점): 2500 -> 40.0 °C (400)
        assert_eq!(calib.compensate_temp_x10(2500), 400);
        // 중간점 (T0 + (T1-T0)/2): 1500 -> 32.5 °C (325)
        assert_eq!(calib.compensate_temp_x10(1500), 325);

        // 습도 기준점 1 (H0 지점): 1000 -> 30.0% rH (300)
        assert_eq!(calib.compensate_humidity_x10(1000), 300);
        // 습도 기준점 2 (H1 지점): 5000 -> 70.0% rH (700)
        assert_eq!(calib.compensate_humidity_x10(5000), 700);
        // 습도 중간점: 3000 -> 50.0% rH (500)
        assert_eq!(calib.compensate_humidity_x10(3000), 500);
    }
}
