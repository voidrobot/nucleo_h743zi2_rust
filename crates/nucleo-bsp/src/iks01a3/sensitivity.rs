//! # X-NUCLEO-IKS01A3 센서 물리량 감도(Sensitivity) 및 환산 계수 모듈
//!
//! 각 센서 데이터시트에 명시된 LSB당 물리 단위 환산 계수를 상수로 정의하고,
//! 원시 ADC 레지스터 정수값으로부터 공학 단위(mg, dps, mgauss, hPa, °C)로의
//! 변환 함수를 제공한다.

/// LSM6DSO 6축 고정밀 IMU 감도 계수
pub mod lsm6dso {
    /// ±2g 풀스케일 가속도계 감도: 0.061 mg/LSB
    pub const ACCEL_MG_PER_LSB: f32 = 0.061;

    /// ±250 dps 풀스케일 자이로스코프 감도: 8.75 mdps/LSB = 0.00875 dps/LSB
    pub const GYRO_DPS_PER_LSB: f32 = 0.00875;

    /// 원시 가속도 ADC 값 -> 밀리지(mg) 단위 변환 (정수 연산)
    #[inline]
    pub fn raw_to_mg(raw: i16) -> i16 {
        ((raw as i32 * 61) / 1000) as i16
    }

    /// 원시 가속도 ADC 값 -> 밀리지(mg) 단위 변환 (f32)
    #[inline]
    pub fn raw_to_mg_f32(raw: i16) -> f32 {
        raw as f32 * ACCEL_MG_PER_LSB
    }

    /// 원시 가속도 ADC 값 -> SI 단위 가속도 (m/s^2) 고정밀 변환 (f32)
    #[inline]
    pub fn raw_to_mps2_f32(raw: i16) -> f32 {
        raw as f32 * (ACCEL_MG_PER_LSB * 0.001 * super::constants::STANDARD_GRAVITY)
    }

    /// 원시 각속도 ADC 값 -> 초당 각도(dps) 단위 변환 (정수 연산)
    #[inline]
    pub fn raw_to_dps(raw: i16) -> i16 {
        ((raw as i32 * 875) / 100000) as i16
    }

    /// 원시 각속도 ADC 값 -> 초당 각도(dps) 단위 변환 (f32)
    #[inline]
    pub fn raw_to_dps_f32(raw: i16) -> f32 {
        raw as f32 * GYRO_DPS_PER_LSB
    }
}

/// LIS2DW12 보조 3축 가속도계 감도 계수
pub mod lis2dw12 {
    /// 14-bit High-Performance ±2g 감도: 0.244 mg/LSB
    pub const ACCEL_MG_PER_LSB: f32 = 0.244;

    /// 14비트 좌측 정렬 16비트 레지스터 -> 밀리지(mg) 단위 변환
    #[inline]
    pub fn raw_to_mg(raw_16bit: i16) -> i16 {
        let val_14bit = raw_16bit >> 2;
        ((val_14bit as i32 * 244) / 1000) as i16
    }
}

/// LIS2MDL 3축 지자기 센서 감도 계수
pub mod lis2mdl {
    /// 지자기 감도: 1.5 mgauss/LSB
    pub const MAG_MGAUSS_PER_LSB: f32 = 1.5;

    /// 원시 지자기 ADC 값 -> 밀리가우스(mgauss) 단위 변환
    #[inline]
    pub fn raw_to_mgauss(raw: i16) -> i16 {
        ((raw as i32 * 15) / 10) as i16
    }

    /// 원시 지자기 ADC 값 -> 밀리가우스(mgauss) 단위 변환 (f32)
    #[inline]
    pub fn raw_to_mgauss_f32(raw: i16) -> f32 {
        raw as f32 * MAG_MGAUSS_PER_LSB
    }
}

/// LPS22HH 기압 및 온도 감도 계수
pub mod lps22hh {
    /// 기압 분해능: 4096 LSB/hPa
    pub const PRESS_LSB_PER_HPA: u32 = 4096;

    /// 24비트 원시 기압값 -> hPa * 10 (소수점 1자리 정수) 단위 변환
    #[inline]
    pub fn raw_to_hpa_x10(raw_24bit: u32) -> u32 {
        (raw_24bit * 10) / PRESS_LSB_PER_HPA
    }

    /// 16비트 원시 온도값 -> °C * 10 (소수점 1자리 정수) 단위 변환 (100 LSB/°C)
    #[inline]
    pub fn raw_to_temp_x10(raw_16bit: i16) -> i16 {
        (raw_16bit * 10) / 100
    }
}

/// STTS751 정밀 온도계 감도 계수
pub mod stts751 {
    /// 하위 바이트 (4비트) 분해능: 0.0625 °C/LSB
    pub const LOW_BYTE_RESOLUTION_C: f32 = 0.0625;

    /// 상위(정수부)/하위(소수부 4비트) 바이트 -> °C * 10 단위 변환
    #[inline]
    pub fn raw_to_temp_x10(high_byte: u8, low_byte: u8) -> i16 {
        let h = high_byte as i8 as i32;
        let l = (low_byte >> 4) as i32;
        ((h * 10) + ((l * 625) / 1000)) as i16
    }
}

/// 공통 물리 상수 및 수학적 단위 변환 계수
pub mod constants {
    /// 표준 중력 가속도 (m/s^2)
    pub const STANDARD_GRAVITY: f32 = 9.80665;

    /// 라디안 -> 도(Degree) 변환 계수
    pub const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

    /// 도(Degree) -> 라디안 변환 계수
    pub const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
}
