//! # X-NUCLEO-IKS01A3 센서 쉴드 I2C 주소 및 레지스터 맵
//!
//! STMicroelectronics X-NUCLEO-IKS01A3 확장 보드에 탑재된 6종 센서의
//! 7비트 I2C 슬레이브 주소, WHO_AM_I 시그니처 및 제어 레지스터 정의이다.

/// 7비트 I2C 슬레이브 주소
pub mod addresses {
    /// LSM6DSO: 6축 초저전력 IMU (기본 주소: SA0=1 -> 0x6B)
    pub const ADDR_LSM6DSO: u8 = 0x6B;
    /// LIS2MDL: 3축 고성능 지자기 센서 (0x1E)
    pub const ADDR_LIS2MDL: u8 = 0x1E;
    /// LIS2DW12: 3축 초저전력 보조 가속도계 (SA0=1 -> 0x19)
    pub const ADDR_LIS2DW12: u8 = 0x19;
    /// LPS22HH: 260~1260 hPa 고정밀 압력 센서 (SA0=1 -> 0x5D)
    pub const ADDR_LPS22HH: u8 = 0x5D;
    /// STTS751: ±0.5°C 고정밀 로컬 온도 센서 (0x4A)
    pub const ADDR_STTS751: u8 = 0x4A;
    /// HTS221: 정전용량식 디지털 상대습도 및 온도 센서 (0x5F)
    pub const ADDR_HTS221: u8 = 0x5F;
}

/// WHO_AM_I 레지스터 주소 및 하드웨어 식별값
pub mod who_am_i {
    pub const REG_WHO_AM_I: u8 = 0x0F;
    pub const REG_LIS2MDL_WHO_AM_I: u8 = 0x4F;
    pub const REG_STTS751_PRODUCT_ID: u8 = 0xFD;

    pub const ID_LSM6DSO: u8 = 0x6C;
    pub const ID_LIS2MDL: u8 = 0x40;
    pub const ID_LIS2DW12: u8 = 0x44;
    pub const ID_LPS22HH: u8 = 0xB3;
    pub const ID_STTS751: u8 = 0x01;
    pub const ID_HTS221: u8 = 0xBC;
}

/// LSM6DSO 6축 IMU 레지스터 및 비트마스크
pub mod lsm6dso {
    pub const CTRL1_XL: u8 = 0x10;
    pub const CTRL2_G: u8 = 0x11;
    pub const CTRL3_C: u8 = 0x12;
    pub const CTRL8_XL: u8 = 0x17;

    // 데이터 출력 레지스터 (버스트 읽기: OUTX_L_G 0x22 ~ OUTZ_H_A 0x2D, 총 12바이트)
    pub const OUTX_L_G: u8 = 0x22;
    pub const OUTX_L_A: u8 = 0x28;

    // 가속도계 ODR 및 스케일 설정값
    /// ODR=416Hz (High-Performance), ±2g 범위, LPF2_XL_EN=1 (하드웨어 LPF2 활성화)
    pub const VAL_CTRL1_XL_416HZ_2G_LPF2: u8 = 0x62;
    /// ODR=104Hz, ±2g
    pub const VAL_CTRL1_XL_104HZ_2G: u8 = 0x40;

    // 자이로스코프 ODR 및 풀스케일 설정값
    /// ODR=416Hz (High-Performance), ±250dps
    pub const VAL_CTRL2_G_416HZ_250DPS: u8 = 0x60;
    /// ODR=104Hz, ±250dps
    pub const VAL_CTRL2_G_104HZ_250DPS: u8 = 0x40;

    // 2차 디지털 저역통과필터 (LPF2) 컷오프 주파수 설정
    /// LPF2 Cutoff = ODR / 10 = 41.6 Hz (100Hz 샘플링 기준 50Hz 나이퀴스트 주파수 방어)
    pub const VAL_CTRL8_XL_LPF2_ODR_DIV_10: u8 = 0x20;

    // 감도 계수 및 단위 변환 함수 통합 re-export
    pub use super::super::sensitivity::lsm6dso::*;
}

/// LIS2MDL 3축 지자기 센서 레지스터 및 설정값
pub mod lis2mdl {
    pub const CFG_REG_A: u8 = 0x60;
    pub const CFG_REG_B: u8 = 0x61;
    pub const CFG_REG_C: u8 = 0x62;
    pub const OUTX_L_REG: u8 = 0x68;

    pub const VAL_CFG_REG_A_RESET: u8 = 0x80;
    pub const VAL_CFG_REG_A_10HZ_CONT: u8 = 0x00;
    pub const VAL_CFG_REG_C_BDU: u8 = 0x10;

    pub use super::super::sensitivity::lis2mdl::*;
}

/// LIS2DW12 3축 보조 가속도계 레지스터 및 설정값
pub mod lis2dw12 {
    pub const CTRL1: u8 = 0x20;
    pub const OUT_X_L: u8 = 0x28;

    /// 200Hz ODR (High-Performance 14-bit), ±2g
    pub const VAL_CTRL1_200HZ_14BIT_2G: u8 = 0x64;

    pub use super::super::sensitivity::lis2dw12::*;
}

/// LPS22HH 기압계 레지스터 및 설정값
pub mod lps22hh {
    pub const CTRL_REG1: u8 = 0x10;
    pub const CTRL_REG2: u8 = 0x11;
    pub const PRESS_OUT_XL: u8 = 0x28; // 3B 기압 (0x28~0x2A), 2B 온도 (0x2B~0x2C)

    /// 1Hz ODR, Block Data Update(BDU)=1
    pub const VAL_CTRL_REG1_1HZ_BDU: u8 = 0x12;

    pub use super::super::sensitivity::lps22hh::*;
}

/// STTS751 정밀 온도계 레지스터 및 설정값
pub mod stts751 {
    pub const TEMP_HIGH: u8 = 0x00;
    pub const TEMP_LOW: u8 = 0x02;
    pub const CONFIG: u8 = 0x03;
    pub const CONVERSION_RATE: u8 = 0x04;

    pub const VAL_CONFIG_CONTINUOUS: u8 = 0x00;
    pub const VAL_RATE_1_CONV_PER_SEC: u8 = 0x04;

    pub use super::super::sensitivity::stts751::*;
}

/// HTS221 온습도계 레지스터 및 설정값
pub mod hts221 {
    pub const AV_CONF: u8 = 0x10;
    pub const CTRL_REG1: u8 = 0x20;

    // 온습도 데이터 레지스터 (다중 바이트 읽기 시 MSB 0x80 비트 OR 필요)
    pub const HUMIDITY_OUT_L: u8 = 0x28 | 0x80;
    pub const TEMP_OUT_L: u8 = 0x2A | 0x80;

    pub const VAL_AV_CONF_DEFAULT: u8 = 0x1B;
    /// PD=1 (Power Down 해제), BDU=1, ODR=1Hz
    pub const VAL_CTRL_REG1_1HZ_PD_BDU: u8 = 0x85;

    // OTP 캘리브레이션 레지스터 (0x30 ~ 0x3F)
    pub const CALIB_H0_RH_X2: u8 = 0x30 | 0x80;
    pub const CALIB_H1_RH_X2: u8 = 0x31 | 0x80;
    pub const CALIB_T0_DEGC_X8: u8 = 0x32 | 0x80;
    pub const CALIB_T1_DEGC_X8: u8 = 0x33 | 0x80;
    pub const CALIB_T1_T0_MSB: u8 = 0x35 | 0x80;
    pub const CALIB_H0_T0_OUT_L: u8 = 0x36 | 0x80;
    pub const CALIB_H1_T0_OUT_L: u8 = 0x3A | 0x80;
    pub const CALIB_T0_OUT_L: u8 = 0x3C | 0x80;
    pub const CALIB_T1_OUT_L: u8 = 0x3E | 0x80;
}
