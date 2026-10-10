//! # X-NUCLEO-IKS01A3 모션 및 환경 센서 쉴드 드라이버 패키지
//!
//! STMicroelectronics 사의 X-NUCLEO-IKS01A3 아두이노 UNO R3 쉴드 지원 모듈이다.
//!
//! ## 지원 센서 목록
//! 1. **LSM6DSO**: 6축 고정밀 초저전력 IMU (3축 가속도 + 3축 자이로)
//! 2. **LIS2MDL**: 3축 고성능 지자기 센서 (Magnetometer)
//! 3. **LIS2DW12**: 3축 초저전력 보조 가속도계 (Auxiliary Accelerometer)
//! 4. **LPS22HH**: 260~1260 hPa 고정밀 대기압 및 온도 센서
//! 5. **STTS751**: ±0.5°C 고정밀 I2C 로컬 온도 센서
//! 6. **HTS221**: 정전용량식 디지털 온습도 센서 (OTP 캘리브레이션 지원)

pub mod registers;
pub mod sensitivity;
pub mod hts221_calib;

pub use registers::addresses::*;
pub use registers::who_am_i::*;
pub use sensitivity::constants::*;
pub use hts221_calib::Hts221Calibration;
