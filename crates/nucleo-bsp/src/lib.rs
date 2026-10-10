#![cfg_attr(not(test), no_std)]

//! # NUCLEO-H743ZI2 & X-NUCLEO-IKS01A3 Board Support Package (BSP)
//!
//! NUCLEO-H743ZI2 보드 및 IKS01A3 확장 센서 쉴드를 위한 공통 하드웨어 추상화 계층이다.

pub use defmt;
#[cfg(not(test))]
use defmt_rtt as _;
#[cfg(not(test))]
use panic_probe as _;

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{PB0, PB14, PE1};

pub mod iks01a3;
pub mod uid;

/// NUCLEO-H743ZI2 온보드 핀아웃 매핑
/// - LED1 (Green): PB0
/// - LED2 (Yellow): PE1
/// - LED3 (Red): PB14
/// - B1 (User Push Button): PC13
///
/// Arduino Uno V3 확장 커넥터 (X-NUCLEO-IKS01A3 센서 버스 연동용):
/// - D15 (I2C1_SCL): PB8
/// - D14 (I2C1_SDA): PB9
pub mod pins {}

/// NUCLEO-H743ZI2 온보드 사용자 LED 세트
pub struct BoardLeds<'d> {
    pub green: Output<'d>,
    pub yellow: Output<'d>,
    pub red: Output<'d>,
}

impl<'d> BoardLeds<'d> {
    /// 기본 GPIO 출력 설정으로 온보드 LED 3종(Green, Yellow, Red)을 초기화한다.
    pub fn new(pb0: PB0, pe1: PE1, pb14: PB14) -> Self {
        Self {
            green: Output::new(pb0, Level::Low, Speed::Low),
            yellow: Output::new(pe1, Level::Low, Speed::Low),
            red: Output::new(pb14, Level::Low, Speed::Low),
        }
    }
}
