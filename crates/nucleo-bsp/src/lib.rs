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
use embassy_stm32::peripherals::{PA1, PA2, PA7, PB0, PB13, PB14, PC1, PC4, PC5, PE1, PG11, PG13};
use embassy_stm32::time::Hertz;

/// X-NUCLEO-IKS01A3 I2C1 Fast Mode 버스 주파수 (400 kHz)
pub const I2C_FAST_MODE_HZ: Hertz = Hertz(400_000);

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
///
/// LAN8742A 온보드 RMII 이더넷 핀 매핑:
/// - REF_CLK: PA1, MDIO: PA2, MDC: PC1, CRS_DV: PA7
/// - RXD0: PC4, RXD1: PC5, TXD0: PG13, TXD1: PB13, TX_EN: PG11
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

/// NUCLEO-H743ZI2 온보드 LAN8742A RMII 이더넷 핀셋
pub struct BoardRmiiPins {
    pub ref_clk: PA1,
    pub mdio: PA2,
    pub mdc: PC1,
    pub crs_dv: PA7,
    pub rx_d0: PC4,
    pub rx_d1: PC5,
    pub tx_d0: PG13,
    pub tx_d1: PB13,
    pub tx_en: PG11,
}

impl BoardRmiiPins {
    /// NUCLEO-H743ZI2 온보드 이더넷 RMII 핀셋을 묶어 생성한다.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pa1: PA1,
        pa2: PA2,
        pc1: PC1,
        pa7: PA7,
        pc4: PC4,
        pc5: PC5,
        pg13: PG13,
        pb13: PB13,
        pg11: PG11,
    ) -> Self {
        Self {
            ref_clk: pa1,
            mdio: pa2,
            mdc: pc1,
            crs_dv: pa7,
            rx_d0: pc4,
            rx_d1: pc5,
            tx_d0: pg13,
            tx_d1: pb13,
            tx_en: pg11,
        }
    }
}
