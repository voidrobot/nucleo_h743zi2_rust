#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::i2c::{self, I2c};
use embassy_time::{Duration, Ticker};
use nucleo_bsp::iks01a3::*;
use nucleo_bsp::{BoardLeds, I2C_FAST_MODE_HZ};

use mixed_cpp_legacy_05::cpp_bridge::SafeBiquadFilter;

/// Biquad 필터 및 센서 취득 샘플링 주파수 (Hz)
const SAMPLING_RATE_HZ: u64 = 100;
/// 버터워스 저역통과 필터 차단 주파수 (Hz)
const FILTER_CUTOFF_HZ: f32 = 5.0;
/// 텔레메트리 RTT 출력 주기 (스텝 수, 100스텝 = 1초)
const TELEMETRY_INTERVAL_STEPS: u32 = 100;
/// 하트비트 LED 토글 주기 (스텝 수, 50스텝 = 0.5초)
const HEARTBEAT_INTERVAL_STEPS: u32 = 50;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("============================================================");
    info!(">>> NUCLEO-H743ZI2 Mixed Language (Rust + Legacy C++) <<<");
    info!("============================================================");

    let p = embassy_stm32::init(Default::default());
    let mut leds = BoardLeds::new(p.PB0, p.PE1, p.PB14);
    leds.green.set_high();

    // 1. I2C1 마스터 버스 초기화 (400kHz Fast Mode)
    let mut i2c = I2c::new(
        p.I2C1,
        p.PB8, // SCL
        p.PB9, // SDA
        Irqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        I2C_FAST_MODE_HZ,
        Default::default(),
    );
    info!("I2C1 버스 400kHz 초기화 완료 (PB8/PB9)");

    // 2. LSM6DSO 6축 IMU 센서 WHO_AM_I 검증 및 활성화
    let mut who = [0u8; 1];
    if let Err(e) = i2c
        .write_read(ADDR_LSM6DSO, &[REG_WHO_AM_I], &mut who)
        .await
    {
        error!("LSM6DSO WHO_AM_I 읽기 실패: {:?}", e);
        leds.red.set_high();
        return;
    }
    info!(
        "LSM6DSO WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})",
        who[0], ID_LSM6DSO
    );

    // CTRL1_XL = 0x62 (416Hz ODR, ±2g, LPF2 활성화)
    let _ = i2c
        .write(
            ADDR_LSM6DSO,
            &[lsm6dso::CTRL1_XL, lsm6dso::VAL_CTRL1_XL_416HZ_2G_LPF2],
        )
        .await;
    // CTRL2_G = 0x60 (416Hz ODR, ±250dps)
    let _ = i2c
        .write(
            ADDR_LSM6DSO,
            &[lsm6dso::CTRL2_G, lsm6dso::VAL_CTRL2_G_416HZ_250DPS],
        )
        .await;
    info!("LSM6DSO 416Hz ODR 하드웨어 가속도계 가동 완료");

    // 3. 레거시 C++ Biquad 필터 인스턴스 초기화 (Zero-Allocation 인라인 스택 할당)
    // 샘플링 주파수: 100 Hz, 차단 주파수: 5 Hz (고주파 노이즈 제거), Q: 1/√2 (Butterworth)
    let q_butterworth = core::f32::consts::FRAC_1_SQRT_2;
    let mut filter_x =
        SafeBiquadFilter::new_lpf(SAMPLING_RATE_HZ as f32, FILTER_CUTOFF_HZ, q_butterworth);
    let mut filter_y =
        SafeBiquadFilter::new_lpf(SAMPLING_RATE_HZ as f32, FILTER_CUTOFF_HZ, q_butterworth);
    let mut filter_z =
        SafeBiquadFilter::new_lpf(SAMPLING_RATE_HZ as f32, FILTER_CUTOFF_HZ, q_butterworth);
    info!(
        "C++ BiquadFilter 3축(X, Y, Z) 인스턴스 초기화 완료 (Fs={}Hz, Fc={}Hz)",
        SAMPLING_RATE_HZ, FILTER_CUTOFF_HZ
    );

    leds.green.set_low();

    // 4. 100 Hz (10ms) 비동기 센서 취득 및 레거시 C++ 필터 실시간 연동 루프
    let mut ticker = Ticker::every(Duration::from_hz(SAMPLING_RATE_HZ));
    let mut count = 0u32;
    let mut accel_buf = [0u8; 6];

    loop {
        ticker.next().await;
        count = count.wrapping_add(1);

        // LSM6DSO 가속도계 데이터 레지스터 (OUTX_L_A ~ OUTZ_H_A) 6바이트 버스트 읽기
        if let Err(e) = i2c
            .write_read(ADDR_LSM6DSO, &[lsm6dso::OUTX_L_A], &mut accel_buf)
            .await
        {
            warn!("가속도계 읽기 오류: {:?}", e);
            continue;
        }

        let ax_raw = i16::from_le_bytes([accel_buf[0], accel_buf[1]]);
        let ay_raw = i16::from_le_bytes([accel_buf[2], accel_buf[3]]);
        let az_raw = i16::from_le_bytes([accel_buf[4], accel_buf[5]]);

        // ±2g 범위 정밀 감도 모듈 적용 (0.061 mg/LSB)
        let ax_mg = lsm6dso::raw_to_mg_f32(ax_raw);
        let ay_mg = lsm6dso::raw_to_mg_f32(ay_raw);
        let az_mg = lsm6dso::raw_to_mg_f32(az_raw);

        // --- C++ 레거시 Direct Form II Transposed Biquad LPF FFI 호출 ---
        let ax_filt = filter_x.process(ax_mg);
        let ay_filt = filter_y.process(ay_mg);
        let az_filt = filter_z.process(az_mg);

        // 매 100스텝(1초)마다 RTT로 원시 노이즈 데이터 vs C++ 필터링 데이터 비교 출력
        if count.is_multiple_of(TELEMETRY_INTERVAL_STEPS) {
            info!(
                "[100Hz #{}] RAW: [{=f32}, {=f32}, {=f32}] mg | C++ LPF: [{=f32}, {=f32}, {=f32}] mg",
                count, ax_mg, ay_mg, az_mg, ax_filt, ay_filt, az_filt
            );
        }

        // 50스텝(0.5초)마다 녹색 LED 토글 (하트비트)
        if count.is_multiple_of(HEARTBEAT_INTERVAL_STEPS) {
            leds.green.toggle();
        }
    }
}
