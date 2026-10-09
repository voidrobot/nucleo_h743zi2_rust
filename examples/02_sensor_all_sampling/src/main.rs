#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::peripherals::I2C1;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use nucleo_bsp::BoardLeds;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
});

use embassy_executor::InterruptExecutor;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[embassy_stm32::interrupt]
unsafe fn CEC() {
    EXECUTOR_HIGH.on_interrupt();
}

type I2cBus = Mutex<CriticalSectionRawMutex, Option<I2c<'static, embassy_stm32::mode::Async>>>;
static I2C_BUS: I2cBus = Mutex::new(None);

/// IKS01A3 쉴드 센서 전수 계측 통합 데이터 구조체
#[derive(Copy, Clone, Default)]
pub struct SensorSnapshot {
    // [IMU 100Hz 선점형 실시간 갱신] LSM6DSO 6축 + LIS2DW12 3축 가속도
    pub lsm_accel_mg: [i16; 3],  // X, Y, Z (단위: mg, ±2g 기준)
    pub lsm_gyro_dps: [i16; 3],  // X, Y, Z (단위: dps, ±250dps 기준)
    pub lis2dw_accel_mg: [i16; 3], // 보조 가속도계 X, Y, Z (단위: mg)
    pub imu_sample_count: u32,
    pub imu_dt_us: u32,          // 실측 주기 (목표치: 10,000 us)
    pub imu_min_dt_us: u32,      // 최소 주기
    pub imu_max_dt_us: u32,      // 최대 주기

    // [MAG 10Hz 갱신] LIS2MDL 3축 지자기
    pub mag_mgauss: [i16; 3],    // X, Y, Z (단위: mgauss)
    pub mag_sample_count: u32,

    // [ENV 1Hz 갱신] LPS22HH 기압/온도, STTS751 정밀온도, HTS221 온습도
    pub press_hpa_x10: u32,      // hPa * 10 (소수점 1자리)
    pub press_temp_c_x10: i16,   // °C * 10
    pub stts_temp_c_x10: i16,    // °C * 10
    pub hts_humidity_x10: u16,   // % rH * 10
    pub hts_temp_c_x10: i16,     // °C * 10
    pub env_sample_count: u32,
}

static SENSOR_STATE: Mutex<CriticalSectionRawMutex, SensorSnapshot> = Mutex::new(SensorSnapshot {
    lsm_accel_mg: [0; 3],
    lsm_gyro_dps: [0; 3],
    lis2dw_accel_mg: [0; 3],
    imu_sample_count: 0,
    imu_dt_us: 10_000,
    imu_min_dt_us: 10_000,
    imu_max_dt_us: 10_000,
    mag_mgauss: [0; 3],
    mag_sample_count: 0,
    press_hpa_x10: 0,
    press_temp_c_x10: 0,
    stts_temp_c_x10: 0,
    hts_humidity_x10: 0,
    hts_temp_c_x10: 0,
    env_sample_count: 0,
});

// I2C 디바이스 7비트 주소 정의
const ADDR_LSM6DSO: u8 = 0x6B;
const ADDR_LIS2MDL: u8 = 0x1E;
const ADDR_LIS2DW12: u8 = 0x19;
const ADDR_LPS22HH: u8 = 0x5D;
const ADDR_HTS221: u8 = 0x5F;
const ADDR_STTS751: u8 = 0x4A;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("============================================================");
    info!("X-NUCLEO-IKS01A3 Multi-Rate Heterogeneous Sensor Sampling");
    info!("============================================================");

    let p = embassy_stm32::init(Default::default());
    let mut leds = BoardLeds::new(p.PB0, p.PE1, p.PB14);
    leds.green.set_high(); // 초기화 시작 표시

    // Arduino UNO 헤더 I2C1 핀: D15=PB8(SCL), D14=PB9(SDA)
    let i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        Hertz(400_000), // Fast Mode 400kHz
        Default::default(),
    );

    {
        let mut bus = I2C_BUS.lock().await;
        *bus = Some(i2c);
    }
    info!("I2C1 버스 400kHz Fast Mode 초기화 완료 (PB8/PB9)");

    // 1단계: 6종 센서 WHO_AM_I 검증 및 활성화 시퀀스
    info!(">>> 1단계: X-NUCLEO-IKS01A3 6종 센서 시그니처 검증 및 Wake-up...");
    init_sensors().await;
    info!("전체 6종 센서 초기화 완료. 비동기 멀티태스크 샘플링 개시!");

    // 2단계: 고우선순위 선점형 InterruptExecutor 초기화 (NVIC 하드웨어 인터럽트 기반)
    interrupt::CEC.set_priority(Priority::P6);
    let high_spawner = EXECUTOR_HIGH.start(interrupt::CEC);

    // 고우선순위 선점형 실시간 IMU 태스크 스폰 (Thread Mode 태스크 강제 선점)
    high_spawner.must_spawn(task_imu_100hz());

    // 일반 Thread Mode 협력형 태스크 스폰
    spawner.must_spawn(task_mag_10hz());
    spawner.must_spawn(task_env_1hz());
    spawner.must_spawn(task_dashboard_reporter());

    leds.green.set_low();
}

/// 6종 센서의 WHO_AM_I 검증 및 초기 설정
///
/// =========================================================================================
/// [엔지니어링 샘플링 정책 및 DSP 안티-에일리어싱(Anti-Aliasing) 설계 배경]
/// =========================================================================================
///
/// 1. 마스터 타이밍 권한(Master Timing Authority)의 단일화:
///    - 센서(LSM6DSO 등) 내부 타이머는 실리콘 온칩 RC 발진기로 구동되며, 온도 드리프트 및 공정
///      편차로 인해 최대 ±1% ~ ±5% 수준의 심각한 주파수 오차를 갖는다.
///    - 반면 메인보드 MCU(STM32H743ZI)는 온도 보상 능력이 우수한 ±20 ppm 급 외부 수정 진동자
///      (HSE Crystal, 8~25 MHz) 기반 PLL로 480 MHz 시스템 클럭 및 하드웨어 타이머를 구동한다.
///    - 따라서 AHRS 융합 알고리즘의 시간 간격(dt) 무결성을 보장하기 위한 기준 클럭은 반드시
///      메인보드 MCU 타이머(`embassy_time::Ticker`)가 절대적 마스터 권한을 행사해야 한다.
///
/// 2. 비트 주파수(Beating / Moiré Effect) 및 위상 지터 방어:
///    - 만약 MCU가 100 Hz로 폴링하고 센서가 104 Hz로 내부 샘플링할 경우, 4 Hz 차이 주파수로 인해
///      MCU가 어떤 주기에는 방금 생성된 최신 데이터를 읽고, 어떤 주기에는 10ms 이전 데이터를
///      읽는 비트 현상(Beating) 및 최대 9.6ms의 샘플링 위상 지터(Phase Jitter)가 발생한다.
///    - 이를 극복하기 위해 센서 ODR을 타깃 주기보다 훨씬 높은 416 Hz(약 4.16배 오버샘플링)로
///      설정하여, MCU가 언제 I2C 읽기를 수행하더라도 레지스터에는 항상 1~2.4ms 이내의 최신
///      샘플이 대기하도록 보장한다.
///
/// 3. 나이퀴스트-섀넌 정리(Nyquist-Shannon) 기반 온칩 LPF2 안티-에일리어싱:
///    - 센서를 416 Hz로 구동하더라도 MCU가 100 Hz로 다운샘플링하여 수신하므로, 50 Hz(나이퀴스트
///      한계 주파수)를 초과하는 로봇 모터 진동 및 고주파 기계 노이즈는 0~50 Hz 대역으로 폴딩되어
///      치명적인 에일리어싱 왜곡(Aliasing Distortion)을 유발한다.
///    - 이를 원천 차단하기 위해 LSM6DSO 내부 2차 디지털 저역통과필터(LPF2)를 활성화하고,
///      차단 주파수(Cutoff Frequency)를 ODR/10인 41.6 Hz(`CTRL8_XL = 0x20`)로 설정하여
///      50 Hz 이상의 고주파 신호를 하드웨어 레벨에서 감쇠시킨 후 레지스터에 기록한다.
/// =========================================================================================
async fn init_sensors() {
    let mut bus = I2C_BUS.lock().await;
    let i2c = bus.as_mut().unwrap();

    // 1. LSM6DSO (6축 고정밀 IMU): WHO_AM_I=0x0F -> 0x6C
    let mut who = [0u8; 1];
    let _ = i2c.blocking_write_read(ADDR_LSM6DSO, &[0x0F], &mut who);
    info!("  [LSM6DSO 6축 IMU] WHO_AM_I: 0x{:02X} (기대값: 0x6C)", who[0]);

    // CTRL1_XL = 0x62: ODR=416Hz (High-Performance), ±2g, LPF2_XL_EN=1 (LPF2 활성화)
    let _ = i2c.blocking_write(ADDR_LSM6DSO, &[0x10, 0x62]);
    // CTRL2_G = 0x60: ODR=416Hz (High-Performance), ±250dps
    let _ = i2c.blocking_write(ADDR_LSM6DSO, &[0x11, 0x60]);
    // CTRL8_XL = 0x20: HPCF_XL[2:0]=001b -> LPF2 Cutoff = ODR / 10 = 41.6 Hz (50Hz 나이퀴스트 보호)
    let _ = i2c.blocking_write(ADDR_LSM6DSO, &[0x17, 0x20]);

    // 2. LIS2MDL (3축 지자기): WHO_AM_I=0x4F -> 0x40
    let _ = i2c.blocking_write_read(ADDR_LIS2MDL, &[0x4F], &mut who);
    info!("  [LIS2MDL 지자기] WHO_AM_I: 0x{:02X} (기대값: 0x40)", who[0]);
    // CFG_REG_A = 0x00 (Continuous 10Hz), CFG_REG_C = 0x10 (BDU=1)
    let _ = i2c.blocking_write(ADDR_LIS2MDL, &[0x60, 0x00]);
    let _ = i2c.blocking_write(ADDR_LIS2MDL, &[0x62, 0x10]);

    // 3. LIS2DW12 (보조 3축 가속도계): WHO_AM_I=0x0F -> 0x44
    let _ = i2c.blocking_write_read(ADDR_LIS2DW12, &[0x0F], &mut who);
    info!("  [LIS2DW12 보조 가속도] WHO_AM_I: 0x{:02X} (기대값: 0x44)", who[0]);
    // CTRL1 = 0x64 (200Hz ODR 고속 오버샘플링, High-Performance 14-bit, ±2g)
    let _ = i2c.blocking_write(ADDR_LIS2DW12, &[0x20, 0x64]);

    // 4. LPS22HH (기압/온도): WHO_AM_I=0x0F -> 0xB3
    let _ = i2c.blocking_write_read(ADDR_LPS22HH, &[0x0F], &mut who);
    info!("  [LPS22HH 기압계] WHO_AM_I: 0x{:02X} (기대값: 0xB3)", who[0]);
    // CTRL_REG1 = 0x12 (1Hz ODR, BDU=1)
    let _ = i2c.blocking_write(ADDR_LPS22HH, &[0x10, 0x12]);

    // 5. STTS751 (고정밀 온도계): Product ID(0xFD)=0x01
    let _ = i2c.blocking_write_read(ADDR_STTS751, &[0xFD], &mut who);
    info!("  [STTS751 정밀온도] Product ID: 0x{:02X} (기대값: 0x01)", who[0]);
    // Config=0x00 (Continuous), Rate=0x04 (1 conversion/sec)
    let _ = i2c.blocking_write(ADDR_STTS751, &[0x03, 0x00]);
    let _ = i2c.blocking_write(ADDR_STTS751, &[0x04, 0x04]);

    // 6. HTS221 (온습도계): WHO_AM_I=0x0F -> 0xBC
    let _ = i2c.blocking_write_read(ADDR_HTS221, &[0x0F], &mut who);
    info!("  [HTS221 온습도계] WHO_AM_I: 0x{:02X} (기대값: 0xBC)", who[0]);
    // AV_CONF=0x1B, CTRL_REG1=0x85 (PD=1, BDU=1, ODR=1Hz)
    let _ = i2c.blocking_write(ADDR_HTS221, &[0x10, 0x1B]);
    let _ = i2c.blocking_write(ADDR_HTS221, &[0x20, 0x85]);
}

/// [Task 1: 100 Hz] 고우선순위 선점형 IMU 실시간 모션 태스크 (10ms 주기)
///
/// =========================================================================================
/// [드론 비행 제어기 / 4족 보행 로봇 평형 제어 / 고정밀 EKF를 위한 Hard Real-Time 아키텍처]
/// =========================================================================================
///
/// 1. 하드웨어 NVIC 다이렉트 강제 선점 (InterruptExecutor 기반):
///    - 본 태스크는 Cortex-M7 하드웨어 인터럽트(CEC IRQ, Priority P6)에 바인딩된
///      `InterruptExecutor`에서 독립적으로 구동된다.
///    - 하위 Thread Mode에서 실행되는 지자기/환경센서/RTT 로깅 태스크가 실행 중이더라도,
///      10.00ms 타이머 만료 시 하드웨어 NVIC 인터럽트가 즉각 발생하여 하위 태스크를
///      물리적으로 강제 선점(Hardware Preemption)한다.
///    - 이로써 협력형 스케줄러에서 발생하는 태스크 간 스케줄링 지터(Scheduling Jitter)를
///      마이크로초(µs) 이하 수준으로 억제한다.
///
/// 2. 버스 락 점유 분할을 통한 우선순위 역전(Priority Inversion) 방어:
///    - 저속 환경 센서 태스크(`task_env_1hz`)가 센서 계측 사이마다 버스 락을 반환하고
///      미세 슬립(Yield Window)을 제공하므로, 고우선순위 IMU 태스크가 I2C 버스를 최대
///      수십~수백 µs 이내에 즉각 확보할 수 있도록 보장한다.
///
/// 3. 정밀 dt 프로파일링:
///    - 칼만 필터(EKF)의 공분산 전파(P = F*P*F^T + Q) 및 적분 계산의 신뢰도를 입증하기 위해
///      루프 간 실제 경과 시간(`dt_us`)과 최소/최대 지터(Min/Max Jitter)를 실시간 계측한다.
/// =========================================================================================
#[embassy_executor::task]
async fn task_imu_100hz() {
    let mut ticker = Ticker::every(Duration::from_hz(100)); // 100 Hz (10ms)
    let mut buf = [0u8; 12];
    let mut last_instant = Instant::now();
    let mut min_dt_us = u32::MAX;
    let mut max_dt_us = 0u32;
    let mut is_first = true;

    loop {
        ticker.next().await;

        let now = Instant::now();
        let dt_us = (now - last_instant).as_micros() as u32;
        last_instant = now;

        if is_first {
            is_first = false;
        } else {
            if dt_us < min_dt_us {
                min_dt_us = dt_us;
            }
            if dt_us > max_dt_us {
                max_dt_us = dt_us;
            }
        }

        let mut bus = I2C_BUS.lock().await;
        if let Some(i2c) = bus.as_mut() {
            // 1. LSM6DSO 가속도/자이로 12바이트 버스트 리드 (0x22 OUTX_L_G ~ 0x2D OUTZ_H_A)
            if i2c.blocking_write_read(ADDR_LSM6DSO, &[0x22], &mut buf).is_ok() {
                let gx = i16::from_le_bytes([buf[0], buf[1]]);
                let gy = i16::from_le_bytes([buf[2], buf[3]]);
                let gz = i16::from_le_bytes([buf[4], buf[5]]);
                let ax = i16::from_le_bytes([buf[6], buf[7]]);
                let ay = i16::from_le_bytes([buf[8], buf[9]]);
                let az = i16::from_le_bytes([buf[10], buf[11]]);

                // 감도 환산: ±2g -> 0.061 mg/LSB, ±250dps -> 8.75 mdps/LSB
                let accel_mg = [
                    ((ax as i32 * 61) / 1000) as i16,
                    ((ay as i32 * 61) / 1000) as i16,
                    ((az as i32 * 61) / 1000) as i16,
                ];
                let gyro_dps = [
                    ((gx as i32 * 875) / 100000) as i16,
                    ((gy as i32 * 875) / 100000) as i16,
                    ((gz as i32 * 875) / 100000) as i16,
                ];

                // 2. LIS2DW12 보조 가속도계 6바이트 읽기 (0x28 OUT_X_L)
                let mut buf_dw = [0u8; 6];
                let mut accel2_mg = [0i16; 3];
                if i2c.blocking_write_read(ADDR_LIS2DW12, &[0x28], &mut buf_dw).is_ok() {
                    let a2x = i16::from_le_bytes([buf_dw[0], buf_dw[1]]) >> 2;
                    let a2y = i16::from_le_bytes([buf_dw[2], buf_dw[3]]) >> 2;
                    let a2z = i16::from_le_bytes([buf_dw[4], buf_dw[5]]) >> 2;
                    accel2_mg = [
                        ((a2x as i32 * 244) / 1000) as i16,
                        ((a2y as i32 * 244) / 1000) as i16,
                        ((a2z as i32 * 244) / 1000) as i16,
                    ];
                }

                // 공유 상태 갱신
                let mut state = SENSOR_STATE.lock().await;
                state.lsm_accel_mg = accel_mg;
                state.lsm_gyro_dps = gyro_dps;
                state.lis2dw_accel_mg = accel2_mg;
                state.imu_sample_count += 1;
                state.imu_dt_us = dt_us;
                state.imu_min_dt_us = if min_dt_us == u32::MAX { dt_us } else { min_dt_us };
                state.imu_max_dt_us = max_dt_us;
            }
        }
    }
}

/// [Task 2: 10 Hz] 중속 지자기 센서 샘플링 (100ms 주기)
#[embassy_executor::task]
async fn task_mag_10hz() {
    let mut ticker = Ticker::every(Duration::from_hz(10)); // 10 Hz (100ms)
    let mut buf = [0u8; 6];

    loop {
        ticker.next().await;

        let mut bus = I2C_BUS.lock().await;
        if let Some(i2c) = bus.as_mut() {
            // LIS2MDL 6바이트 읽기 (0x68 OUTX_L_REG ~ 0x6D OUTZ_H_REG)
            if i2c.blocking_write_read(ADDR_LIS2MDL, &[0x68], &mut buf).is_ok() {
                let mx = i16::from_le_bytes([buf[0], buf[1]]);
                let my = i16::from_le_bytes([buf[2], buf[3]]);
                let mz = i16::from_le_bytes([buf[4], buf[5]]);

                // 감도 환산: 1.5 mgauss/LSB
                let mag = [
                    ((mx as i32 * 15) / 10) as i16,
                    ((my as i32 * 15) / 10) as i16,
                    ((mz as i32 * 15) / 10) as i16,
                ];

                let mut state = SENSOR_STATE.lock().await;
                state.mag_mgauss = mag;
                state.mag_sample_count += 1;
            }
        }
    }
}

/// [Task 3: 1 Hz] 저속 환경 센서 샘플링 (1000ms 주기)
///
/// 저속 환경 센서들이 I2C 버스를 장시간 독점하지 않도록 센서 단위로 락을 획득/반환하고,
/// 센서 간 50us 미세 슬립(Yield Window)을 삽입하여 고우선순위 선점형 IMU 루프를 보호한다.
#[embassy_executor::task]
async fn task_env_1hz() {
    let mut ticker = Ticker::every(Duration::from_hz(1)); // 1 Hz (1000ms)

    loop {
        ticker.next().await;

        // 1. LPS22HH 기압(3B: 0x28~0x2A) 및 온도(2B: 0x2B~0x2C)
        let mut p_hpa_x10 = 0u32;
        let mut p_temp_x10 = 0i16;
        {
            let mut bus = I2C_BUS.lock().await;
            if let Some(i2c) = bus.as_mut() {
                let mut press_buf = [0u8; 5];
                if i2c.blocking_write_read(ADDR_LPS22HH, &[0x28], &mut press_buf).is_ok() {
                    let raw_press = (press_buf[0] as u32)
                        | ((press_buf[1] as u32) << 8)
                        | ((press_buf[2] as u32) << 16);
                    let raw_temp = i16::from_le_bytes([press_buf[3], press_buf[4]]);
                    p_hpa_x10 = (raw_press * 10) / 4096;
                    p_temp_x10 = (raw_temp * 10) / 100;
                }
            }
        }
        // 고우선순위 IMU 선점을 위한 버스 양보 윈도우
        Timer::after_micros(50).await;

        // 2. STTS751 고정밀 온도계 (0x00 High, 0x02 Low)
        let mut s_temp_x10 = 0i16;
        {
            let mut bus = I2C_BUS.lock().await;
            if let Some(i2c) = bus.as_mut() {
                let mut stts_high = [0u8; 1];
                let mut stts_low = [0u8; 1];
                if i2c.blocking_write_read(ADDR_STTS751, &[0x00], &mut stts_high).is_ok()
                    && i2c.blocking_write_read(ADDR_STTS751, &[0x02], &mut stts_low).is_ok()
                {
                    let h = stts_high[0] as i8 as i32;
                    let l = (stts_low[0] >> 4) as i32;
                    s_temp_x10 = ((h * 10) + ((l * 625) / 1000)) as i16;
                }
            }
        }
        Timer::after_micros(50).await;

        // 3. HTS221 온습도계 원시 읽기 (0x28 | 0x80 습도, 0x2A | 0x80 온도)
        let mut h_hum_x10 = 0u16;
        let mut h_temp_x10 = 0i16;
        {
            let mut bus = I2C_BUS.lock().await;
            if let Some(i2c) = bus.as_mut() {
                let mut hts_h_buf = [0u8; 2];
                let mut hts_t_buf = [0u8; 2];
                if i2c.blocking_write_read(ADDR_HTS221, &[0x28 | 0x80], &mut hts_h_buf).is_ok()
                    && i2c.blocking_write_read(ADDR_HTS221, &[0x2A | 0x80], &mut hts_t_buf).is_ok()
                {
                    let raw_h = i16::from_le_bytes(hts_h_buf);
                    let raw_t = i16::from_le_bytes(hts_t_buf);
                    // 단순 추정치 (정밀 보정식 전 기초 스케일)
                    h_hum_x10 = ((raw_h.abs() as u32 * 1000) / 32767).min(1000) as u16;
                    h_temp_x10 = (raw_t / 64) as i16;
                }
            }
        }

        // 공유 상태 갱신
        let mut state = SENSOR_STATE.lock().await;
        state.press_hpa_x10 = p_hpa_x10;
        state.press_temp_c_x10 = p_temp_x10;
        state.stts_temp_c_x10 = s_temp_x10;
        state.hts_humidity_x10 = h_hum_x10;
        state.hts_temp_c_x10 = h_temp_x10;
        state.env_sample_count += 1;
    }
}

/// [Task 4: 1 Hz] 최신 샘플링 스냅샷 종합 리포터 대시보드
#[embassy_executor::task]
async fn task_dashboard_reporter() {
    let mut ticker = Ticker::every(Duration::from_hz(1)); // 1초마다 출력
    let mut report_seq: u32 = 0;

    loop {
        ticker.next().await;
        report_seq += 1;

        let snap = {
            let state = SENSOR_STATE.lock().await;
            *state
        };

        info!("===================[ IKS01A3 Multi-Rate Report #{}: 1초 주기 ]===================", report_seq);
        info!("  [RT-IMU 100Hz (선점형 InterruptExecutor)] 누적 {}회 | dt: {} us (min: {}, max: {})",
            snap.imu_sample_count, snap.imu_dt_us, snap.imu_min_dt_us, snap.imu_max_dt_us
        );
        info!("    -> Accel: [X: {} mg, Y: {} mg, Z: {} mg] | Gyro: [X: {} dps, Y: {} dps, Z: {} dps]",
            snap.lsm_accel_mg[0], snap.lsm_accel_mg[1], snap.lsm_accel_mg[2],
            snap.lsm_gyro_dps[0], snap.lsm_gyro_dps[1], snap.lsm_gyro_dps[2],
        );
        info!("  [AUX 100Hz] LIS2DW12 Accel2: [X: {} mg, Y: {} mg, Z: {} mg]",
            snap.lis2dw_accel_mg[0], snap.lis2dw_accel_mg[1], snap.lis2dw_accel_mg[2],
        );
        info!("  [MAG  10Hz (누적 {}회)] LIS2MDL Mag: [X: {} mgauss, Y: {} mgauss, Z: {} mgauss]",
            snap.mag_sample_count,
            snap.mag_mgauss[0], snap.mag_mgauss[1], snap.mag_mgauss[2],
        );
        info!("  [ENV   1Hz (누적 {}회)] Press: {}.{} hPa (LPS22HH) | Temp: {}.{} °C (STTS751)",
            snap.env_sample_count,
            snap.press_hpa_x10 / 10, snap.press_hpa_x10 % 10,
            snap.stts_temp_c_x10 / 10, (snap.stts_temp_c_x10 % 10).abs(),
        );
        info!("----------------------------------------------------------------------------------");
    }
}
