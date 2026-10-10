#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::peripherals::I2C1;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use nucleo_bsp::iks01a3::registers::*;
use nucleo_bsp::iks01a3::sensitivity::*;
use nucleo_bsp::iks01a3::*;
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

/// HTS221 공장 출하 OTP 캘리브레이션 파라미터 저장소
static HTS221_CALIB: Mutex<CriticalSectionRawMutex, Hts221Calibration> =
    Mutex::new(Hts221Calibration {
        h0_rh_x2: 0,
        h1_rh_x2: 0,
        h0_t0_out: 0,
        h1_t0_out: 0,
        t0_degc_x8: 0,
        t1_degc_x8: 0,
        t0_out: 0,
        t1_out: 0,
    });

/// IKS01A3 쉴드 센서 전수 계측 통합 데이터 구조체
#[derive(Copy, Clone, Default)]
pub struct SensorSnapshot {
    // [IMU 100Hz 선점형 실시간 갱신] LSM6DSO 6축 + LIS2DW12 3축 가속도
    pub lsm_accel_mg: [i16; 3],    // X, Y, Z (단위: mg, ±2g 기준)
    pub lsm_gyro_dps: [i16; 3],    // X, Y, Z (단위: dps, ±250dps 기준)
    pub lis2dw_accel_mg: [i16; 3], // 보조 가속도계 X, Y, Z (단위: mg)
    pub imu_sample_count: u32,
    pub imu_dt_us: u32,            // 실측 주기 (목표치: 10,000 us)
    pub imu_min_dt_us: u32,        // 최소 주기
    pub imu_max_dt_us: u32,        // 최대 주기

    // [MAG 10Hz 갱신] LIS2MDL 3축 지자기
    pub mag_mgauss: [i16; 3],      // X, Y, Z (단위: mgauss)
    pub mag_sample_count: u32,

    // [ENV 1Hz 갱신] LPS22HH 기압/온도, STTS751 정밀온도, HTS221 온습도 (정규 캘리브레이션 적용)
    pub press_hpa_x10: u32,        // hPa * 10 (소수점 1자리)
    pub press_temp_c_x10: i16,     // °C * 10
    pub stts_temp_c_x10: i16,      // °C * 10
    pub hts_humidity_x10: u16,     // % rH * 10
    pub hts_temp_c_x10: i16,       // °C * 10
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
    info!("I2C1 버스 400kHz Fast Mode DMA 비동기 드라이버 초기화 완료 (PB8/PB9)");

    // 1단계: 6종 센서 WHO_AM_I 검증 및 활성화 시퀀스
    info!(">>> 1단계: X-NUCLEO-IKS01A3 6종 센서 시그니처 검증 및 Wake-up...");
    init_sensors().await;
    info!("전체 6종 센서 초기화 및 HTS221 캘리브레이션 완료. 비동기 멀티태스크 샘플링 개시!");

    // 2단계: 고우선순위 선점형 InterruptExecutor 초기화 (NVIC 하드웨어 인터럽트 기반)
    interrupt::CEC.set_priority(Priority::P6);
    let high_spawner = EXECUTOR_HIGH.start(interrupt::CEC);

    // 고우선순위 선점형 실시간 IMU 태스크 스폰
    high_spawner.must_spawn(task_imu_100hz());

    // 일반 Thread Mode 협력형 태스크 스폰
    spawner.must_spawn(task_mag_10hz());
    spawner.must_spawn(task_env_1hz());
    spawner.must_spawn(task_dashboard_reporter());

    leds.green.set_low();
}

/// 6종 센서의 WHO_AM_I 검증 및 초기 설정
async fn init_sensors() {
    let mut bus = I2C_BUS.lock().await;
    let i2c = bus.as_mut().expect("I2C 버스 미초기화");

    // 1. LSM6DSO (6축 고정밀 IMU): WHO_AM_I=0x0F -> 0x6C
    let mut who = [0u8; 1];
    if let Err(e) = i2c.write_read(ADDR_LSM6DSO, &[REG_WHO_AM_I], &mut who).await {
        error!("LSM6DSO WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!("  [LSM6DSO 6축 IMU] WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})", who[0], ID_LSM6DSO);
    }

    if let Err(e) = i2c.write(ADDR_LSM6DSO, &[lsm6dso::CTRL1_XL, lsm6dso::VAL_CTRL1_XL_416HZ_2G_LPF2]).await {
        error!("LSM6DSO CTRL1_XL 설정 실패: {:?}", e);
    }
    if let Err(e) = i2c.write(ADDR_LSM6DSO, &[lsm6dso::CTRL2_G, lsm6dso::VAL_CTRL2_G_416HZ_250DPS]).await {
        error!("LSM6DSO CTRL2_G 설정 실패: {:?}", e);
    }
    if let Err(e) = i2c.write(ADDR_LSM6DSO, &[lsm6dso::CTRL8_XL, lsm6dso::VAL_CTRL8_XL_LPF2_ODR_DIV_10]).await {
        error!("LSM6DSO CTRL8_XL 설정 실패: {:?}", e);
    }

    // 2. LIS2MDL (3축 지자기): WHO_AM_I=0x4F -> 0x40
    if let Err(e) = i2c.write_read(ADDR_LIS2MDL, &[REG_LIS2MDL_WHO_AM_I], &mut who).await {
        error!("LIS2MDL WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!("  [LIS2MDL 지자기] WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})", who[0], ID_LIS2MDL);
    }
    let _ = i2c.write(ADDR_LIS2MDL, &[lis2mdl::CFG_REG_A, lis2mdl::VAL_CFG_REG_A_10HZ_CONT]).await;
    let _ = i2c.write(ADDR_LIS2MDL, &[lis2mdl::CFG_REG_C, lis2mdl::VAL_CFG_REG_C_BDU]).await;

    // 3. LIS2DW12 (보조 3축 가속도계): WHO_AM_I=0x0F -> 0x44
    if let Err(e) = i2c.write_read(ADDR_LIS2DW12, &[REG_WHO_AM_I], &mut who).await {
        error!("LIS2DW12 WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!("  [LIS2DW12 보조 가속도] WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})", who[0], ID_LIS2DW12);
    }
    let _ = i2c.write(ADDR_LIS2DW12, &[lis2dw12::CTRL1, lis2dw12::VAL_CTRL1_200HZ_14BIT_2G]).await;

    // 4. LPS22HH (기압/온도): WHO_AM_I=0x0F -> 0xB3
    if let Err(e) = i2c.write_read(ADDR_LPS22HH, &[REG_WHO_AM_I], &mut who).await {
        error!("LPS22HH WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!("  [LPS22HH 기압계] WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})", who[0], ID_LPS22HH);
    }
    let _ = i2c.write(ADDR_LPS22HH, &[lps22hh::CTRL_REG1, lps22hh::VAL_CTRL_REG1_1HZ_BDU]).await;

    // 5. STTS751 (고정밀 온도계): Product ID(0xFD)=0x01
    if let Err(e) = i2c.write_read(ADDR_STTS751, &[REG_STTS751_PRODUCT_ID], &mut who).await {
        error!("STTS751 Product ID 읽기 실패: {:?}", e);
    } else {
        info!("  [STTS751 정밀온도] Product ID: 0x{:02X} (기대값: 0x{:02X})", who[0], ID_STTS751);
    }
    let _ = i2c.write(ADDR_STTS751, &[stts751::CONFIG, stts751::VAL_CONFIG_CONTINUOUS]).await;
    let _ = i2c.write(ADDR_STTS751, &[stts751::CONVERSION_RATE, stts751::VAL_RATE_1_CONV_PER_SEC]).await;

    // 6. HTS221 (온습도계): WHO_AM_I=0x0F -> 0xBC
    if let Err(e) = i2c.write_read(ADDR_HTS221, &[REG_WHO_AM_I], &mut who).await {
        error!("HTS221 WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!("  [HTS221 온습도계] WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})", who[0], ID_HTS221);
    }
    let _ = i2c.write(ADDR_HTS221, &[hts221::AV_CONF, hts221::VAL_AV_CONF_DEFAULT]).await;
    let _ = i2c.write(ADDR_HTS221, &[hts221::CTRL_REG1, hts221::VAL_CTRL_REG1_1HZ_PD_BDU]).await;

    // HTS221 공장 캘리브레이션 계수 16바이트 읽기 (0x30~0x3F)
    let mut calib_buf = [0u8; 16];
    if let Ok(_) = i2c.write_read(ADDR_HTS221, &[hts221::CALIB_H0_RH_X2], &mut calib_buf).await {
        let calib = Hts221Calibration::from_raw_registers(&calib_buf);
        let mut c_guard = HTS221_CALIB.lock().await;
        *c_guard = calib;
        info!("  [HTS221 캘리브레이션] OTP 파라미터 로드 완료 (T0: {}x8, T1: {}x8, H0: {}x2, H1: {}x2)",
            calib.t0_degc_x8, calib.t1_degc_x8, calib.h0_rh_x2, calib.h1_rh_x2
        );
    } else {
        error!("HTS221 OTP 캘리브레이션 데이터 로드 실패!");
    }
}

/// [Task 1: 100 Hz] 고우선순위 선점형 IMU 실시간 모션 태스크 (10ms 주기)
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
            // 1. LSM6DSO 가속도/자이로 12바이트 버스트 비동기 읽기 (0x22 OUTX_L_G ~ 0x2D OUTZ_H_A)
            if i2c.write_read(ADDR_LSM6DSO, &[lsm6dso::OUTX_L_G], &mut buf).await.is_ok() {
                let gx = i16::from_le_bytes([buf[0], buf[1]]);
                let gy = i16::from_le_bytes([buf[2], buf[3]]);
                let gz = i16::from_le_bytes([buf[4], buf[5]]);
                let ax = i16::from_le_bytes([buf[6], buf[7]]);
                let ay = i16::from_le_bytes([buf[8], buf[9]]);
                let az = i16::from_le_bytes([buf[10], buf[11]]);

                // 감도 변환 모듈 적용
                let accel_mg = [
                    lsm6dso::raw_to_mg(ax),
                    lsm6dso::raw_to_mg(ay),
                    lsm6dso::raw_to_mg(az),
                ];
                let gyro_dps = [
                    lsm6dso::raw_to_dps(gx),
                    lsm6dso::raw_to_dps(gy),
                    lsm6dso::raw_to_dps(gz),
                ];

                // 2. LIS2DW12 보조 가속도계 6바이트 비동기 읽기 (0x28 OUT_X_L)
                let mut buf_dw = [0u8; 6];
                let mut accel2_mg = [0i16; 3];
                if i2c.write_read(ADDR_LIS2DW12, &[lis2dw12::OUT_X_L], &mut buf_dw).await.is_ok() {
                    let a2x = i16::from_le_bytes([buf_dw[0], buf_dw[1]]);
                    let a2y = i16::from_le_bytes([buf_dw[2], buf_dw[3]]);
                    let a2z = i16::from_le_bytes([buf_dw[4], buf_dw[5]]);
                    accel2_mg = [
                        lis2dw12::raw_to_mg(a2x),
                        lis2dw12::raw_to_mg(a2y),
                        lis2dw12::raw_to_mg(a2z),
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
            // LIS2MDL 6바이트 비동기 읽기
            if i2c.write_read(ADDR_LIS2MDL, &[lis2mdl::OUTX_L_REG], &mut buf).await.is_ok() {
                let mx = i16::from_le_bytes([buf[0], buf[1]]);
                let my = i16::from_le_bytes([buf[2], buf[3]]);
                let mz = i16::from_le_bytes([buf[4], buf[5]]);

                let mag = [
                    lis2mdl::raw_to_mgauss(mx),
                    lis2mdl::raw_to_mgauss(my),
                    lis2mdl::raw_to_mgauss(mz),
                ];

                let mut state = SENSOR_STATE.lock().await;
                state.mag_mgauss = mag;
                state.mag_sample_count += 1;
            }
        }
    }
}

/// [Task 3: 1 Hz] 저속 환경 센서 샘플링 (1000ms 주기, OTP 정규 캘리브레이션 적용)
#[embassy_executor::task]
async fn task_env_1hz() {
    let mut ticker = Ticker::every(Duration::from_hz(1)); // 1 Hz (1000ms)

    loop {
        ticker.next().await;

        // 1. LPS22HH 기압(3B) 및 온도(2B)
        let mut p_hpa_x10 = 0u32;
        let mut p_temp_x10 = 0i16;
        {
            let mut bus = I2C_BUS.lock().await;
            if let Some(i2c) = bus.as_mut() {
                let mut press_buf = [0u8; 5];
                if i2c.write_read(ADDR_LPS22HH, &[lps22hh::PRESS_OUT_XL], &mut press_buf).await.is_ok() {
                    let raw_press = (press_buf[0] as u32)
                        | ((press_buf[1] as u32) << 8)
                        | ((press_buf[2] as u32) << 16);
                    let raw_temp = i16::from_le_bytes([press_buf[3], press_buf[4]]);
                    p_hpa_x10 = lps22hh::raw_to_hpa_x10(raw_press);
                    p_temp_x10 = lps22hh::raw_to_temp_x10(raw_temp);
                }
            }
        }
        Timer::after_micros(50).await;

        // 2. STTS751 고정밀 온도계 (0x00 High, 0x02 Low)
        let mut s_temp_x10 = 0i16;
        {
            let mut bus = I2C_BUS.lock().await;
            if let Some(i2c) = bus.as_mut() {
                let mut stts_high = [0u8; 1];
                let mut stts_low = [0u8; 1];
                if i2c.write_read(ADDR_STTS751, &[stts751::TEMP_HIGH], &mut stts_high).await.is_ok()
                    && i2c.write_read(ADDR_STTS751, &[stts751::TEMP_LOW], &mut stts_low).await.is_ok()
                {
                    s_temp_x10 = stts751::raw_to_temp_x10(stts_high[0], stts_low[0]);
                }
            }
        }
        Timer::after_micros(50).await;

        // 3. HTS221 온습도계: 공장 출하 OTP 캘리브레이션 1차 선형 보간 적용
        let mut h_hum_x10 = 0u16;
        let mut h_temp_x10 = 0i16;
        {
            let mut bus = I2C_BUS.lock().await;
            if let Some(i2c) = bus.as_mut() {
                let mut hts_h_buf = [0u8; 2];
                let mut hts_t_buf = [0u8; 2];
                if i2c.write_read(ADDR_HTS221, &[hts221::HUMIDITY_OUT_L], &mut hts_h_buf).await.is_ok()
                    && i2c.write_read(ADDR_HTS221, &[hts221::TEMP_OUT_L], &mut hts_t_buf).await.is_ok()
                {
                    let raw_h = i16::from_le_bytes(hts_h_buf);
                    let raw_t = i16::from_le_bytes(hts_t_buf);

                    let calib = *HTS221_CALIB.lock().await;
                    h_hum_x10 = calib.compensate_humidity_x10(raw_h);
                    h_temp_x10 = calib.compensate_temp_x10(raw_t);
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
        info!("  [ENV   1Hz (누적 {}회)] Press: {}.{} hPa (LPS22HH) | Temp: {}.{} °C (STTS751) | HTS221: {}.{} % rH, {}.{} °C",
            snap.env_sample_count,
            snap.press_hpa_x10 / 10, snap.press_hpa_x10 % 10,
            snap.stts_temp_c_x10 / 10, (snap.stts_temp_c_x10 % 10).abs(),
            snap.hts_humidity_x10 / 10, snap.hts_humidity_x10 % 10,
            snap.hts_temp_c_x10 / 10, (snap.hts_temp_c_x10 % 10).abs(),
        );
        info!("----------------------------------------------------------------------------------");
    }
}
