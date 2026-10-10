#![no_std]
#![no_main]

use core::fmt::Write as _;
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::eth::generic_smi::GenericSMI;
use embassy_stm32::eth::{self, Ethernet, PacketQueue};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::peripherals::{ETH, I2C1};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::Write as _;
use heapless::String;
use nucleo_bsp::iks01a3::*;
use nucleo_bsp::uid;
use nucleo_bsp::{BoardLeds, BoardRmiiPins, I2C_FAST_MODE_HZ};
use static_cell::StaticCell;

// 1. 하드웨어 인터럽트 바인딩 (I2C1 + ETH)
bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
    ETH => eth::InterruptHandler;
});

// 2. 고우선순위 선점형 InterruptExecutor (NVIC CEC IRQ 바인딩)
use embassy_executor::InterruptExecutor;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[embassy_stm32::interrupt]
unsafe fn CEC() {
    EXECUTOR_HIGH.on_interrupt();
}

// 3. I2C 버스 뮤텍스
type I2cBus = Mutex<CriticalSectionRawMutex, Option<I2c<'static, embassy_stm32::mode::Async>>>;
static I2C_BUS: I2cBus = Mutex::new(None);

/// HTS221 공장 캘리브레이션 파라미터 저장소
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

// 4. 통합 센서 계측 데이터 스냅샷 구조체
#[derive(Copy, Clone, Default)]
pub struct SensorSnapshot {
    // [IMU 100Hz 선점형 실시간 갱신]
    pub lsm_accel_mg: [i16; 3],    // LSM6DSO 가속도 X, Y, Z (mg)
    pub lsm_gyro_dps: [i16; 3],    // LSM6DSO 각속도 X, Y, Z (dps)
    pub lis2dw_accel_mg: [i16; 3], // LIS2DW12 보조 가속도 X, Y, Z (mg)
    pub imu_sample_count: u32,
    pub imu_dt_us: u32,     // 실측 루프 주기 (목표: 10,000 us)
    pub imu_min_dt_us: u32, // 최소 주기
    pub imu_max_dt_us: u32, // 최대 주기

    // [MAG 10Hz 갱신]
    pub mag_mgauss: [i16; 3], // LIS2MDL 지자기 X, Y, Z (mgauss)
    pub mag_sample_count: u32,

    // [ENV 1Hz 갱신]
    pub press_hpa_x10: u32,    // LPS22HH 기압 (hPa * 10)
    pub press_temp_c_x10: i16, // LPS22HH 온도 (°C * 10)
    pub stts_temp_c_x10: i16,  // STTS751 정밀 온도 (°C * 10)
    pub hts_humidity_x10: u16, // HTS221 습도 (% rH * 10)
    pub hts_temp_c_x10: i16,   // HTS221 온도 (°C * 10)
    pub env_sample_count: u32,

    // [네트워크/서버 통계]
    pub http_request_count: u32,
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
    http_request_count: 0,
});

// 5. 이더넷 패킷 큐 및 네트워크 스택 리소스 (StaticCell 기반 무손실 안전 정적 할당)
static PACKET_QUEUE: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
static STACK_RESOURCES: StaticCell<embassy_net::StackResources<4>> = StaticCell::new();

type Device = Ethernet<'static, ETH, GenericSMI>;

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}

// 6. 메인 진입점
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut leds = BoardLeds::new(p.PB0, p.PE1, p.PB14);
    leds.green.set_high(); // 부팅 표시

    info!("============================================================");
    info!("NUCLEO-H743ZI2 DHCP Ethernet & Sensor Web Dashboard (03)");
    info!("============================================================");

    // [1] I2C1 400kHz Fast Mode DMA 비동기 초기화 (PB8/PB9)
    let i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        I2C_FAST_MODE_HZ,
        Default::default(),
    );
    {
        let mut bus = I2C_BUS.lock().await;
        *bus = Some(i2c);
    }
    info!("I2C1 400kHz DMA 비동기 버스 초기화 완료 (PB8/PB9)");

    // [2] X-NUCLEO-IKS01A3 6종 센서 시그니처 검증 및 고속 오버샘플링 초기화
    init_sensors().await;

    // [3] 이더넷 RMII LAN8742A 드라이버 초기화 (STM32 고유 UID 기반 EUI-48 MAC 주소 생성)
    let mac_addr = uid::get_unique_mac_address();
    let queue = PACKET_QUEUE.init(PacketQueue::new());
    let rmii_pins = BoardRmiiPins::new(
        p.PA1,  // ref_clk
        p.PA2,  // mdio
        p.PC1,  // mdc
        p.PA7,  // crs
        p.PC4,  // rx_d0
        p.PC5,  // rx_d1
        p.PG13, // tx_d0
        p.PB13, // tx_d1
        p.PG11, // tx_en
    );

    let eth_device = Ethernet::new(
        queue,
        p.ETH,
        Irqs,
        rmii_pins.ref_clk,
        rmii_pins.mdio,
        rmii_pins.mdc,
        rmii_pins.crs_dv,
        rmii_pins.rx_d0,
        rmii_pins.rx_d1,
        rmii_pins.tx_d0,
        rmii_pins.tx_d1,
        rmii_pins.tx_en,
        GenericSMI::new(0),
        mac_addr,
    );
    info!("LAN8742A RMII 이더넷 드라이버 초기화 완료 (STM32 UID MAC: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X})",
        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]
    );

    // [4] embassy-net TCP/IP 스택 및 DHCPv4 클라이언트 구성 (UID 파생 난수 시드)
    let net_config = embassy_net::Config::dhcpv4(Default::default());
    let seed = uid::get_uid_prng_seed();
    let resources = STACK_RESOURCES.init(embassy_net::StackResources::new());
    let (stack, runner) = embassy_net::new(eth_device, net_config, resources, seed);

    spawner.must_spawn(net_task(runner));

    // [5] 2계층 실시간 센서 샘플링 파이프라인 스폰
    interrupt::CEC.set_priority(Priority::P6);
    let high_spawner = EXECUTOR_HIGH.start(interrupt::CEC);
    high_spawner.must_spawn(task_imu_100hz());

    spawner.must_spawn(task_mag_10hz());
    spawner.must_spawn(task_env_1hz());

    // [6] 비동기 HTTP 웹서버 및 대시보드 리포터 스폰
    spawner.must_spawn(task_web_server(stack));
    spawner.must_spawn(task_dashboard_reporter(stack));

    leds.green.set_low();
}

/// 6종 센서의 WHO_AM_I 검증 및 레지스터 설정
async fn init_sensors() {
    let mut bus = I2C_BUS.lock().await;
    let i2c = bus.as_mut().expect("I2C 버스 미초기화");

    // 1. LSM6DSO (6축 IMU): 416Hz ODR + LPF2 (41.6Hz Cutoff)
    let mut who = [0u8; 1];
    if let Err(e) = i2c
        .write_read(ADDR_LSM6DSO, &[REG_WHO_AM_I], &mut who)
        .await
    {
        error!("LSM6DSO WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!(
            "  [LSM6DSO 6축 IMU] WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})",
            who[0], ID_LSM6DSO
        );
    }
    let _ = i2c
        .write(
            ADDR_LSM6DSO,
            &[lsm6dso::CTRL1_XL, lsm6dso::VAL_CTRL1_XL_416HZ_2G_LPF2],
        )
        .await;
    let _ = i2c
        .write(
            ADDR_LSM6DSO,
            &[lsm6dso::CTRL2_G, lsm6dso::VAL_CTRL2_G_416HZ_250DPS],
        )
        .await;
    let _ = i2c
        .write(
            ADDR_LSM6DSO,
            &[lsm6dso::CTRL8_XL, lsm6dso::VAL_CTRL8_XL_LPF2_ODR_DIV_10],
        )
        .await;

    // 2. LIS2MDL (3축 지자기)
    if let Err(e) = i2c
        .write_read(ADDR_LIS2MDL, &[REG_LIS2MDL_WHO_AM_I], &mut who)
        .await
    {
        error!("LIS2MDL WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!(
            "  [LIS2MDL 지자기] WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})",
            who[0], ID_LIS2MDL
        );
    }
    let _ = i2c
        .write(
            ADDR_LIS2MDL,
            &[lis2mdl::CFG_REG_A, lis2mdl::VAL_CFG_REG_A_10HZ_CONT],
        )
        .await;
    let _ = i2c
        .write(
            ADDR_LIS2MDL,
            &[lis2mdl::CFG_REG_C, lis2mdl::VAL_CFG_REG_C_BDU],
        )
        .await;

    // 3. LIS2DW12 (보조 가속도계)
    if let Err(e) = i2c
        .write_read(ADDR_LIS2DW12, &[REG_WHO_AM_I], &mut who)
        .await
    {
        error!("LIS2DW12 WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!(
            "  [LIS2DW12 보조 가속도] WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})",
            who[0], ID_LIS2DW12
        );
    }
    let _ = i2c
        .write(
            ADDR_LIS2DW12,
            &[lis2dw12::CTRL1, lis2dw12::VAL_CTRL1_200HZ_14BIT_2G],
        )
        .await;

    // 4. LPS22HH (기압/온도)
    if let Err(e) = i2c
        .write_read(ADDR_LPS22HH, &[REG_WHO_AM_I], &mut who)
        .await
    {
        error!("LPS22HH WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!(
            "  [LPS22HH 기압계] WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})",
            who[0], ID_LPS22HH
        );
    }
    let _ = i2c
        .write(
            ADDR_LPS22HH,
            &[lps22hh::CTRL_REG1, lps22hh::VAL_CTRL_REG1_1HZ_BDU],
        )
        .await;

    // 5. STTS751 (정밀 온도계)
    if let Err(e) = i2c
        .write_read(ADDR_STTS751, &[REG_STTS751_PRODUCT_ID], &mut who)
        .await
    {
        error!("STTS751 Product ID 읽기 실패: {:?}", e);
    } else {
        info!(
            "  [STTS751 정밀온도] Product ID: 0x{:02X} (기대값: 0x{:02X})",
            who[0], ID_STTS751
        );
    }
    let _ = i2c
        .write(
            ADDR_STTS751,
            &[stts751::CONFIG, stts751::VAL_CONFIG_CONTINUOUS],
        )
        .await;
    let _ = i2c
        .write(
            ADDR_STTS751,
            &[stts751::CONVERSION_RATE, stts751::VAL_RATE_1_CONV_PER_SEC],
        )
        .await;

    // 6. HTS221 (온습도계)
    if let Err(e) = i2c.write_read(ADDR_HTS221, &[REG_WHO_AM_I], &mut who).await {
        error!("HTS221 WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!(
            "  [HTS221 온습도계] WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})",
            who[0], ID_HTS221
        );
    }
    let _ = i2c
        .write(ADDR_HTS221, &[hts221::AV_CONF, hts221::VAL_AV_CONF_DEFAULT])
        .await;
    let _ = i2c
        .write(
            ADDR_HTS221,
            &[hts221::CTRL_REG1, hts221::VAL_CTRL_REG1_1HZ_PD_BDU],
        )
        .await;

    // HTS221 공장 캘리브레이션 계수 16바이트 읽기
    let mut calib_buf = [0u8; 16];
    if i2c
        .write_read(ADDR_HTS221, &[hts221::CALIB_H0_RH_X2], &mut calib_buf)
        .await
        .is_ok()
    {
        let calib = Hts221Calibration::from_raw_registers(&calib_buf);
        let mut c_guard = HTS221_CALIB.lock().await;
        *c_guard = calib;
        info!("  [HTS221 캘리브레이션] OTP 파라미터 로드 완료");
    }
}

/// [Task 1: 100 Hz] 고우선순위 선점형 IMU 실시간 모션 태스크 (비동기 I2C 읽기)
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
            // 1. LSM6DSO 12바이트 버스트 비동기 읽기
            if i2c
                .write_read(ADDR_LSM6DSO, &[lsm6dso::OUTX_L_G], &mut buf)
                .await
                .is_ok()
            {
                let gx = i16::from_le_bytes([buf[0], buf[1]]);
                let gy = i16::from_le_bytes([buf[2], buf[3]]);
                let gz = i16::from_le_bytes([buf[4], buf[5]]);
                let ax = i16::from_le_bytes([buf[6], buf[7]]);
                let ay = i16::from_le_bytes([buf[8], buf[9]]);
                let az = i16::from_le_bytes([buf[10], buf[11]]);

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

                // 2. LIS2DW12 보조 가속도계 6바이트 비동기 읽기
                let mut buf_dw = [0u8; 6];
                let mut accel2_mg = [0i16; 3];
                if i2c
                    .write_read(ADDR_LIS2DW12, &[lis2dw12::OUT_X_L], &mut buf_dw)
                    .await
                    .is_ok()
                {
                    let a2x = i16::from_le_bytes([buf_dw[0], buf_dw[1]]);
                    let a2y = i16::from_le_bytes([buf_dw[2], buf_dw[3]]);
                    let a2z = i16::from_le_bytes([buf_dw[4], buf_dw[5]]);
                    accel2_mg = [
                        lis2dw12::raw_to_mg(a2x),
                        lis2dw12::raw_to_mg(a2y),
                        lis2dw12::raw_to_mg(a2z),
                    ];
                }

                let mut state = SENSOR_STATE.lock().await;
                state.lsm_accel_mg = accel_mg;
                state.lsm_gyro_dps = gyro_dps;
                state.lis2dw_accel_mg = accel2_mg;
                state.imu_sample_count += 1;
                state.imu_dt_us = dt_us;
                state.imu_min_dt_us = if min_dt_us == u32::MAX {
                    dt_us
                } else {
                    min_dt_us
                };
                state.imu_max_dt_us = max_dt_us;
            }
        }
    }
}

/// [Task 2: 10 Hz] 중속 지자기 센서 비동기 샘플링 (100ms 주기)
#[embassy_executor::task]
async fn task_mag_10hz() {
    let mut ticker = Ticker::every(Duration::from_hz(10));
    let mut buf = [0u8; 6];

    loop {
        ticker.next().await;

        let mut bus = I2C_BUS.lock().await;
        if let Some(i2c) = bus.as_mut() {
            if i2c
                .write_read(ADDR_LIS2MDL, &[lis2mdl::OUTX_L_REG], &mut buf)
                .await
                .is_ok()
            {
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

/// [Task 3: 1 Hz] 저속 환경 센서 비동기 샘플링 (1000ms 주기, OTP 캘리브레이션)
#[embassy_executor::task]
async fn task_env_1hz() {
    let mut ticker = Ticker::every(Duration::from_hz(1));

    loop {
        ticker.next().await;

        // 1. LPS22HH 기압 및 온도
        let mut p_hpa_x10 = 0u32;
        let mut p_temp_x10 = 0i16;
        {
            let mut bus = I2C_BUS.lock().await;
            if let Some(i2c) = bus.as_mut() {
                let mut press_buf = [0u8; 5];
                if i2c
                    .write_read(ADDR_LPS22HH, &[lps22hh::PRESS_OUT_XL], &mut press_buf)
                    .await
                    .is_ok()
                {
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

        // 2. STTS751 정밀 온도계
        let mut s_temp_x10 = 0i16;
        {
            let mut bus = I2C_BUS.lock().await;
            if let Some(i2c) = bus.as_mut() {
                let mut stts_high = [0u8; 1];
                let mut stts_low = [0u8; 1];
                if i2c
                    .write_read(ADDR_STTS751, &[stts751::TEMP_HIGH], &mut stts_high)
                    .await
                    .is_ok()
                    && i2c
                        .write_read(ADDR_STTS751, &[stts751::TEMP_LOW], &mut stts_low)
                        .await
                        .is_ok()
                {
                    s_temp_x10 = stts751::raw_to_temp_x10(stts_high[0], stts_low[0]);
                }
            }
        }
        Timer::after_micros(50).await;

        // 3. HTS221 온습도계: OTP 정규 캘리브레이션 1차 선형 보간 적용
        let mut h_hum_x10 = 0u16;
        let mut h_temp_x10 = 0i16;
        {
            let mut bus = I2C_BUS.lock().await;
            if let Some(i2c) = bus.as_mut() {
                let mut hts_h_buf = [0u8; 2];
                let mut hts_t_buf = [0u8; 2];
                if i2c
                    .write_read(ADDR_HTS221, &[hts221::HUMIDITY_OUT_L], &mut hts_h_buf)
                    .await
                    .is_ok()
                    && i2c
                        .write_read(ADDR_HTS221, &[hts221::TEMP_OUT_L], &mut hts_t_buf)
                        .await
                        .is_ok()
                {
                    let raw_h = i16::from_le_bytes(hts_h_buf);
                    let raw_t = i16::from_le_bytes(hts_t_buf);

                    let calib = *HTS221_CALIB.lock().await;
                    h_hum_x10 = calib.compensate_humidity_x10(raw_h);
                    h_temp_x10 = calib.compensate_temp_x10(raw_t);
                }
            }
        }

        let mut state = SENSOR_STATE.lock().await;
        state.press_hpa_x10 = p_hpa_x10;
        state.press_temp_c_x10 = p_temp_x10;
        state.stts_temp_c_x10 = s_temp_x10;
        state.hts_humidity_x10 = h_hum_x10;
        state.hts_temp_c_x10 = h_temp_x10;
        state.env_sample_count += 1;
    }
}

/// [Task 4: Port 80 HTTP 웹서버] GET /api/sensors (JSON) 및 GET / (모던 대시보드 UI) 서빙
#[embassy_executor::task]
async fn task_web_server(stack: embassy_net::Stack<'static>) -> ! {
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 2048];
    let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(5)));

    loop {
        socket.abort();
        if let Err(e) = socket.accept(80).await {
            warn!("TCP accept 에러: {:?}", defmt::Debug2Format(&e));
            Timer::after_millis(50).await;
            continue;
        }

        let mut buf = [0u8; 1024];
        let n = match socket.read(&mut buf).await {
            Ok(0) => {
                socket.close();
                continue;
            }
            Ok(n) => n,
            Err(_) => {
                socket.abort();
                continue;
            }
        };

        {
            let mut state = SENSOR_STATE.lock().await;
            state.http_request_count += 1;
        }

        let req = core::str::from_utf8(&buf[..n]).unwrap_or("");
        if req.starts_with("GET /api/sensors") {
            let snap = *SENSOR_STATE.lock().await;
            let mut json = String::<1024>::new();
            let _ = write!(
                json,
                concat!(
                    "{{\"imu\":{{\"ax\":{},\"ay\":{},\"az\":{},",
                    "\"gx\":{},\"gy\":{},\"gz\":{},\"count\":{},\"dt_us\":{},\"min_dt\":{},\"max_dt\":{}}},",
                    "\"aux\":{{\"ax\":{},\"ay\":{},\"az\":{}}},",
                    "\"mag\":{{\"mx\":{},\"my\":{},\"mz\":{},\"count\":{}}},",
                    "\"env\":{{\"press_hpa\":{}.{},\"stts_temp_c\":{}.{},\"hts_humidity\":{}.{},\"hts_temp_c\":{}.{},\"count\":{}}},",
                    "\"server\":{{\"requests\":{}}}}}",
                ),
                snap.lsm_accel_mg[0], snap.lsm_accel_mg[1], snap.lsm_accel_mg[2],
                snap.lsm_gyro_dps[0], snap.lsm_gyro_dps[1], snap.lsm_gyro_dps[2],
                snap.imu_sample_count, snap.imu_dt_us, snap.imu_min_dt_us, snap.imu_max_dt_us,
                snap.lis2dw_accel_mg[0], snap.lis2dw_accel_mg[1], snap.lis2dw_accel_mg[2],
                snap.mag_mgauss[0], snap.mag_mgauss[1], snap.mag_mgauss[2], snap.mag_sample_count,
                snap.press_hpa_x10 / 10, snap.press_hpa_x10 % 10,
                snap.stts_temp_c_x10 / 10, (snap.stts_temp_c_x10 % 10).abs(),
                snap.hts_humidity_x10 / 10, snap.hts_humidity_x10 % 10,
                snap.hts_temp_c_x10 / 10, (snap.hts_temp_c_x10 % 10).abs(),
                snap.env_sample_count,
                snap.http_request_count,
            );

            let mut header = String::<256>::new();
            let _ = write!(
                header,
                "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                json.len()
            );
            let _ = socket.write_all(header.as_bytes()).await;
            let _ = socket.write_all(json.as_bytes()).await;
        } else {
            // GET /: 모던 글래스모피즘 센서 대시보드
            let body = DASHBOARD_HTML;
            let mut header = String::<256>::new();
            let _ = write!(
                header,
                "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                body.len()
            );
            let _ = socket.write_all(header.as_bytes()).await;
            let _ = socket.write_all(body.as_bytes()).await;
        }

        let _ = socket.flush().await;
        socket.close();
        socket.abort();
    }
}

/// [Task 5: 1 Hz 대시보드 리포터] DHCP IP 주소 및 센서 상태 RTT 콘솔 출력
#[embassy_executor::task]
async fn task_dashboard_reporter(stack: embassy_net::Stack<'static>) {
    let mut ticker = Ticker::every(Duration::from_hz(1));
    let mut seq: u32 = 0;
    let mut dhcp_notified = false;

    loop {
        ticker.next().await;
        seq += 1;

        if stack.is_config_up() {
            if !dhcp_notified {
                dhcp_notified = true;
                if let Some(cfg) = stack.config_v4() {
                    info!("============================================================");
                    info!("★ DHCP IP 주소 할당 완료! ★");
                    info!("  -> IP 주소: {:?}", cfg.address);
                    info!(
                        "  -> 웹 브라우저에서 http://{:?} 접속 가능",
                        cfg.address.address()
                    );
                    if let Some(gw) = cfg.gateway {
                        info!("  -> 게이트웨이: {:?}", gw);
                    }
                    info!("============================================================");
                }
            }
        } else {
            info!("이더넷 링크 대기 또는 DHCP IP 요청 중... (시퀀스 #{})", seq);
            continue;
        }

        let snap = *SENSOR_STATE.lock().await;
        info!(
            "[WebReport #{}] RT-IMU 100Hz: dt={}us (min={}, max={}) | HTTP Req: {} | Accel: [{}, {}, {}] mg | Gyro: [{}, {}, {}] dps",
            seq, snap.imu_dt_us, snap.imu_min_dt_us, snap.imu_max_dt_us, snap.http_request_count,
            snap.lsm_accel_mg[0], snap.lsm_accel_mg[1], snap.lsm_accel_mg[2],
            snap.lsm_gyro_dps[0], snap.lsm_gyro_dps[1], snap.lsm_gyro_dps[2],
        );
    }
}

// 7. 완전 독립형 글래스모피즘 웹 대시보드 HTML/CSS/JS
const DASHBOARD_HTML: &str = r#"<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>NUCLEO-H743ZI2 Sensor Telemetry</title>
<style>
:root{--bg:#0b0f19;--card:rgba(20,28,45,0.7);--border:rgba(255,255,255,0.08);--accent:#38bdf8;--acc-grad:linear-gradient(135deg,#38bdf8,#818cf8);--text:#f1f5f9;--sub:#94a3b8;--green:#34d399;--yellow:#fbbf24}
*{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
body{background:radial-gradient(ellipse at top,#1e293b 0%,var(--bg) 100%);color:var(--text);min-height:100vh;padding:24px 16px}
.container{max-width:1100px;margin:0 auto}
header{display:flex;justify-content:space-between;align-items:center;margin-bottom:24px;padding-bottom:16px;border-bottom:1px solid var(--border)}
.logo-title{display:flex;align-items:center;gap:12px}
.logo-badge{background:var(--acc-grad);color:#fff;font-weight:700;padding:6px 12px;border-radius:8px;font-size:14px;letter-spacing:0.5px}
h1{font-size:22px;font-weight:700;color:#fff}
.status-pill{display:flex;align-items:center;gap:8px;background:rgba(52,211,153,0.1);color:var(--green);border:1px solid rgba(52,211,153,0.3);padding:6px 14px;border-radius:20px;font-size:13px;font-weight:600}
.pulse{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green);animation:blink 1.5s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:0.3}}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:16px;margin-bottom:20px}
.card{background:var(--card);backdrop-filter:blur(16px);border:1px solid var(--border);border-radius:14px;padding:20px;box-shadow:0 8px 32px rgba(0,0,0,0.37)}
.card-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:16px}
.card-title{font-size:15px;font-weight:700;color:var(--accent);text-transform:uppercase;letter-spacing:0.5px}
.card-rate{font-size:12px;background:rgba(255,255,255,0.06);padding:3px 8px;border-radius:6px;color:var(--sub)}
.metric-row{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px}
.metric-label{color:var(--sub);font-size:13px}
.metric-val{font-size:17px;font-weight:700;color:#fff;font-variant-numeric:tabular-nums}
.unit{font-size:12px;color:var(--sub);font-weight:400;margin-left:2px}
.bar-track{width:100%;height:6px;background:rgba(255,255,255,0.08);border-radius:3px;overflow:hidden;margin-top:4px}
.bar-fill{height:100%;background:var(--acc-grad);border-radius:3px;transition:width 0.2s ease}
.bar-center{display:flex;width:100%;height:6px;background:rgba(255,255,255,0.08);border-radius:3px;margin-top:4px;position:relative}
.bar-center-fill{height:100%;background:#818cf8;border-radius:3px;position:absolute;transition:all 0.2s ease}
.footer{text-align:center;color:var(--sub);font-size:12px;margin-top:28px}
</style>
</head>
<body>
<div class="container">
<header>
<div class="logo-title">
<span class="logo-badge">STM32H743</span>
<h1>X-NUCLEO-IKS01A3 Real-Time Dashboard</h1>
</div>
<div class="status-pill"><div class="pulse"></div><span id="conn-status">LIVE TELEMETRY</span></div>
</header>
<div class="grid">
<!-- IMU Card -->
<div class="card">
<div class="card-header"><span class="card-title">LSM6DSO 6축 IMU</span><span class="card-rate">100 Hz (Preemptive)</span></div>
<div class="metric-row"><span class="metric-label">루프 주기 (dt)</span><span class="metric-val" id="imu-dt">10000<span class="unit">µs</span></span></div>
<div class="metric-row"><span class="metric-label">최소 / 최대 주기</span><span class="metric-val" id="imu-jitter">10000 / 10000<span class="unit">µs</span></span></div>
<div style="margin-top:14px">
<div class="metric-row"><span class="metric-label">가속도 X / Y / Z</span><span class="metric-val" id="imu-accel">0, 0, 0<span class="unit">mg</span></span></div>
<div class="bar-center"><div id="bar-ax" class="bar-center-fill"></div></div>
<div class="metric-row" style="margin-top:12px"><span class="metric-label">각속도 X / Y / Z</span><span class="metric-val" id="imu-gyro">0, 0, 0<span class="unit">dps</span></span></div>
<div class="bar-center"><div id="bar-gx" class="bar-center-fill"></div></div>
</div>
</div>
<!-- MAG & AUX Card -->
<div class="card">
<div class="card-header"><span class="card-title">LIS2MDL 지자기 &amp; LIS2DW12</span><span class="card-rate">10 Hz / 100 Hz</span></div>
<div class="metric-row"><span class="metric-label">지자기 X / Y / Z</span><span class="metric-val" id="mag-val">0, 0, 0<span class="unit">mgauss</span></span></div>
<div class="bar-center"><div id="bar-mx" class="bar-center-fill"></div></div>
<div style="margin-top:16px">
<div class="metric-row"><span class="metric-label">보조 가속도계 (LIS2DW12)</span><span class="metric-val" id="aux-accel">0, 0, 0<span class="unit">mg</span></span></div>
<div class="bar-center"><div id="bar-aux" class="bar-center-fill"></div></div>
</div>
</div>
<!-- Environmental Card -->
<div class="card">
<div class="card-header"><span class="card-title">환경 센서 (기압/온습도)</span><span class="card-rate">1 Hz</span></div>
<div class="metric-row"><span class="metric-label">대기압 (LPS22HH)</span><span class="metric-val" id="env-press">1013.2<span class="unit">hPa</span></span></div>
<div class="bar-track"><div id="bar-press" class="bar-fill" style="width:50%"></div></div>
<div class="metric-row" style="margin-top:14px"><span class="metric-label">정밀 온도 (STTS751)</span><span class="metric-val" id="env-temp">24.5<span class="unit">°C</span></span></div>
<div class="bar-track"><div id="bar-temp" class="bar-fill" style="width:40%"></div></div>
<div class="metric-row" style="margin-top:14px"><span class="metric-label">상대 습도 (HTS221 OTP)</span><span class="metric-val" id="env-hum">45.0<span class="unit">% rH</span></span></div>
<div class="bar-track"><div id="bar-hum" class="bar-fill" style="width:45%"></div></div>
</div>
<!-- System Status Card -->
<div class="card">
<div class="card-header"><span class="card-title">시스템 런타임 &amp; 통계</span><span class="card-rate">실시간</span></div>
<div class="metric-row"><span class="metric-label">IMU 실시간 샘플링 누적</span><span class="metric-val" id="imu-count">0<span class="unit">회</span></span></div>
<div class="metric-row"><span class="metric-label">HTTP REST 요청 처리</span><span class="metric-val" id="http-req">0<span class="unit">회</span></span></div>
<div class="metric-row"><span class="metric-label">네트워크 프로토콜</span><span class="metric-val">DHCPv4 / HTTP 1.1</span></div>
<div class="metric-row"><span class="metric-label">하드웨어 아키텍처</span><span class="metric-val">Cortex-M7 480MHz</span></div>
</div>
</div>
<div class="footer">NUCLEO-H743ZI2 + X-NUCLEO-IKS01A3 Firmware &bull; Embassy Async Framework</div>
</div>
<script>
async function updateTelemetry(){
try{
const res=await fetch('/api/sensors');
if(!res.ok)throw new Error();
const d=await res.json();
document.getElementById('conn-status').textContent='LIVE TELEMETRY';
document.getElementById('imu-dt').innerHTML=`${d.imu.dt_us}<span class="unit">µs</span>`;
document.getElementById('imu-jitter').innerHTML=`${d.imu.min_dt} / ${d.imu.max_dt}<span class="unit">µs</span>`;
document.getElementById('imu-accel').innerHTML=`${d.imu.ax}, ${d.imu.ay}, ${d.imu.az}<span class="unit">mg</span>`;
document.getElementById('imu-gyro').innerHTML=`${d.imu.gx}, ${d.imu.gy}, ${d.imu.gz}<span class="unit">dps</span>`;
document.getElementById('mag-val').innerHTML=`${d.mag.mx}, ${d.mag.my}, ${d.mag.mz}<span class="unit">mgauss</span>`;
document.getElementById('aux-accel').innerHTML=`${d.aux.ax}, ${d.aux.ay}, ${d.aux.az}<span class="unit">mg</span>`;
document.getElementById('imu-count').innerHTML=`${d.imu.count}<span class="unit">회</span>`;
document.getElementById('env-press').innerHTML=`${d.env.press_hpa.toFixed(1)}<span class="unit">hPa</span>`;
document.getElementById('env-temp').innerHTML=`${d.env.stts_temp_c.toFixed(1)}<span class="unit">°C</span>`;
document.getElementById('env-hum').innerHTML=`${d.env.hts_humidity.toFixed(1)}<span class="unit">% rH</span>`;
document.getElementById('http-req').innerHTML=`${d.server.requests}<span class="unit">회</span>`;
const setCenterBar=(id,val,max)=>{const el=document.getElementById(id);const pct=Math.min(Math.abs(val)/max*50,50);if(val>=0){el.style.left='50%';el.style.width=pct+'%';el.style.background='#38bdf8'}else{el.style.left=(50-pct)+'%';el.style.width=pct+'%';el.style.background='#f43f5e'}};
setCenterBar('bar-ax',d.imu.ax,2000);
setCenterBar('bar-gx',d.imu.gx,250);
setCenterBar('bar-mx',d.mag.mx,1000);
setCenterBar('bar-aux',d.aux.ax,2000);
document.getElementById('bar-press').style.width=Math.min(Math.max((d.env.press_hpa-950)/(1050-950)*100,0),100)+'%';
document.getElementById('bar-temp').style.width=Math.min(Math.max(d.env.stts_temp_c/50*100,0),100)+'%';
document.getElementById('bar-hum').style.width=Math.min(Math.max(d.env.hts_humidity,0),100)+'%';
}catch(e){document.getElementById('conn-status').textContent='RECONNECTING...';}
}
setInterval(updateTelemetry,300);
updateTelemetry();
</script>
</body>
</html>"#;
