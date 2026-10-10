//! # NUCLEO-H743ZI2 SO(3) Right-Invariant InEKF AHRS & 3D Web Dashboard (04_ahrs_so3_inekf)
//!
//! Features:
//! - NVIC CEC IRQ (Priority P6) 100 Hz Preemptive RT-IMU loop with SO(3) Lie Group integration and constant-Jacobian accelerometer update
//! - 10 Hz LIS2MDL Magnetometer update
//! - LAN8742A RMII Ethernet driver with DHCPv4 auto IP configuration
//! - Embedded HTTP Web Server on port 80
//! - REST API: `GET /api/ahrs`
//! - Self-Contained Dark Glassmorphism 3D Web Dashboard with GPU-accelerated 3D NUCLEO board attitude visualizer

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
use so3_inekf::{InEKFConfig, RightInvariantInEKF};
use static_cell::StaticCell;

/// InEKF 최소/최대 시간 증분 클램핑 경계 (초)
const MIN_INTEGRATION_DT_S: f32 = 0.001;
const MAX_INTEGRATION_DT_S: f32 = 0.050;

/// 지자기 유효성 검증 최소 자기장 크기 제곱 임계값 (mgauss^2)
const MIN_VALID_MAG_NORM_SQ: f32 = 100.0;

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

// 리눅스 /proc/stat 틱 샘플링 통계 카운터 및 코어 상태 플래그
static IS_SLEEPING: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
static IS_RT_ACTIVE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
static TICK_IDLE_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static TICK_BUSY_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// [리눅스 /proc/stat 1 kHz 틱 샘플러 ISR]
/// 1ms마다 발생하는 하드웨어 타이머 인터럽트 시점의 CPU 실행 컨텍스트 스냅샷
#[embassy_stm32::interrupt]
unsafe fn TIM7() {
    let tim = embassy_stm32::pac::TIM7;
    tim.sr().write(|w| w.set_uif(false));

    // 코어가 WFE 저전력 슬립 중이고, RT 선점 인터럽트가 실행 중이지 않을 때만 순수 IDLE로 판정
    if IS_SLEEPING.load(core::sync::atomic::Ordering::Relaxed)
        && !IS_RT_ACTIVE.load(core::sync::atomic::Ordering::Relaxed)
    {
        TICK_IDLE_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    } else {
        TICK_BUSY_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
}

/// STM32H743 기본 타이머 TIM7을 1,000 Hz (1ms 주기) 틱 샘플러로 초기화
unsafe fn init_proc_stat_timer() {
    embassy_stm32::pac::RCC
        .apb1lenr()
        .modify(|w| w.set_tim7en(true));
    let tim = embassy_stm32::pac::TIM7;
    tim.cr1().write(|w| w.set_cen(false));
    tim.psc().write_value(63); // 64MHz -> 1MHz (1µs)
    tim.arr().write(|w| w.set_arr(999)); // 1MHz / 1000 = 1000Hz (1ms)
    tim.cnt().write(|w| w.set_cnt(0));
    tim.sr().write(|w| w.set_uif(false));
    tim.dier().write(|w| w.set_uie(true));
    tim.cr1().write(|w| w.set_cen(true));

    interrupt::TIM7.set_priority(Priority::P5);
    interrupt::TIM7.enable();
}

// 3. I2C 버스 뮤텍스
type I2cBus = Mutex<CriticalSectionRawMutex, Option<I2c<'static, embassy_stm32::mode::Async>>>;
static I2C_BUS: I2cBus = Mutex::new(None);

// 4. InEKF 필터 뮤텍스 (100Hz IMU 적분 및 10Hz 지자기 보정 공유)
type InEkfMutex = Mutex<CriticalSectionRawMutex, RightInvariantInEKF>;
static INEKF_FILTER: InEkfMutex = Mutex::new(RightInvariantInEKF::new());

// 5. 통합 AHRS 텔레메트리 스냅샷 구조체
#[derive(Copy, Clone)]
pub struct AhrsSnapshot {
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub quat: [f32; 4],
    pub rot_matrix: [[f32; 3]; 3],
    pub bias_dps: [f32; 3],
    pub cov_trace: f32,
    pub is_stationary: bool,
    pub imu_accel_mg: [i16; 3],
    pub imu_gyro_dps: [i16; 3],
    pub mag_mgauss: [i16; 3],
    pub sample_count: u32,
    pub imu_dt_us: u32,
    pub inekf_calc_us: u32,
    pub cpu_load_pct: f32,
    pub http_request_count: u32,
}

impl Default for AhrsSnapshot {
    fn default() -> Self {
        Self {
            roll_deg: 0.0,
            pitch_deg: 0.0,
            yaw_deg: 0.0,
            quat: [1.0, 0.0, 0.0, 0.0],
            rot_matrix: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            bias_dps: [0.0; 3],
            cov_trace: 0.0,
            is_stationary: false,
            imu_accel_mg: [0; 3],
            imu_gyro_dps: [0; 3],
            mag_mgauss: [0; 3],
            sample_count: 0,
            imu_dt_us: 10000,
            inekf_calc_us: 20,
            cpu_load_pct: 0.5,
            http_request_count: 0,
        }
    }
}

static AHRS_SNAPSHOT: Mutex<CriticalSectionRawMutex, AhrsSnapshot> = Mutex::new(AhrsSnapshot {
    roll_deg: 0.0,
    pitch_deg: 0.0,
    yaw_deg: 0.0,
    quat: [1.0, 0.0, 0.0, 0.0],
    rot_matrix: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    bias_dps: [0.0; 3],
    cov_trace: 0.0,
    is_stationary: false,
    imu_accel_mg: [0; 3],
    imu_gyro_dps: [0; 3],
    mag_mgauss: [0; 3],
    sample_count: 0,
    imu_dt_us: 10000,
    inekf_calc_us: 20,
    cpu_load_pct: 0.5,
    http_request_count: 0,
});

// 6. 이더넷 패킷 큐 및 네트워크 스택 리소스 (StaticCell 기반 안전 정적 할당)
static PACKET_QUEUE: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
static STACK_RESOURCES: StaticCell<embassy_net::StackResources<4>> = StaticCell::new();

type Device = Ethernet<'static, ETH, GenericSMI>;

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());
    info!(">>> NUCLEO-H743ZI2 SO(3) Right-Invariant InEKF AHRS 시작 <<<");

    // ARM Cortex-M7 DWT 하드웨어 사이클 카운터 활성화 (InEKF 마이크로초 정밀 프로파일링용)
    unsafe {
        let mut cp = cortex_m::peripheral::Peripherals::steal();
        cp.DCB.enable_trace();
        cortex_m::peripheral::DWT::unlock();
        cp.DWT.enable_cycle_counter();

        // 리눅스 /proc/stat 스타일 1 kHz 틱 샘플러 타이머 가동
        init_proc_stat_timer();
    }

    let executor = cortex_m::singleton!(: embassy_executor::raw::Executor = embassy_executor::raw::Executor::new(core::ptr::null_mut())).unwrap();
    let spawner = executor.spawner();

    spawner.must_spawn(main_task(spawner, p));

    // 리눅스 /proc/stat 무오버헤드 저전력 메인 루프 (DWT 역산 및 사후 차감 전면 소각)
    loop {
        IS_SLEEPING.store(false, core::sync::atomic::Ordering::Relaxed);
        unsafe {
            executor.poll();
        }
        IS_SLEEPING.store(true, core::sync::atomic::Ordering::Release);
        cortex_m::asm::wfe();
        IS_SLEEPING.store(false, core::sync::atomic::Ordering::Release);
    }
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner, p: embassy_stm32::Peripherals) {
    let leds = BoardLeds::new(p.PB0, p.PE1, p.PB14);
    spawner.must_spawn(task_led_heartbeat(leds.green));

    // I2C1 초기화 (PB8=SCL, PB9=SDA, 400kHz Fast Mode)
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

    // 센서 하드웨어 초기화
    init_sensors().await;

    // InEKF 필터 초기화 (데이터시트 기반 노이즈 공분산 파라미터 적용)
    {
        let mut filter = INEKF_FILTER.lock().await;
        *filter = RightInvariantInEKF::with_config(InEKFConfig::for_lsm6dso_and_lis2mdl(
            STANDARD_GRAVITY,
        ));
    }

    // NVIC CEC IRQ 바인딩 및 고우선순위(P6) 설정
    interrupt::CEC.set_priority(Priority::P6);
    let high_spawner = EXECUTOR_HIGH.start(interrupt::CEC);

    // [Task 1] 100 Hz 하드 실시간 RT-IMU 선점 루프 스폰
    high_spawner.must_spawn(task_imu_high_priority_rt());

    // [Task 2] 10 Hz 지자기 센서 비동기 관측 루프 스폰
    spawner.must_spawn(task_mag_sampling());

    // [Task 3] LAN8742A RMII 이더넷 드라이버 초기화 (STM32 고유 UID 기반 EUI-48 MAC 주소)
    let mac_addr = uid::get_unique_mac_address();
    let queue = PACKET_QUEUE.init(PacketQueue::new());
    let rmii_pins = BoardRmiiPins::new(
        p.PA1,  // RMII_REF_CLK
        p.PA2,  // RMII_MDIO
        p.PC1,  // RMII_MDC
        p.PA7,  // RMII_CRS_DV
        p.PC4,  // RMII_RXD0
        p.PC5,  // RMII_RXD1
        p.PG13, // RMII_TXD0
        p.PB13, // RMII_TXD1
        p.PG11, // RMII_TX_EN
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

    let net_seed = uid::get_uid_prng_seed();
    let net_config = embassy_net::Config::dhcpv4(Default::default());
    let resources = STACK_RESOURCES.init(embassy_net::StackResources::new());

    let (stack, runner) = embassy_net::new(eth_device, net_config, resources, net_seed);

    spawner.must_spawn(task_net_runner(runner));
    spawner.must_spawn(task_web_server(stack));
    spawner.must_spawn(task_rtt_reporter(stack));
}

/// [초기화] LSM6DSO (IMU) 및 LIS2MDL (지자기) 레지스터 설정 (BSP 상수 연동)
async fn init_sensors() {
    let mut bus_guard = I2C_BUS.lock().await;
    let i2c = bus_guard.as_mut().expect("I2C 버스 미초기화");

    // 1. LSM6DSO 초기화
    let mut whoami = [0u8; 1];
    if let Err(e) = i2c
        .write_read(ADDR_LSM6DSO, &[REG_WHO_AM_I], &mut whoami)
        .await
    {
        error!("LSM6DSO WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!(
            "LSM6DSO WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})",
            whoami[0], ID_LSM6DSO
        );
    }

    let _ = i2c
        .write(
            ADDR_LSM6DSO,
            &[lsm6dso::CTRL1_XL, lsm6dso::VAL_CTRL1_XL_104HZ_2G],
        )
        .await;
    let _ = i2c
        .write(
            ADDR_LSM6DSO,
            &[lsm6dso::CTRL2_G, lsm6dso::VAL_CTRL2_G_104HZ_250DPS],
        )
        .await;

    // 2. LIS2MDL 초기화
    let mut mag_who = [0u8; 1];
    if let Err(e) = i2c
        .write_read(ADDR_LIS2MDL, &[REG_LIS2MDL_WHO_AM_I], &mut mag_who)
        .await
    {
        error!("LIS2MDL WHO_AM_I 읽기 실패: {:?}", e);
    } else {
        info!(
            "LIS2MDL WHO_AM_I: 0x{:02X} (기대값: 0x{:02X})",
            mag_who[0], ID_LIS2MDL
        );
    }

    let _ = i2c
        .write(
            ADDR_LIS2MDL,
            &[lis2mdl::CFG_REG_A, lis2mdl::VAL_CFG_REG_A_RESET],
        )
        .await;
    Timer::after_millis(10).await;
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
}

/// [Task 1: 100 Hz 하드 실시간 RT-IMU 선점 루프]
#[embassy_executor::task]
async fn task_imu_high_priority_rt() {
    let mut ticker = Ticker::every(Duration::from_hz(100));
    let mut last_tick = Instant::now();
    let mut count: u32 = 0;

    loop {
        ticker.next().await;
        let now = Instant::now();
        let dt_us = (now - last_tick).as_micros() as u32;
        last_tick = now;
        count = count.wrapping_add(1);

        // 1. LSM6DSO 6축 데이터 읽기
        let mut buf = [0u8; 12];
        {
            let mut bus_guard = I2C_BUS.lock().await;
            if let Some(i2c) = bus_guard.as_mut() {
                let _ = i2c
                    .write_read(ADDR_LSM6DSO, &[lsm6dso::OUTX_L_G], &mut buf)
                    .await;
            }
        }

        // --- 순수 CPU 연산 구간 시작 (I2C 버스 대기 제외) ---
        IS_RT_ACTIVE.store(true, core::sync::atomic::Ordering::Relaxed);
        let t_calc_start = Instant::now();

        let gx_raw = i16::from_le_bytes([buf[0], buf[1]]);
        let gy_raw = i16::from_le_bytes([buf[2], buf[3]]);
        let gz_raw = i16::from_le_bytes([buf[4], buf[5]]);
        let ax_raw = i16::from_le_bytes([buf[6], buf[7]]);
        let ay_raw = i16::from_le_bytes([buf[8], buf[9]]);
        let az_raw = i16::from_le_bytes([buf[10], buf[11]]);

        // 물리 단위 변환 (BSP 정밀 감도 및 상수 모듈 적용)
        let gx_dps = lsm6dso::raw_to_dps_f32(gx_raw);
        let gy_dps = lsm6dso::raw_to_dps_f32(gy_raw);
        let gz_dps = lsm6dso::raw_to_dps_f32(gz_raw);
        let gyro_radps = [
            gx_dps * DEG_TO_RAD,
            gy_dps * DEG_TO_RAD,
            gz_dps * DEG_TO_RAD,
        ];

        let ax_mg = lsm6dso::raw_to_mg(ax_raw);
        let ay_mg = lsm6dso::raw_to_mg(ay_raw);
        let az_mg = lsm6dso::raw_to_mg(az_raw);
        // 정수 나눗셈/절삭 오차 없이 직접 단정도 부동소수점 가속도(m/s^2)로 산출
        let accel_mps2 = [
            lsm6dso::raw_to_mps2_f32(ax_raw),
            lsm6dso::raw_to_mps2_f32(ay_raw),
            lsm6dso::raw_to_mps2_f32(az_raw),
        ];

        // 2. SO(3) Right-Invariant InEKF 연산 (실측 가변 dt_s 반영 및 안전 클램핑)
        let dt_s = (dt_us as f32 * 1e-6).clamp(MIN_INTEGRATION_DT_S, MAX_INTEGRATION_DT_S);
        let mut filter = INEKF_FILTER.lock().await;
        // 정지 상태(Stillness / ZARU) 감지 및 바이어스 적응 갱신
        filter.update_stillness(gyro_radps, accel_mps2);
        filter.predict(gyro_radps, dt_s);
        filter.update_accel(accel_mps2);

        // 3. 자세 및 쿼터니언 유도
        let (roll, pitch, yaw) = filter.rot.to_euler_deg();
        let quat = filter.rot.to_quaternion();
        let rot_m = filter.rot.data;
        let bias_dps = [
            filter.bias_gyro[0] * RAD_TO_DEG,
            filter.bias_gyro[1] * RAD_TO_DEG,
            filter.bias_gyro[2] * RAD_TO_DEG,
        ];
        let trace = filter.cov_trace();
        let is_stationary = filter.is_stationary;

        let calc_us = (Instant::now() - t_calc_start).as_micros() as u32;

        // 4. 스냅샷 원자적 갱신
        let mut snap = AHRS_SNAPSHOT.lock().await;
        snap.roll_deg = roll;
        snap.pitch_deg = pitch;
        snap.yaw_deg = yaw;
        snap.quat = quat;
        snap.rot_matrix = rot_m;
        snap.bias_dps = bias_dps;
        snap.cov_trace = trace;
        snap.is_stationary = is_stationary;
        snap.imu_accel_mg = [ax_mg, ay_mg, az_mg];
        snap.imu_gyro_dps = [gx_dps as i16, gy_dps as i16, gz_dps as i16];
        snap.sample_count = count;
        snap.imu_dt_us = dt_us;
        snap.inekf_calc_us = calc_us;

        IS_RT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
    }
}

/// [Task 2: 10 Hz 지자기 센서 비동기 관측 루프]
#[embassy_executor::task]
async fn task_mag_sampling() {
    let mut ticker = Ticker::every(Duration::from_hz(10));
    loop {
        ticker.next().await;
        let mut buf = [0u8; 6];
        {
            let mut bus_guard = I2C_BUS.lock().await;
            if let Some(i2c) = bus_guard.as_mut() {
                let _ = i2c
                    .write_read(ADDR_LIS2MDL, &[lis2mdl::OUTX_L_REG], &mut buf)
                    .await;
            }
        }

        let mx_raw = i16::from_le_bytes([buf[0], buf[1]]);
        let my_raw = i16::from_le_bytes([buf[2], buf[3]]);
        let mz_raw = i16::from_le_bytes([buf[4], buf[5]]);

        // LIS2MDL 정밀 감도 모듈 적용 (1.5 mgauss/LSB)
        let mx_mgauss = lis2mdl::raw_to_mgauss(mx_raw);
        let my_mgauss = lis2mdl::raw_to_mgauss(my_raw);
        let mz_mgauss = lis2mdl::raw_to_mgauss(mz_raw);

        let mx_f = lis2mdl::raw_to_mgauss_f32(mx_raw);
        let my_f = lis2mdl::raw_to_mgauss_f32(my_raw);
        let mz_f = lis2mdl::raw_to_mgauss_f32(mz_raw);
        let norm_sq = mx_f * mx_f + my_f * my_f + mz_f * mz_f;

        if norm_sq > MIN_VALID_MAG_NORM_SQ {
            let inv_norm = 1.0 / libm::sqrtf(norm_sq);
            let mag_norm = [mx_f * inv_norm, my_f * inv_norm, mz_f * inv_norm];

            let mut filter = INEKF_FILTER.lock().await;
            filter.update_mag(mag_norm);
        }

        let mut snap = AHRS_SNAPSHOT.lock().await;
        snap.mag_mgauss = [mx_mgauss, my_mgauss, mz_mgauss];
    }
}

/// [Task 3: 이더넷 스택 러너]
#[embassy_executor::task]
async fn task_net_runner(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}

/// [Task 4: 내장 HTTP 웹서버 (포트 80)]
#[embassy_executor::task]
async fn task_web_server(stack: embassy_net::Stack<'static>) {
    info!("DHCP IP 주소 할당 대기 중...");
    stack.wait_config_up().await;
    if let Some(cfg) = stack.config_v4() {
        info!(
            ">>> 이더넷 웹서버 준비 완료: http://{}/ <<<",
            cfg.address.address()
        );
    }

    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];

    loop {
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(5)));

        if let Err(e) = socket.accept(80).await {
            warn!("TCP 소켓 수락 실패: {:?}", e);
            continue;
        }

        let mut req_buf = [0u8; 512];
        let n = match socket.read(&mut req_buf).await {
            Ok(0) => {
                socket.abort();
                continue;
            }
            Ok(n) => n,
            Err(_) => {
                socket.abort();
                continue;
            }
        };

        let req_str = match core::str::from_utf8(&req_buf[..n]) {
            Ok(s) => s,
            Err(_) => {
                socket.abort();
                continue;
            }
        };

        let is_api = req_str.starts_with("GET /api/ahrs");

        let snap = {
            let mut s = AHRS_SNAPSHOT.lock().await;
            s.http_request_count = s.http_request_count.wrapping_add(1);
            *s
        };

        if is_api {
            // GET /api/ahrs: 실시간 JSON 텔레메트리 (CPU 부하 및 InEKF 연산 시간 포함)
            let mut json = String::<896>::new();
            let _ = write!(
                json,
                "{{\"euler\":{{\"roll\":{}.{:02},\"pitch\":{}.{:02},\"yaw\":{}.{:02}}},\"quat\":{{\"w\":{}.{:03},\"x\":{}.{:03},\"y\":{}.{:03},\"z\":{}.{:03}}},\"bias_dps\":{{\"bx\":{}.{:02},\"by\":{}.{:02},\"bz\":{}.{:02}}},\"mat\":[[{}.{:02},{}.{:02},{}.{:02}],[{}.{:02},{}.{:02},{}.{:02}],[{}.{:02},{}.{:02},{}.{:02}]],\"imu\":{{\"ax\":{},\"ay\":{},\"az\":{},\"gx\":{},\"gy\":{},\"gz\":{}}},\"mag\":{{\"mx\":{},\"my\":{},\"mz\":{}}},\"stats\":{{\"count\":{},\"dt_us\":{},\"calc_us\":{},\"cpu_load\":{}.{:02},\"trace\":{}.{:04},\"reqs\":{},\"stationary\":{}}}}}",
                snap.roll_deg as i32, (snap.roll_deg.abs() * 100.0) as u32 % 100,
                snap.pitch_deg as i32, (snap.pitch_deg.abs() * 100.0) as u32 % 100,
                snap.yaw_deg as i32, (snap.yaw_deg.abs() * 100.0) as u32 % 100,
                snap.quat[0] as i32, (snap.quat[0].abs() * 1000.0) as u32 % 1000,
                snap.quat[1] as i32, (snap.quat[1].abs() * 1000.0) as u32 % 1000,
                snap.quat[2] as i32, (snap.quat[2].abs() * 1000.0) as u32 % 1000,
                snap.quat[3] as i32, (snap.quat[3].abs() * 1000.0) as u32 % 1000,
                snap.bias_dps[0] as i32, (snap.bias_dps[0].abs() * 100.0) as u32 % 100,
                snap.bias_dps[1] as i32, (snap.bias_dps[1].abs() * 100.0) as u32 % 100,
                snap.bias_dps[2] as i32, (snap.bias_dps[2].abs() * 100.0) as u32 % 100,
                snap.rot_matrix[0][0] as i32, (snap.rot_matrix[0][0].abs() * 100.0) as u32 % 100,
                snap.rot_matrix[0][1] as i32, (snap.rot_matrix[0][1].abs() * 100.0) as u32 % 100,
                snap.rot_matrix[0][2] as i32, (snap.rot_matrix[0][2].abs() * 100.0) as u32 % 100,
                snap.rot_matrix[1][0] as i32, (snap.rot_matrix[1][0].abs() * 100.0) as u32 % 100,
                snap.rot_matrix[1][1] as i32, (snap.rot_matrix[1][1].abs() * 100.0) as u32 % 100,
                snap.rot_matrix[1][2] as i32, (snap.rot_matrix[1][2].abs() * 100.0) as u32 % 100,
                snap.rot_matrix[2][0] as i32, (snap.rot_matrix[2][0].abs() * 100.0) as u32 % 100,
                snap.rot_matrix[2][1] as i32, (snap.rot_matrix[2][1].abs() * 100.0) as u32 % 100,
                snap.rot_matrix[2][2] as i32, (snap.rot_matrix[2][2].abs() * 100.0) as u32 % 100,
                snap.imu_accel_mg[0], snap.imu_accel_mg[1], snap.imu_accel_mg[2],
                snap.imu_gyro_dps[0], snap.imu_gyro_dps[1], snap.imu_gyro_dps[2],
                snap.mag_mgauss[0], snap.mag_mgauss[1], snap.mag_mgauss[2],
                snap.sample_count, snap.imu_dt_us, snap.inekf_calc_us,
                snap.cpu_load_pct as i32, (snap.cpu_load_pct.abs() * 100.0) as u32 % 100,
                snap.cov_trace as i32, (snap.cov_trace.abs() * 10000.0) as u32 % 10000,
                snap.http_request_count,
                if snap.is_stationary { "true" } else { "false" },
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
            // GET /: 3D 자세 시각화 다크 글래스모피즘 웹 대시보드
            let body = DASHBOARD_3D_HTML;
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

/// [Task 5: 1 Hz RTT 상태 리포터]
#[embassy_executor::task]
async fn task_rtt_reporter(stack: embassy_net::Stack<'static>) {
    let mut ticker = Ticker::every(Duration::from_hz(1));
    loop {
        ticker.next().await;

        let snap = {
            let s = AHRS_SNAPSHOT.lock().await;
            *s
        };

        // 리눅스 /proc/stat 1 kHz 하드웨어 틱 통계 샘플링 회계 (사후 차감 보정식 전면 소각)
        let idle_ticks = TICK_IDLE_COUNT.swap(0, core::sync::atomic::Ordering::Relaxed);
        let busy_ticks = TICK_BUSY_COUNT.swap(0, core::sync::atomic::Ordering::Relaxed);
        let total_ticks = idle_ticks + busy_ticks;

        let (cpu_load_pct, idle_pct) = if total_ticks > 0 {
            let load = (busy_ticks as f32 / total_ticks as f32) * 100.0;
            (load.clamp(0.0, 100.0), (100.0 - load).clamp(0.0, 100.0))
        } else {
            (0.0, 100.0)
        };

        {
            let mut s = AHRS_SNAPSHOT.lock().await;
            s.cpu_load_pct = cpu_load_pct;
        }

        let ip_str = if let Some(cfg) = stack.config_v4() {
            cfg.address.address()
        } else {
            embassy_net::Ipv4Address::new(0, 0, 0, 0)
        };

        info!(
            "[AHRS 1Hz] IP: {} | CPU: {=f32}% (Idle: {=f32}%, InEKF: {}µs) | Roll: {=f32}° | Pitch: {=f32}° | Yaw: {=f32}° | Bias: [{=f32}, {=f32}, {=f32}] | Trace: {=f32} | IMU cnt: {}",
            ip_str, cpu_load_pct, idle_pct, snap.inekf_calc_us, snap.roll_deg, snap.pitch_deg, snap.yaw_deg,
            snap.bias_dps[0], snap.bias_dps[1], snap.bias_dps[2],
            snap.cov_trace, snap.sample_count
        );
    }
}

/// [Task 6: 온보드 LED 하트비트]
#[embassy_executor::task]
async fn task_led_heartbeat(mut led: embassy_stm32::gpio::Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        Timer::after_millis(500).await;
    }
}

/// 플래시 내장 완전 독립형(Zero-CDN) 3D 웹 대시보드 HTML/CSS/JS 자산
const DASHBOARD_3D_HTML: &str = r#"<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>NUCLEO-H743ZI2 SO(3) InEKF 3D AHRS</title>
<style>
:root {
  --bg: #090d16;
  --panel: rgba(18, 24, 38, 0.7);
  --border: rgba(255, 255, 255, 0.08);
  --accent: #38bdf8;
  --accent-glow: rgba(56, 189, 248, 0.35);
  --text: #f1f5f9;
  --subtext: #94a3b8;
  --green: #4ade80;
  --red: #f87171;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  background: radial-gradient(circle at top right, #1e293b, var(--bg));
  color: var(--text);
  min-height: 100vh;
  padding: 24px;
}
header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 24px;
  padding-bottom: 16px;
  border-bottom: 1px solid var(--border);
}
h1 { font-size: 1.4rem; font-weight: 700; letter-spacing: -0.5px; }
h1 span { color: var(--accent); }
.badge {
  font-size: 0.75rem;
  background: rgba(56, 189, 248, 0.15);
  color: var(--accent);
  padding: 4px 10px;
  border-radius: 9999px;
  border: 1px solid var(--accent);
}
.grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 20px;
  margin-bottom: 20px;
}
@media (max-width: 900px) {
  .grid { grid-template-columns: 1fr; }
}
.card {
  background: var(--panel);
  backdrop-filter: blur(12px);
  border: 1px solid var(--border);
  border-radius: 16px;
  padding: 20px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);
}
.card-title {
  font-size: 0.85rem;
  color: var(--subtext);
  text-transform: uppercase;
  letter-spacing: 0.5px;
  margin-bottom: 16px;
  display: flex;
  justify-content: space-between;
}
/* 3D Visualizer Scene */
.scene3d {
  width: 100%;
  height: 300px;
  perspective: 800px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: radial-gradient(circle at center, rgba(30, 41, 59, 0.4) 0%, rgba(9, 13, 22, 0.95) 75%);
  border-radius: 12px;
  overflow: hidden;
  position: relative;
}
.camera-rig {
  position: relative;
  width: 100%;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  transform-style: preserve-3d;
  transform: rotateX(25deg) rotateY(-30deg);
}
.world-floor {
  position: absolute;
  width: 260px;
  height: 260px;
  border-radius: 50%;
  border: 1px dashed rgba(56, 189, 248, 0.2);
  background: radial-gradient(circle, rgba(56, 189, 248, 0.05) 0%, transparent 70%);
  transform: rotateX(90deg) translateZ(-65px);
  pointer-events: none;
}
.world-floor::after {
  content: '';
  position: absolute;
  top: 50%; left: 0; right: 0;
  height: 1px;
  background: rgba(255, 255, 255, 0.08);
}
.world-floor::before {
  content: '';
  position: absolute;
  left: 50%; top: 0; bottom: 0;
  width: 1px;
  background: rgba(255, 255, 255, 0.08);
}
.prism {
  width: 150px;
  height: 14px;
  position: relative;
  transform-style: preserve-3d;
  transition: transform 0.08s cubic-bezier(0.2, 0.8, 0.2, 1);
}
.p-face {
  position: absolute;
  box-sizing: border-box;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.65rem;
  font-weight: 700;
  border-radius: 2px;
}
.p-face.top {
  width: 150px;
  height: 100px;
  background: linear-gradient(135deg, rgba(30, 41, 59, 0.95) 0%, rgba(15, 23, 42, 0.98) 100%);
  border: 1.5px solid rgba(56, 189, 248, 0.7);
  box-shadow: inset 0 0 15px rgba(56, 189, 248, 0.15), 0 0 20px rgba(0, 0, 0, 0.6);
  transform: rotateX(90deg) translateZ(7px) translateY(-50px);
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  padding: 8px 12px;
  color: #e2e8f0;
}
.p-face.bottom {
  width: 150px;
  height: 100px;
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(56, 189, 248, 0.3);
  transform: rotateX(-90deg) translateZ(7px) translateY(50px);
}
.p-face.front {
  width: 150px;
  height: 14px;
  background: rgba(239, 68, 68, 0.25);
  border: 1px solid #ef4444;
  color: #fca5a5;
  font-size: 0.6rem;
  transform: translateZ(50px);
}
.p-face.back {
  width: 150px;
  height: 14px;
  background: rgba(30, 41, 59, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.3);
  transform: rotateY(180deg) translateZ(50px);
}
.p-face.right {
  width: 100px;
  height: 14px;
  background: rgba(34, 197, 94, 0.25);
  border: 1px solid #22c55e;
  color: #86efac;
  font-size: 0.6rem;
  transform: rotateY(90deg) translateZ(75px) translateX(-50px);
}
.p-face.left {
  width: 100px;
  height: 14px;
  background: rgba(30, 41, 59, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.3);
  transform: rotateY(-90deg) translateZ(75px) translateX(-50px);
}
/* Body Frame RGB 3축 (Tripod) */
.axis-beam {
  position: absolute;
  transform-style: preserve-3d;
  pointer-events: none;
}
.axis-x {
  width: 85px;
  height: 3px;
  background: #ef4444;
  box-shadow: 0 0 8px #ef4444;
  top: 50%; left: 50%;
  transform-origin: left center;
  transform: rotateY(90deg) translateZ(0);
}
.axis-x::after {
  content: '+X (Roll)';
  position: absolute;
  right: -58px; top: -9px;
  font-size: 0.65rem;
  font-weight: 700;
  color: #fca5a5;
  background: rgba(15, 23, 42, 0.85);
  border: 1px solid #ef4444;
  padding: 1px 4px;
  border-radius: 3px;
  white-space: nowrap;
}
.axis-y {
  width: 85px;
  height: 3px;
  background: #22c55e;
  box-shadow: 0 0 8px #22c55e;
  top: 50%; left: 50%;
  transform-origin: left center;
  transform: rotateY(0deg) translateZ(0);
}
.axis-y::after {
  content: '+Y (Pitch)';
  position: absolute;
  right: -60px; top: -9px;
  font-size: 0.65rem;
  font-weight: 700;
  color: #86efac;
  background: rgba(15, 23, 42, 0.85);
  border: 1px solid #22c55e;
  padding: 1px 4px;
  border-radius: 3px;
  white-space: nowrap;
}
.axis-z {
  width: 75px;
  height: 3px;
  background: #38bdf8;
  box-shadow: 0 0 8px #38bdf8;
  top: 50%; left: 50%;
  transform-origin: left center;
  transform: rotateZ(-90deg) translateZ(0);
}
.axis-z::after {
  content: '+Z (Yaw)';
  position: absolute;
  right: -55px; top: -9px;
  font-size: 0.65rem;
  font-weight: 700;
  color: #7dd3fc;
  background: rgba(15, 23, 42, 0.85);
  border: 1px solid #38bdf8;
  padding: 1px 4px;
  border-radius: 3px;
  white-space: nowrap;
}
.origin-hub {
  position: absolute;
  width: 10px;
  height: 10px;
  background: #fff;
  border-radius: 50%;
  top: 50%; left: 50%;
  transform: translate(-50%, -50%);
  box-shadow: 0 0 10px #fff;
  z-index: 10;
}
.chip {
  background: #0f172a;
  border: 1px solid #94a3b8;
  color: #38bdf8;
  padding: 2px 5px;
  border-radius: 3px;
  font-size: 0.6rem;
}
.axes {
  display: flex;
  gap: 8px;
  font-size: 0.65rem;
}
.axis-x-tag { color: #f87171; font-weight: 700; }
.axis-y-tag { color: #4ade80; font-weight: 700; }
.axis-z-tag { color: #38bdf8; font-weight: 700; }

/* Attitude Angles */
.stat-row {
  display: flex;
  justify-content: space-around;
  text-align: center;
  margin-top: 10px;
}
.stat-item .val {
  font-size: 1.8rem;
  font-weight: 700;
  color: var(--accent);
}
.stat-item .unit {
  font-size: 0.8rem;
  color: var(--subtext);
}
/* Matrix & Details */
.matrix-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 6px;
  font-family: monospace;
  font-size: 0.8rem;
  background: rgba(0,0,0,0.3);
  padding: 10px;
  border-radius: 8px;
  text-align: center;
}
.detail-table {
  width: 100%;
  font-size: 0.85rem;
  border-collapse: collapse;
  margin-top: 10px;
}
.detail-table td {
  padding: 6px 0;
  border-bottom: 1px solid rgba(255,255,255,0.05);
}
.detail-table td:last-child {
  text-align: right;
  font-family: monospace;
  color: var(--accent);
}
</style>
</head>
<body>
<header>
  <div>
    <h1>NUCLEO-H743ZI2 <span>SO(3) InEKF AHRS</span></h1>
    <p style="font-size:0.8rem;color:var(--subtext)">Right-Invariant Lie Group Manifold Filter (100Hz RT-IMU)</p>
  </div>
  <div style="display:flex;gap:10px;align-items:center">
    <div class="badge" id="cpu-badge" style="border-color:#38bdf8;color:#38bdf8;background:rgba(56,189,248,0.12)">CPU: 0.0% (InEKF 0 µs)</div>
    <div class="badge" id="status">연결 중...</div>
  </div>
</header>

<div class="grid">
  <!-- 3D Attitude Visualizer -->
  <div class="card">
    <div class="card-title">
      <span>3D Body Frame 자세 동기화 (GPU 가속)</span>
      <span class="axes"><span class="axis-x-tag">+X (Roll)</span> <span class="axis-y-tag">+Y (Pitch)</span> <span class="axis-z-tag">+Z (Yaw)</span></span>
    </div>
    <div class="scene3d">
      <div class="camera-rig">
        <div class="world-floor"></div>
        <div class="prism" id="board">
          <div class="p-face front">FRONT (+X)</div>
          <div class="p-face back">BACK</div>
          <div class="p-face top">
            <div style="font-size:0.6rem;color:#cbd5e1;display:flex;justify-content:space-between">
              <span>NUCLEO-H743ZI2</span>
              <span>480MHz</span>
            </div>
            <div style="display:flex;justify-content:center;gap:6px;margin:2px 0">
              <span class="chip">LSM6DSO</span>
              <span class="chip">LIS2MDL</span>
            </div>
            <div style="font-size:0.55rem;color:#94a3b8;text-align:center">BODY FRAME (X-FWD, Y-RIGHT, Z-UP)</div>
          </div>
          <div class="p-face bottom"></div>
          <div class="p-face left">LEFT (-Y)</div>
          <div class="p-face right">RIGHT (+Y)</div>

          <!-- Body Frame RGB 3축 (Tripod) -->
          <div class="origin-hub"></div>
          <div class="axis-beam axis-x"></div>
          <div class="axis-beam axis-y"></div>
          <div class="axis-beam axis-z"></div>
        </div>
      </div>
    </div>
    <div class="stat-row">
      <div class="stat-item"><div class="val" id="roll">0.0</div><div class="unit">Roll (X) °</div></div>
      <div class="stat-item"><div class="val" id="pitch">0.0</div><div class="unit">Pitch (Y) °</div></div>
      <div class="stat-item"><div class="val" id="yaw">0.0</div><div class="unit">Yaw (Z) °</div></div>
    </div>
  </div>

  <!-- Lie Group Rotation & Covariance -->
  <div class="card">
    <div class="card-title">
      <span>SO(3) 회전 행렬 R 및 칼만 공분산</span>
      <span id="cov-trace" style="color:var(--green)">Trace: 0.0000</span>
    </div>
    <div class="matrix-grid" id="rot-mat">
      <div>1.00</div><div>0.00</div><div>0.00</div>
      <div>0.00</div><div>1.00</div><div>0.00</div>
      <div>0.00</div><div>0.00</div><div>1.00</div>
    </div>
    <table class="detail-table">
      <tr><td>동작 모드 (Stillness / ZARU)</td><td id="motion-mode" style="font-weight:700;color:var(--accent)">초기화 중</td></tr>
      <tr><td>MCU CPU 부하 (Cortex-M7 480MHz)</td><td id="cpu-stat" style="font-weight:700;color:#38bdf8">0.0 %</td></tr>
      <tr><td>InEKF 1회 순수 연산 (No I/O)</td><td id="inekf-stat" style="font-family:monospace;color:#38bdf8">0 µs / 10,000 µs</td></tr>
      <tr><td>단위 쿼터니언 (w, x, y, z)</td><td id="quat">1.00, 0.00, 0.00, 0.00</td></tr>
      <tr><td>추정 자이로 바이어스 bx, by, bz (dps)</td><td id="bias">0.00, 0.00, 0.00</td></tr>
      <tr><td>가속도계 (X, Y, Z mg)</td><td id="acc">0, 0, 1000</td></tr>
      <tr><td>지자기 센서 (X, Y, Z mgauss)</td><td id="mag">0, 0, 0</td></tr>
      <tr><td>100Hz 루프 샘플수 / 주기(dt)</td><td id="loop-stats">0 cnt / 10000 µs</td></tr>
    </table>
  </div>
</div>

<script>
const board = document.getElementById('board');
const rollEl = document.getElementById('roll');
const pitchEl = document.getElementById('pitch');
const yawEl = document.getElementById('yaw');
const quatEl = document.getElementById('quat');
const biasEl = document.getElementById('bias');
const accEl = document.getElementById('acc');
const magEl = document.getElementById('mag');
const loopEl = document.getElementById('loop-stats');
const traceEl = document.getElementById('cov-trace');
const matEl = document.getElementById('rot-mat');
const statusEl = document.getElementById('status');
const modeEl = document.getElementById('motion-mode');
const cpuBadge = document.getElementById('cpu-badge');
const cpuStatEl = document.getElementById('cpu-stat');
const inekfStatEl = document.getElementById('inekf-stat');

let curR = 0, curP = 0, curY = 0;
let hasInitAngles = false;

// 오일러 각 +-180도 경계 래핑 시 CSS 360도 반대 회전 플립(휙 도는 현상) 완전 방지
function unwrapAngle(target, current) {
  let diff = (target - current) % 360;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  return current + diff;
}

async function updateTele() {
  try {
    const res = await fetch('/api/ahrs');
    if (!res.ok) return;
    const d = await res.json();
    if (d.stats.stationary) {
      statusEl.textContent = '정지 상태 (ZARU 영점 보정)';
      statusEl.style.color = '#38bdf8';
      modeEl.textContent = '정지 (ZARU 적분 동결 + 바이어스 흡수)';
      modeEl.style.color = '#38bdf8';
    } else {
      statusEl.textContent = '100Hz RT 동적 기동 중';
      statusEl.style.color = '#4ade80';
      modeEl.textContent = '동적 기동 (적응형 공분산 보호)';
      modeEl.style.color = '#4ade80';
    }

    const cpu = d.stats.cpu_load !== undefined ? d.stats.cpu_load : 0.0;
    const calc = d.stats.calc_us !== undefined ? d.stats.calc_us : 0;
    if (cpuBadge) cpuBadge.textContent = `CPU: ${cpu.toFixed(1)}% (InEKF ${calc} µs)`;
    if (cpuStatEl) cpuStatEl.textContent = `${cpu.toFixed(1)} %`;
    if (inekfStatEl) inekfStatEl.textContent = `${calc} µs / 10,000 µs (${((calc / 10000) * 100).toFixed(2)}%)`;

    const r = d.euler.roll;
    const p = d.euler.pitch;
    const y = d.euler.yaw;

    rollEl.textContent = r.toFixed(1);
    pitchEl.textContent = p.toFixed(1);
    yawEl.textContent = y.toFixed(1);

    if (!hasInitAngles) {
      curR = r; curP = p; curY = y;
      hasInitAngles = true;
    } else {
      curR = unwrapAngle(r, curR);
      curP = unwrapAngle(p, curP);
      curY = unwrapAngle(y, curY);
    }

    // 3D 보드 모델 회전 동기화 (연속 언래핑 각도 적용으로 특이점 플립 제로화)
    board.style.transform = `rotateX(${-curP}deg) rotateZ(${-curR}deg) rotateY(${curY}deg)`;

    quatEl.textContent = `${d.quat.w.toFixed(3)}, ${d.quat.x.toFixed(3)}, ${d.quat.y.toFixed(3)}, ${d.quat.z.toFixed(3)}`;
    biasEl.textContent = `${d.bias_dps.bx.toFixed(2)}, ${d.bias_dps.by.toFixed(2)}, ${d.bias_dps.bz.toFixed(2)}`;
    accEl.textContent = `${d.imu.ax}, ${d.imu.ay}, ${d.imu.az}`;
    magEl.textContent = `${d.mag.mx}, ${d.mag.my}, ${d.mag.mz}`;
    loopEl.textContent = `${d.stats.count} cnt / ${d.stats.dt_us} µs`;
    traceEl.textContent = `P Trace: ${d.stats.trace.toFixed(4)}`;

    const m = d.mat;
    matEl.innerHTML = `
      <div>${m[0][0].toFixed(2)}</div><div>${m[0][1].toFixed(2)}</div><div>${m[0][2].toFixed(2)}</div>
      <div>${m[1][0].toFixed(2)}</div><div>${m[1][1].toFixed(2)}</div><div>${m[1][2].toFixed(2)}</div>
      <div>${m[2][0].toFixed(2)}</div><div>${m[2][1].toFixed(2)}</div><div>${m[2][2].toFixed(2)}</div>
    `;
  } catch (e) {
    statusEl.textContent = '오프라인';
    statusEl.style.color = '#f87171';
  }
}
setInterval(updateTele, 80); // 12.5 Hz 브라우저 폴링
</script>
</body>
</html>
"#;
