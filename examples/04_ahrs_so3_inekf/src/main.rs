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
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::eth::generic_smi::GenericSMI;
use embassy_stm32::eth::{self, Ethernet, PacketQueue};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::peripherals::{ETH, I2C1};
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::Write as _;
use heapless::String;
use nucleo_bsp::BoardLeds;
use so3_inekf::RightInvariantInEKF;

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
    http_request_count: 0,
});

// 5. 이더넷 패킷 큐 및 네트워크 스택 리소스 (정적 할당)
static mut PACKET_QUEUE: PacketQueue<4, 4> = PacketQueue::new();
static mut STACK_RESOURCES: embassy_net::StackResources<4> = embassy_net::StackResources::new();

type Device = Ethernet<'static, ETH, GenericSMI>;

// I2C 디바이스 주소
const ADDR_LSM6DSO: u8 = 0x6B;
const ADDR_LIS2MDL: u8 = 0x1E;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!(">>> NUCLEO-H743ZI2 SO(3) Right-Invariant InEKF AHRS 시작 <<<");

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
        Hertz(400_000),
        Default::default(),
    );

    {
        let mut bus = I2C_BUS.lock().await;
        *bus = Some(i2c);
    }

    // 센서 하드웨어 초기화
    init_sensors().await;

    // InEKF 필터 초기화
    {
        let mut filter = INEKF_FILTER.lock().await;
        *filter = RightInvariantInEKF::new();
    }

    // NVIC CEC IRQ 바인딩 및 고우선순위(P6) 설정
    interrupt::CEC.set_priority(Priority::P6);
    let high_spawner = EXECUTOR_HIGH.start(interrupt::CEC);

    // [Task 1] 100 Hz 하드 실시간 RT-IMU 선점 루프 스폰
    high_spawner.must_spawn(task_imu_high_priority_rt());

    // [Task 2] 10 Hz 지자기 센서 비동기 관측 루프 스폰
    spawner.must_spawn(task_mag_sampling());

    // [Task 3] LAN8742A RMII 이더넷 드라이버 초기화 및 DHCPv4 네트워크 스택
    let mac_addr = [0x00, 0x80, 0xE1, 0xDE, 0xAD, 0x04];
    let queue = unsafe { &mut *core::ptr::addr_of_mut!(PACKET_QUEUE) };

    let eth_device = Ethernet::new(
        queue,
        p.ETH,
        Irqs,
        p.PA1,  // RMII_REF_CLK
        p.PA2,  // RMII_MDIO
        p.PC1,  // RMII_MDC
        p.PA7,  // RMII_CRS_DV
        p.PC4,  // RMII_RXD0
        p.PC5,  // RMII_RXD1
        p.PG13, // RMII_TXD0
        p.PB13, // RMII_TXD1
        p.PG11, // RMII_TX_EN
        GenericSMI::new(0),
        mac_addr,
    );

    let net_seed = 0x1234_5678_9ABC_DEF4;
    let net_config = embassy_net::Config::dhcpv4(Default::default());
    let resources = unsafe { &mut *core::ptr::addr_of_mut!(STACK_RESOURCES) };

    let (stack, runner) = embassy_net::new(eth_device, net_config, resources, net_seed);

    spawner.must_spawn(task_net_runner(runner));
    spawner.must_spawn(task_web_server(stack));
    spawner.must_spawn(task_rtt_reporter(stack));
}

/// [초기화] LSM6DSO (IMU) 및 LIS2MDL (지자기) 레지스터 설정
async fn init_sensors() {
    let mut bus_guard = I2C_BUS.lock().await;
    let i2c = bus_guard.as_mut().unwrap();

    // 1. LSM6DSO 초기화
    let mut whoami = [0u8; 1];
    let _ = i2c.write_read(ADDR_LSM6DSO, &[0x0F], &mut whoami).await;
    info!("LSM6DSO WHO_AM_I: 0x{:02X} (기대값: 0x6C)", whoami[0]);

    let _ = i2c.write(ADDR_LSM6DSO, &[0x10, 0x40]).await; // Accel 104Hz, ±2g
    let _ = i2c.write(ADDR_LSM6DSO, &[0x11, 0x40]).await; // Gyro 104Hz, ±250dps

    // 2. LIS2MDL 초기화
    let mut mag_who = [0u8; 1];
    let _ = i2c.write_read(ADDR_LIS2MDL, &[0x4F], &mut mag_who).await;
    info!("LIS2MDL WHO_AM_I: 0x{:02X} (기대값: 0x40)", mag_who[0]);

    let _ = i2c.write(ADDR_LIS2MDL, &[0x60, 0x80]).await; // 리셋
    Timer::after_millis(10).await;
    let _ = i2c.write(ADDR_LIS2MDL, &[0x60, 0x00]).await; // 10Hz 연속 모드
    let _ = i2c.write(ADDR_LIS2MDL, &[0x62, 0x10]).await; // BDU = 1
}

/// [Task 1: 100 Hz 하드 실시간 RT-IMU 선점 루프]
#[embassy_executor::task]
async fn task_imu_high_priority_rt() {
    let mut ticker = Ticker::every(Duration::from_hz(100));
    let mut last_tick = Instant::now();
    let mut count: u32 = 0;
    const PI: f32 = core::f32::consts::PI;

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
                let _ = i2c.write_read(ADDR_LSM6DSO, &[0x22], &mut buf).await;
            }
        }

        let gx_raw = i16::from_le_bytes([buf[0], buf[1]]);
        let gy_raw = i16::from_le_bytes([buf[2], buf[3]]);
        let gz_raw = i16::from_le_bytes([buf[4], buf[5]]);
        let ax_raw = i16::from_le_bytes([buf[6], buf[7]]);
        let ay_raw = i16::from_le_bytes([buf[8], buf[9]]);
        let az_raw = i16::from_le_bytes([buf[10], buf[11]]);

        // 물리 단위 변환
        // Gyro ±250 dps: 8.75 mdps/LSB -> dps -> rad/s
        let gx_dps = gx_raw as f32 * 0.00875;
        let gy_dps = gy_raw as f32 * 0.00875;
        let gz_dps = gz_raw as f32 * 0.00875;
        let gyro_radps = [gx_dps * PI / 180.0, gy_dps * PI / 180.0, gz_dps * PI / 180.0];

        // Accel ±2g: 0.061 mg/LSB -> mg -> m/s^2
        let ax_mg = (ax_raw as f32 * 0.061) as i16;
        let ay_mg = (ay_raw as f32 * 0.061) as i16;
        let az_mg = (az_raw as f32 * 0.061) as i16;
        let accel_mps2 = [
            ax_mg as f32 * 0.001 * 9.80665,
            ay_mg as f32 * 0.001 * 9.80665,
            az_mg as f32 * 0.001 * 9.80665,
        ];

        // 2. SO(3) Right-Invariant InEKF 연산
        let dt_s = 0.01f32; // 100 Hz = 0.01초
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
            filter.bias_gyro[0] * 180.0 / PI,
            filter.bias_gyro[1] * 180.0 / PI,
            filter.bias_gyro[2] * 180.0 / PI,
        ];
        let trace = filter.cov_trace();
        let is_stationary = filter.is_stationary;

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
                let _ = i2c.write_read(ADDR_LIS2MDL, &[0x68], &mut buf).await;
            }
        }

        let mx_raw = i16::from_le_bytes([buf[0], buf[1]]);
        let my_raw = i16::from_le_bytes([buf[2], buf[3]]);
        let mz_raw = i16::from_le_bytes([buf[4], buf[5]]);

        // LIS2MDL: 1.5 mgauss/LSB
        let mx_mgauss = (mx_raw as f32 * 1.5) as i16;
        let my_mgauss = (my_raw as f32 * 1.5) as i16;
        let mz_mgauss = (mz_raw as f32 * 1.5) as i16;

        let mx_f = mx_mgauss as f32;
        let my_f = my_mgauss as f32;
        let mz_f = mz_mgauss as f32;
        let norm_sq = mx_f * mx_f + my_f * my_f + mz_f * mz_f;

        if norm_sq > 100.0 {
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
        info!(">>> 이더넷 웹서버 준비 완료: http://{}/ <<<", cfg.address.address());
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
            // GET /api/ahrs: 실시간 JSON 텔레메트리
            let mut json = String::<768>::new();
            let _ = write!(
                json,
                "{{\"euler\":{{\"roll\":{}.{:02},\"pitch\":{}.{:02},\"yaw\":{}.{:02}}},\"quat\":{{\"w\":{}.{:03},\"x\":{}.{:03},\"y\":{}.{:03},\"z\":{}.{:03}}},\"bias_dps\":{{\"bx\":{}.{:02},\"by\":{}.{:02},\"bz\":{}.{:02}}},\"mat\":[[{}.{:02},{}.{:02},{}.{:02}],[{}.{:02},{}.{:02},{}.{:02}],[{}.{:02},{}.{:02},{}.{:02}]],\"imu\":{{\"ax\":{},\"ay\":{},\"az\":{},\"gx\":{},\"gy\":{},\"gz\":{}}},\"mag\":{{\"mx\":{},\"my\":{},\"mz\":{}}},\"stats\":{{\"count\":{},\"dt_us\":{},\"trace\":{}.{:04},\"reqs\":{},\"stationary\":{}}}}}",
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
                snap.sample_count, snap.imu_dt_us,
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

        let ip_str = if let Some(cfg) = stack.config_v4() {
            cfg.address.address()
        } else {
            embassy_net::Ipv4Address::new(0, 0, 0, 0)
        };

        info!(
            "[AHRS 1Hz] IP: {} | Roll: {=f32}° | Pitch: {=f32}° | Yaw: {=f32}° | Bias: [{=f32}, {=f32}, {=f32}] | Trace: {=f32} | IMU cnt: {}",
            ip_str, snap.roll_deg, snap.pitch_deg, snap.yaw_deg,
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
  height: 280px;
  perspective: 700px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: radial-gradient(circle, rgba(56,189,248,0.05) 0%, transparent 70%);
  border-radius: 12px;
  overflow: hidden;
}
.cube {
  width: 140px;
  height: 20px;
  position: relative;
  transform-style: preserve-3d;
  transition: transform 0.05s linear;
}
.face {
  position: absolute;
  border: 1px solid rgba(56, 189, 248, 0.6);
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.7rem;
  font-weight: 700;
  color: #fff;
  border-radius: 4px;
}
.face.top {
  width: 140px; height: 180px;
  background: linear-gradient(135deg, #065f46 0%, #047857 100%);
  transform: rotateX(90deg) translateZ(10px) translateY(-90px);
  box-shadow: inset 0 0 15px rgba(0,0,0,0.5);
  display: flex;
  flex-direction: column;
  justify-content: space-around;
  padding: 10px;
}
.face.bottom {
  width: 140px; height: 180px;
  background: #064e3b;
  transform: rotateX(-90deg) translateZ(10px) translateY(90px);
}
.face.front {
  width: 140px; height: 20px;
  background: #022c22;
  transform: translateZ(90px);
}
.face.back {
  width: 140px; height: 20px;
  background: #022c22;
  transform: rotateY(180deg) translateZ(90px);
}
.face.right {
  width: 180px; height: 20px;
  background: #047857;
  transform: rotateY(90deg) translateZ(50px) translateX(-90px);
}
.face.left {
  width: 180px; height: 20px;
  background: #047857;
  transform: rotateY(-90deg) translateZ(90px) translateX(-90px);
}
.chip {
  background: #0f172a;
  border: 1px solid #94a3b8;
  color: #38bdf8;
  padding: 3px 6px;
  border-radius: 3px;
  font-size: 0.65rem;
}
.axes {
  display: flex;
  gap: 8px;
  font-size: 0.65rem;
}
.axis-x { color: #f87171; }
.axis-y { color: #4ade80; }
.axis-z { color: #60a5fa; }

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
  <div class="badge" id="status">연결 중...</div>
</header>

<div class="grid">
  <!-- 3D Attitude Visualizer -->
  <div class="card">
    <div class="card-title">
      <span>3D 보드 자세 동기화 (GPU 가속)</span>
      <span class="axes"><span class="axis-x">X(Roll)</span> <span class="axis-y">Y(Pitch)</span> <span class="axis-z">Z(Yaw)</span></span>
    </div>
    <div class="scene3d">
      <div class="cube" id="board">
        <div class="face front">NUCLEO-H743ZI2</div>
        <div class="face back">ST-LINK / V3E</div>
        <div class="face top">
          <div style="font-size:0.6rem;color:#cbd5e1">STM32H743ZI 480MHz</div>
          <div style="display:flex;justify-content:center;gap:6px">
            <span class="chip">LSM6DSO</span>
            <span class="chip">LIS2MDL</span>
          </div>
          <div style="font-size:0.55rem;color:#94a3b8">X-NUCLEO-IKS01A3</div>
        </div>
        <div class="face bottom"></div>
        <div class="face left">RJ45 ETH</div>
        <div class="face right">IKS01A3</div>
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

    const r = d.euler.roll;
    const p = d.euler.pitch;
    const y = d.euler.yaw;

    rollEl.textContent = r.toFixed(1);
    pitchEl.textContent = p.toFixed(1);
    yawEl.textContent = y.toFixed(1);

    // 3D 보드 모델 회전 동기화 (Roll: rotateX, Pitch: rotateZ, Yaw: rotateY)
    board.style.transform = `rotateX(${-p}deg) rotateZ(${-r}deg) rotateY(${y}deg)`;

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
