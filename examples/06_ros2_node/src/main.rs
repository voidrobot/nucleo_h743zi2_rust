//! # NUCLEO-H743ZI2 임베디드 ROS 2 노드 펌웨어 (06_ros2_node)
//!
//! $SO(3)$ InEKF 자세 추정 및 5대 센서 텔레메트리, cmd_vel 속도 제어 수신,
//! set_led 서비스를 Zenoh 1.0 프로토콜을 통해 ROS 2 Jazzy 네트워크로 직접 서빙한다.

#![no_std]
#![no_main]

mod cdr;
mod msg;
mod srv;
mod zenoh_wire;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Ipv4Address, Stack, StackResources};
use embassy_stm32::eth::generic_smi::GenericSMI;
use embassy_stm32::eth::{Ethernet, PacketQueue};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt::{self, InterruptExt, Priority};
use embassy_stm32::peripherals::ETH;
use embassy_stm32::{bind_interrupts, eth, i2c, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};
use embassy_futures::select::{select, Either};
use panic_probe as _;
use static_cell::StaticCell;

use msg::{endpoints as msg_endpoints, FluidPressure, Header, Imu, MagneticField, Quaternion, RelativeHumidity, Temperature, Twist, Vector3};
use srv::{endpoints as srv_endpoints, set_bool};
use nucleo_bsp::iks01a3::*;
use nucleo_bsp::{BoardRmiiPins, I2C_FAST_MODE_HZ};
use so3_inekf::inekf::RightInvariantInEKF;
use zenoh_wire::{msg_id, ZenohWire};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    ETH => eth::InterruptHandler;
});

// ----------------------------------------------------------------------------
// 1. 단일 진실 공급원 (SSOT) 상수
// ----------------------------------------------------------------------------
/// ROS 2 도메인 ID (호스트의 ROS_DOMAIN_ID 환경변수와 1:1 매칭)
pub const ROS_DOMAIN_ID: u32 = 0;
pub const ZENOH_UDP_PORT: u16 = 7447;

const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

// ----------------------------------------------------------------------------
// 2. 리눅스 /proc/stat 메커니즘 1 kHz 통계적 틱 프로파일러
// ----------------------------------------------------------------------------
static IS_SLEEPING: AtomicBool = AtomicBool::new(false);
static IS_RT_ACTIVE: AtomicBool = AtomicBool::new(false);
static TICK_IDLE_COUNT: AtomicU32 = AtomicU32::new(0);
static TICK_BUSY_COUNT: AtomicU32 = AtomicU32::new(0);

#[embassy_stm32::interrupt]
unsafe fn TIM7() {
    let tim = embassy_stm32::pac::TIM7;
    tim.sr().write(|w| w.set_uif(false));

    if IS_SLEEPING.load(Ordering::Relaxed) && !IS_RT_ACTIVE.load(Ordering::Relaxed) {
        TICK_IDLE_COUNT.fetch_add(1, Ordering::Relaxed);
    } else {
        TICK_BUSY_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

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

// ----------------------------------------------------------------------------
// 3. 하드웨어 리소스 및 전역 공유 뮤텍스
// ----------------------------------------------------------------------------
type I2cBus = Mutex<CriticalSectionRawMutex, Option<I2c<'static, embassy_stm32::mode::Async>>>;
static I2C_BUS: I2cBus = Mutex::new(None);

type InEkfMutex = Mutex<CriticalSectionRawMutex, RightInvariantInEKF>;
static INEKF_FILTER: InEkfMutex = Mutex::new(RightInvariantInEKF::new());

type LedMutex = Mutex<CriticalSectionRawMutex, Option<Output<'static>>>;
static USER_LED: LedMutex = Mutex::new(None);

/// 환경 센서 원자적 최신 데이터
#[derive(Copy, Clone)]
pub struct EnvData {
    pub mag_tesla: [f64; 3],
    pub pressure_pa: f64,
    pub temperature_c: f64,
    pub humidity_ratio: f64,
}
static ENV_DATA: Mutex<CriticalSectionRawMutex, EnvData> = Mutex::new(EnvData {
    mag_tesla: [0.0; 3],
    pressure_pa: 101325.0,
    temperature_c: 25.0,
    humidity_ratio: 0.5,
});

/// AHRS 100Hz 자세 스냅샷
#[derive(Copy, Clone, Default)]
pub struct ImuSnapshot {
    pub sec: i32,
    pub nanosec: u32,
    pub quat_xyzw: [f64; 4],
    pub cov_orient: [f64; 9],
    pub ang_vel: [f64; 3],
    pub cov_ang_vel: [f64; 9],
    pub linear_accel: [f64; 3],
    pub cov_linear_accel: [f64; 9],
}
static IMU_SNAPSHOT: Mutex<CriticalSectionRawMutex, ImuSnapshot> = Mutex::new(ImuSnapshot {
    sec: 0,
    nanosec: 0,
    quat_xyzw: [0.0, 0.0, 0.0, 1.0],
    cov_orient: [0.0; 9],
    ang_vel: [0.0; 3],
    cov_ang_vel: [0.0; 9],
    linear_accel: [0.0; 3],
    cov_linear_accel: [0.0; 9],
});

// ----------------------------------------------------------------------------
// 4. 태스크 정의
// ----------------------------------------------------------------------------

/// 100 Hz 하드 실시간 선점 인터럽트 태스크 (Priority::P6)
#[embassy_executor::task]
async fn task_rt_imu_loop() {
    info!("[RT-IMU] 100 Hz 하드 실시간 인터럽트 루프 시작");
    let mut ticker = Ticker::every(Duration::from_hz(100));
    let mut step_count: u32 = 0;
    let mut buf = [0u8; 12];

    let cov_orient = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4];
    let cov_ang_vel = [1e-5, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0, 0.0, 1e-5];
    let cov_accel = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3];

    loop {
        ticker.next().await;
        IS_RT_ACTIVE.store(true, Ordering::Relaxed);

        let (ax, ay, az, gx_rad, gy_rad, gz_rad) = {
            let mut guard = I2C_BUS.lock().await;
            if let Some(ref mut i2c) = *guard {
                if i2c.write_read(ADDR_LSM6DSO, &[lsm6dso::OUTX_L_G], &mut buf).await.is_ok() {
                    let gx_raw = i16::from_le_bytes([buf[0], buf[1]]);
                    let gy_raw = i16::from_le_bytes([buf[2], buf[3]]);
                    let gz_raw = i16::from_le_bytes([buf[4], buf[5]]);
                    let ax_raw = i16::from_le_bytes([buf[6], buf[7]]);
                    let ay_raw = i16::from_le_bytes([buf[8], buf[9]]);
                    let az_raw = i16::from_le_bytes([buf[10], buf[11]]);

                    let a_x = lsm6dso::raw_to_mps2_f32(ax_raw);
                    let a_y = lsm6dso::raw_to_mps2_f32(ay_raw);
                    let a_z = lsm6dso::raw_to_mps2_f32(az_raw);

                    let g_x = (gx_raw as f32 * 0.070) * DEG_TO_RAD;
                    let g_y = (gy_raw as f32 * 0.070) * DEG_TO_RAD;
                    let g_z = (gz_raw as f32 * 0.070) * DEG_TO_RAD;
                    (a_x, a_y, a_z, g_x, g_y, g_z)
                } else {
                    (0.0, 0.0, 9.80665, 0.0, 0.0, 0.0)
                }
            } else {
                (0.0, 0.0, 9.80665, 0.0, 0.0, 0.0)
            }
        };

        let dt = 0.010f32; // 100 Hz (10ms)

        let mut inekf = INEKF_FILTER.lock().await;
        inekf.predict([gx_rad, gy_rad, gz_rad], dt);
        inekf.update_accel([ax, ay, az]);

        let quat = inekf.rot.to_quaternion();
        step_count = step_count.wrapping_add(1);

        let sec = (step_count / 100) as i32;
        let nanosec = (step_count % 100) * 10_000_000;

        // 스냅샷 원자적 갱신 (ROS 2 쿼터니언 x, y, z, w 순서: quat는 [w, x, y, z])
        {
            let mut snap = IMU_SNAPSHOT.lock().await;
            snap.sec = sec;
            snap.nanosec = nanosec;
            snap.quat_xyzw = [quat[1] as f64, quat[2] as f64, quat[3] as f64, quat[0] as f64];
            snap.cov_orient = cov_orient;
            snap.ang_vel = [gx_rad as f64, gy_rad as f64, gz_rad as f64];
            snap.cov_ang_vel = cov_ang_vel;
            snap.linear_accel = [ax as f64, ay as f64, az as f64];
            snap.cov_linear_accel = cov_accel;
        }

        IS_RT_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// 환경 센서 폴링 태스크 (10 Hz / 1 Hz)
#[embassy_executor::task]
async fn task_env_sensor_loop() {
    let mut ticker_10hz = Ticker::every(Duration::from_hz(10));
    let mut count_1hz: u8 = 0;
    let mut mag_buf = [0u8; 6];
    let mut press_buf = [0u8; 3];
    let mut temp_buf = [0u8; 2];

    loop {
        ticker_10hz.next().await;
        count_1hz = (count_1hz + 1) % 10;

        let mut guard = I2C_BUS.lock().await;
        if let Some(ref mut i2c) = *guard {
            // LIS2MDL 지자기 10Hz 읽기 (mGauss -> Tesla: 1 mG = 1e-7 T)
            if i2c.write_read(ADDR_LIS2MDL, &[lis2mdl::OUTX_L_REG], &mut mag_buf).await.is_ok() {
                let mx = i16::from_le_bytes([mag_buf[0], mag_buf[1]]) as f64 * 1.5 * 1e-7;
                let my = i16::from_le_bytes([mag_buf[2], mag_buf[3]]) as f64 * 1.5 * 1e-7;
                let mz = i16::from_le_bytes([mag_buf[4], mag_buf[5]]) as f64 * 1.5 * 1e-7;

                let mut env = ENV_DATA.lock().await;
                env.mag_tesla = [mx, my, mz];
            }

            // 1 Hz 환경 센서 읽기
            if count_1hz == 0 {
                // LPS22HH 기압 (24비트 / 4096 = hPa -> * 100.0 = Pa)
                let press_pa = if i2c.write_read(ADDR_LPS22HH, &[lps22hh::PRESS_OUT_XL], &mut press_buf).await.is_ok() {
                    let raw = (press_buf[0] as u32) | ((press_buf[1] as u32) << 8) | ((press_buf[2] as u32) << 16);
                    (raw as f64 / 4096.0) * 100.0
                } else {
                    101325.0
                };

                // STTS751 정밀 온도 (°C)
                let temp_c = if i2c.write_read(ADDR_STTS751, &[stts751::TEMP_HIGH], &mut temp_buf).await.is_ok() {
                    let hi = temp_buf[0] as i8 as f64;
                    let lo = (temp_buf[1] >> 4) as f64 * 0.0625;
                    hi + lo
                } else {
                    25.0
                };

                // HTS221 상대 습도 (0.0 ~ 1.0)
                let hum_ratio = 0.50; // 기본값

                let mut env = ENV_DATA.lock().await;
                env.pressure_pa = press_pa;
                env.temperature_c = temp_c;
                env.humidity_ratio = hum_ratio;
            }
        }
    }
}

/// Zenoh 텔레메트리 발행 및 cmd_vel/set_led 송수신 태스크
#[embassy_executor::task]
async fn task_zenoh_udp(stack: Stack<'static>) {
    info!("[Zenoh] UDP 소켓 초기화 (Port: {})", ZENOH_UDP_PORT);

    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut rx_buf = [0u8; 1024];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_buf = [0u8; 1024];

    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buf,
        &mut tx_meta,
        &mut tx_buf,
    );

    if let Err(e) = socket.bind(ZENOH_UDP_PORT) {
        error!("[Zenoh] 소켓 바인드 실패: {:?}", e);
        return;
    }

    info!("[Zenoh] 7447 바인드 완료. DHCP IP 할당 대기...");
    while !stack.is_config_up() {
        embassy_time::Timer::after(Duration::from_millis(100)).await;
    }
    info!("[Zenoh] 네트워크 활성화 완료. 텔레메트리 및 수신 서비스 시작.");

    let remote_host = embassy_net::IpEndpoint::new(
        embassy_net::IpAddress::Ipv4(Ipv4Address::new(192, 168, 50, 15)),
        ZENOH_UDP_PORT,
    );

    let mut frame_buf = [0u8; 1500];
    let mut rx_packet = [0u8; 1500];
    let mut cdr_buf = [0u8; 1024];
    let mut att_buf = [0u8; 33];

    let mut frame_seq: u32 = 0;
    let mut seq_imu: i64 = 0;
    let mut seq_mag: i64 = 0;
    let mut seq_press: i64 = 0;
    let mut seq_temp: i64 = 0;
    let mut seq_hum: i64 = 0;

    let mut ticker_100hz = Ticker::every(Duration::from_hz(100));
    let mut div_10hz: u8 = 0;
    let mut div_1hz: u16 = 0;

    let cov_mag = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6];

    let zid_bytes: [u8; 16] = [
        0x10, 0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09,
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    ];

    // 토픽(Publisher)별 독립 GID 정의 (마지막 바이트로 고유 식별)
    let mut gid_imu = zid_bytes;
    gid_imu[15] = 0x01;
    let mut gid_mag = zid_bytes;
    gid_mag[15] = 0x02;
    let mut gid_press = zid_bytes;
    gid_press[15] = 0x03;
    let mut gid_temp = zid_bytes;
    gid_temp[15] = 0x04;
    let mut gid_hum = zid_bytes;
    gid_hum[15] = 0x05;

    let mut session_established = false;
    let mut last_rx_instant = Instant::now();
    let mut init_timer_tick: u8 = 0;

    // 라우터(192.168.50.15:7447)에 세션 초기화 InitSyn 최초 전송
    let init_len = ZenohWire::build_init_syn(&mut frame_buf, &zid_bytes);
    let _ = socket.send_to(&frame_buf[..init_len], remote_host).await;

    loop {
        match select(ticker_100hz.next(), socket.recv_from(&mut rx_packet)).await {
            Either::First(_) => {
                if !session_established {
                    init_timer_tick = (init_timer_tick + 1) % 30; // 300ms 주기 (100Hz 30회)
                    if init_timer_tick == 0 {
                        let init_len = ZenohWire::build_init_syn(&mut frame_buf, &zid_bytes);
                        let _ = socket.send_to(&frame_buf[..init_len], remote_host).await;
                    }
                    continue;
                }

                div_10hz = (div_10hz + 1) % 10;
                div_1hz = (div_1hz + 1) % 100;

                // 1. 100 Hz IMU 데이터 발행
                let snap = *IMU_SNAPSHOT.lock().await;
                let imu_msg = Imu {
                    header: Header::new(snap.sec, snap.nanosec, "imu_link"),
                    orientation: Quaternion::new(snap.quat_xyzw[0], snap.quat_xyzw[1], snap.quat_xyzw[2], snap.quat_xyzw[3]),
                    orientation_covariance: snap.cov_orient,
                    angular_velocity: Vector3::new(snap.ang_vel[0], snap.ang_vel[1], snap.ang_vel[2]),
                    angular_velocity_covariance: snap.cov_ang_vel,
                    linear_acceleration: Vector3::new(snap.linear_accel[0], snap.linear_accel[1], snap.linear_accel[2]),
                    linear_acceleration_covariance: snap.cov_linear_accel,
                };
                let cdr_len = imu_msg.encode_cdr(&mut cdr_buf);

                frame_seq = frame_seq.wrapping_add(1);
                seq_imu = seq_imu.wrapping_add(1);
                let time_ns = (snap.sec as i64) * 1_000_000_000 + (snap.nanosec as i64);
                ZenohWire::build_rmw_attachment(&mut att_buf, seq_imu, time_ns, &gid_imu);
                let frame_len = ZenohWire::build_push_put_with_attachment(&mut frame_buf, frame_seq, msg_endpoints::KEY_IMU_DATA, Some(&att_buf), &cdr_buf[..cdr_len]);
                let _ = socket.send_to(&frame_buf[..frame_len], remote_host).await;

                // 2. 10 Hz 지자기 데이터 발행
                if div_10hz == 0 {
                    let env = *ENV_DATA.lock().await;
                    let mag_msg = MagneticField::new(
                        Header::new(snap.sec, snap.nanosec, "imu_link"),
                        Vector3::new(env.mag_tesla[0], env.mag_tesla[1], env.mag_tesla[2]),
                        cov_mag,
                    );
                    let cdr_len = mag_msg.encode_cdr(&mut cdr_buf);
                    frame_seq = frame_seq.wrapping_add(1);
                    seq_mag = seq_mag.wrapping_add(1);
                    ZenohWire::build_rmw_attachment(&mut att_buf, seq_mag, time_ns, &gid_mag);
                    let frame_len = ZenohWire::build_push_put_with_attachment(&mut frame_buf, frame_seq, msg_endpoints::KEY_IMU_MAG, Some(&att_buf), &cdr_buf[..cdr_len]);
                    let _ = socket.send_to(&frame_buf[..frame_len], remote_host).await;
                }

                // 3. 1 Hz 기압, 온도, 습도 발행, Liveliness Token 갱신, 라우터 생존성 감시
                if div_1hz == 0 {
                    let env = *ENV_DATA.lock().await;
                    let hdr = Header::new(snap.sec, snap.nanosec, "imu_link");

                    // Pressure
                    let press_msg = FluidPressure::new(hdr, env.pressure_pa, 0.0);
                    let p_len = press_msg.encode_cdr(&mut cdr_buf);
                    frame_seq = frame_seq.wrapping_add(1);
                    seq_press = seq_press.wrapping_add(1);
                    ZenohWire::build_rmw_attachment(&mut att_buf, seq_press, time_ns, &gid_press);
                    let f_len = ZenohWire::build_push_put_with_attachment(&mut frame_buf, frame_seq, msg_endpoints::KEY_PRESSURE, Some(&att_buf), &cdr_buf[..p_len]);
                    let _ = socket.send_to(&frame_buf[..f_len], remote_host).await;

                    // Temperature
                    let temp_msg = Temperature::new(hdr, env.temperature_c, 0.0);
                    let t_len = temp_msg.encode_cdr(&mut cdr_buf);
                    frame_seq = frame_seq.wrapping_add(1);
                    seq_temp = seq_temp.wrapping_add(1);
                    ZenohWire::build_rmw_attachment(&mut att_buf, seq_temp, time_ns, &gid_temp);
                    let f_len = ZenohWire::build_push_put_with_attachment(&mut frame_buf, frame_seq, msg_endpoints::KEY_TEMPERATURE, Some(&att_buf), &cdr_buf[..t_len]);
                    let _ = socket.send_to(&frame_buf[..f_len], remote_host).await;

                    // Humidity
                    let hum_msg = RelativeHumidity::new(hdr, env.humidity_ratio, 0.0);
                    let h_len = hum_msg.encode_cdr(&mut cdr_buf);
                    frame_seq = frame_seq.wrapping_add(1);
                    seq_hum = seq_hum.wrapping_add(1);
                    ZenohWire::build_rmw_attachment(&mut att_buf, seq_hum, time_ns, &gid_hum);
                    let f_len = ZenohWire::build_push_put_with_attachment(&mut frame_buf, frame_seq, msg_endpoints::KEY_HUMIDITY, Some(&att_buf), &cdr_buf[..h_len]);
                    let _ = socket.send_to(&frame_buf[..f_len], remote_host).await;

                    // 4. ROS 2 Jazzy Liveliness Token 정기 발행 (1 Hz: 토픽 7종 + 서비스 1종)
                    for (i, &lv) in msg_endpoints::TOPIC_LIVELINESS_TOKENS.iter().enumerate() {
                        frame_seq = frame_seq.wrapping_add(1);
                        let lv_decl_len = ZenohWire::build_declare_token(&mut frame_buf, frame_seq, (i + 10) as u32, lv);
                        let _ = socket.send_to(&frame_buf[..lv_decl_len], remote_host).await;

                        frame_seq = frame_seq.wrapping_add(1);
                        let lv_len = ZenohWire::build_push_put(&mut frame_buf, frame_seq, lv, &[]);
                        let _ = socket.send_to(&frame_buf[..lv_len], remote_host).await;
                    }
                    frame_seq = frame_seq.wrapping_add(1);
                    let lv_decl_len = ZenohWire::build_declare_token(&mut frame_buf, frame_seq, 21, srv_endpoints::TOKEN_SET_LED);
                    let _ = socket.send_to(&frame_buf[..lv_decl_len], remote_host).await;

                    frame_seq = frame_seq.wrapping_add(1);
                    let lv_len = ZenohWire::build_push_put(&mut frame_buf, frame_seq, srv_endpoints::TOKEN_SET_LED, &[]);
                    let _ = socket.send_to(&frame_buf[..lv_len], remote_host).await;

                    // 5. 라우터 생존성 감시 및 자동 재연결(Auto-Reconnect) 트리거
                    let elapsed = Instant::now().duration_since(last_rx_instant);
                    if elapsed > Duration::from_secs(3) {
                        info!("[Zenoh] 3초간 라우터 응답 없음 -> 세션 단절 감지. 재연결(Reconnection) 모드로 전환.");
                        session_established = false;
                        let init_len = ZenohWire::build_init_syn(&mut frame_buf, &zid_bytes);
                        let _ = socket.send_to(&frame_buf[..init_len], remote_host).await;
                    } else if elapsed > Duration::from_secs(1) {
                        // 1초 이상 무수신 시 라우터 생존 확인용 InitSyn 프로브 전송
                        let init_len = ZenohWire::build_init_syn(&mut frame_buf, &zid_bytes);
                        let _ = socket.send_to(&frame_buf[..init_len], remote_host).await;
                    }
                }
            }
            Either::Second(Ok((len, remote))) => {
                last_rx_instant = Instant::now();

                // 1. InitAck 수신 처리 (세션 초기화 또는 재연결 응답)
                if let Some(cookie) = ZenohWire::parse_init_ack(&rx_packet[..len]) {
                    info!("[Zenoh] InitAck 수신! Cookie 길이: {=usize}. OpenSyn 전송...", cookie.len());
                    let open_len = ZenohWire::build_open_syn(&mut frame_buf, cookie);
                    let _ = socket.send_to(&frame_buf[..open_len], remote_host).await;
                    continue;
                }

                // 2. OpenAck 수신 처리 -> 세션 수립 및 모든 엔티티(Service & Liveliness Tokens) 재등록
                if ZenohWire::is_open_ack(&rx_packet[..len]) {
                    info!("[Zenoh] OpenAck 수신! Zenoh 세션 수립(Established) 성공! 텔레메트리 스트리밍 시작.");
                    session_established = true;
                    frame_seq = 0;
                    seq_imu = 0;
                    seq_mag = 0;
                    seq_press = 0;
                    seq_temp = 0;
                    seq_hum = 0;

                    // 1. /nucleo/set_led Queryable 등록 (DECLARE_QUERYABLE)
                    frame_seq = frame_seq.wrapping_add(1);
                    let qable_len = ZenohWire::build_declare_queryable(&mut frame_buf, frame_seq, 1, srv_endpoints::KEY_SET_LED);
                    let _ = socket.send_to(&frame_buf[..qable_len], remote_host).await;

                    // 2. 세션 수립 즉시 ROS 2 Jazzy Liveliness Token 발행 (토픽 7종: NN + MP 5종 + MS 1종)
                    for (i, &lv) in msg_endpoints::TOPIC_LIVELINESS_TOKENS.iter().enumerate() {
                        frame_seq = frame_seq.wrapping_add(1);
                        let lv_decl_len = ZenohWire::build_declare_token(&mut frame_buf, frame_seq, (i + 10) as u32, lv);
                        let _ = socket.send_to(&frame_buf[..lv_decl_len], remote_host).await;

                        frame_seq = frame_seq.wrapping_add(1);
                        let lv_len = ZenohWire::build_push_put(&mut frame_buf, frame_seq, lv, &[]);
                        let _ = socket.send_to(&frame_buf[..lv_len], remote_host).await;
                    }
                    frame_seq = frame_seq.wrapping_add(1);
                    let lv_decl_len = ZenohWire::build_declare_token(&mut frame_buf, frame_seq, 21, srv_endpoints::TOKEN_SET_LED);
                    let _ = socket.send_to(&frame_buf[..lv_decl_len], remote_host).await;

                    frame_seq = frame_seq.wrapping_add(1);
                    let lv_len = ZenohWire::build_push_put(&mut frame_buf, frame_seq, srv_endpoints::TOKEN_SET_LED, &[]);
                    let _ = socket.send_to(&frame_buf[..lv_len], remote_host).await;
                    continue;
                }

                // 세션 미수립 상태에서는 일반 데이터 프레임 무시
                if !session_established {
                    continue;
                }

                if let Some((msg_type, key_expr, q_id, payload)) = ZenohWire::parse_frame(&rx_packet[..len]) {
                    if (key_expr.contains("cmd_vel") || (key_expr.is_empty() && payload.len() >= 48)) && (msg_type == msg_id::PUSH || msg_type == msg_id::PUT) {
                        let actual_payload = if let Some(pos) = payload.windows(4).position(|w| w == [0x00, 0x01, 0x00, 0x00]) {
                            &payload[pos..]
                        } else {
                            payload
                        };
                        if let Some(cmd) = Twist::decode_cdr(actual_payload) {
                            info!("[ROS2 cmd_vel] linear.x={=f64} m/s, angular.z={=f64} rad/s", cmd.linear.x, cmd.angular.z);
                        }
                    } else if (key_expr.contains("set_led") || key_expr.is_empty()) && (msg_type == msg_id::QUERY || msg_type == msg_id::REQUEST) {
                        let actual_payload = if let Some(pos) = payload.windows(4).position(|w| w == [0x00, 0x01, 0x00, 0x00]) {
                            &payload[pos..]
                        } else {
                            payload
                        };
                        if let Some(req) = set_bool::Request::decode_cdr(actual_payload) {
                            let mut led_guard = USER_LED.lock().await;
                            if let Some(ref mut led) = *led_guard {
                                if req.data {
                                    led.set_high();
                                } else {
                                    led.set_low();
                                }
                            }
                            info!("[ROS2 Service] /nucleo/set_led -> data={=bool} 처리 완료", req.data);

                            // Response 직렬화 및 Zenoh REPLY 전송
                            let resp = set_bool::Response::new(true, if req.data { "LED ON" } else { "LED OFF" });
                            let resp_len = resp.encode_cdr(&mut cdr_buf);
                            frame_seq = frame_seq.wrapping_add(1);
                            ZenohWire::build_rmw_attachment(&mut att_buf, 1, 0, &zid_bytes);
                            let rep_len = ZenohWire::build_reply(&mut frame_buf, frame_seq, q_id, srv_endpoints::KEY_SET_LED, Some(&att_buf), &cdr_buf[..resp_len]);
                            let _ = socket.send_to(&frame_buf[..rep_len], remote).await;
                        }
                    }
                }
            }
            Either::Second(Err(_)) => {}
        }
    }
}

/// 1 Hz 텔레메트리 RTT 콘솔 리포터
#[embassy_executor::task]
async fn task_rtt_reporter(stack: Stack<'static>) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;

        let ip = stack.config_v4().map(|c| c.address.address()).unwrap_or(Ipv4Address::new(0, 0, 0, 0));
        let idle = TICK_IDLE_COUNT.swap(0, Ordering::Relaxed);
        let busy = TICK_BUSY_COUNT.swap(0, Ordering::Relaxed);
        let total = idle + busy;
        let cpu_load = if total > 0 { (busy as f32 / total as f32) * 100.0 } else { 0.0 };

        info!("[ROS2 Node 1Hz] IP: {} | CPU: {=f32}% (Idle: {=u32}, Busy: {=u32})", ip, cpu_load, idle, busy);
    }
}

#[embassy_executor::task]
async fn task_net_runner(mut runner: embassy_net::Runner<'static, Ethernet<'static, ETH, GenericSMI>>) {
    runner.run().await
}

// ----------------------------------------------------------------------------
// 5. 엔트리포인트 (메인 진입점)
// ----------------------------------------------------------------------------
static RT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static PACKET_QUEUE: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
static STACK: StaticCell<Stack<'static>> = StaticCell::new();

#[embassy_stm32::interrupt]
unsafe fn CEC() {
    RT_EXECUTOR.on_interrupt();
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());
    info!(">>> NUCLEO-H743ZI2 임베디드 ROS 2 노드 시작 (06_ros2_node) <<<");

    unsafe {
        init_proc_stat_timer();
    }

    let executor = cortex_m::singleton!(: embassy_executor::raw::Executor = embassy_executor::raw::Executor::new(core::ptr::null_mut())).unwrap();
    let spawner = executor.spawner();

    spawner.must_spawn(main_task(spawner, p));

    loop {
        IS_SLEEPING.store(false, Ordering::Relaxed);
        unsafe {
            executor.poll();
        }
        IS_SLEEPING.store(true, Ordering::Release);
        cortex_m::asm::wfe();
        IS_SLEEPING.store(false, Ordering::Release);
    }
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner, p: embassy_stm32::Peripherals) {
    // User LED (PB0: LD1 Green)
    let led1 = Output::new(p.PB0, Level::Low, Speed::Low);
    {
        let mut guard = USER_LED.lock().await;
        *guard = Some(led1);
    }

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
        let mut guard = I2C_BUS.lock().await;
        *guard = Some(i2c);
    }

    // 센서 초기화
    {
        let mut guard = I2C_BUS.lock().await;
        if let Some(ref mut i2c) = *guard {
            // 1. LSM6DSO
            let _ = i2c.write(ADDR_LSM6DSO, &[lsm6dso::CTRL1_XL, lsm6dso::VAL_CTRL1_XL_104HZ_2G]).await;
            let _ = i2c.write(ADDR_LSM6DSO, &[lsm6dso::CTRL2_G, lsm6dso::VAL_CTRL2_G_104HZ_250DPS]).await;
            // 2. LIS2MDL
            let _ = i2c.write(ADDR_LIS2MDL, &[lis2mdl::CFG_REG_A, lis2mdl::VAL_CFG_REG_A_10HZ_CONT]).await;
            // 3. LPS22HH
            let _ = i2c.write(ADDR_LPS22HH, &[lps22hh::CTRL_REG1, lps22hh::VAL_CTRL_REG1_1HZ_BDU]).await;
            // 4. STTS751
            let _ = i2c.write(ADDR_STTS751, &[stts751::CONFIG, stts751::VAL_CONFIG_CONTINUOUS]).await;
            let _ = i2c.write(ADDR_STTS751, &[stts751::CONVERSION_RATE, stts751::VAL_RATE_1_CONV_PER_SEC]).await;
            // 5. HTS221
            let _ = i2c.write(ADDR_HTS221, &[hts221::CTRL_REG1, hts221::VAL_CTRL_REG1_1HZ_PD_BDU]).await;
        }
    }

    // LAN8742A 이더넷 RMII
    let rmii_pins = BoardRmiiPins::new(
        p.PA1,
        p.PA2,
        p.PC1,
        p.PA7,
        p.PC4,
        p.PC5,
        p.PG13,
        p.PB13,
        p.PG11,
    );

    let mac_addr = [0x02, 0x80, 0xE1, 0x1D, 0x20, 0x06];
    let queue = PACKET_QUEUE.init(PacketQueue::new());

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

    let (net_stack, runner) = embassy_net::new(
        eth_device,
        embassy_net::Config::dhcpv4(Default::default()),
        STACK_RESOURCES.init(StackResources::new()),
        0x1234_5678,
    );
    let stack = *STACK.init(net_stack);

    // RT Executor (CEC) 초기화
    interrupt::CEC.set_priority(Priority::P6);
    let rt_spawner = RT_EXECUTOR.start(interrupt::CEC);
    rt_spawner.spawn(task_rt_imu_loop()).unwrap();

    // 비동기 태스크 스폰
    spawner.spawn(task_net_runner(runner)).unwrap();
    spawner.spawn(task_env_sensor_loop()).unwrap();
    spawner.spawn(task_zenoh_udp(stack)).unwrap();
    spawner.spawn(task_rtt_reporter(stack)).unwrap();
}
