//! Pico-ROS C FFI 바인딩 및 Embassy 어댑터 인터페이스
//!
//! NUCLEO-H743ZI2 (Cortex-M7) 임베디드 베어메탈 환경에서
//! C Pico-ROS / zenoh-pico 런타임과 Rust embassy-net 비동기 스택을 1:1 결합한다.

use core::ffi::c_char;
use embassy_time::Instant;

// C FFI 선언
extern "C" {
    pub fn picoros_rust_init(locator: *const c_char) -> i32;
    pub fn picoros_rust_node_init(name: *const c_char, domain_id: u32) -> i32;
    pub fn picoros_rust_spin_once() -> i32;

    pub fn picoros_rust_pub_create(
        name: *const c_char,
        type_: *const c_char,
        hash: *const c_char,
    ) -> *mut core::ffi::c_void;

    pub fn picoros_rust_pub_send(
        handle: *mut core::ffi::c_void,
        payload: *const u8,
        len: usize,
    ) -> i32;

    #[allow(dead_code)]
    pub fn picoros_rust_sub_create(
        name: *const c_char,
        type_: *const c_char,
        hash: *const c_char,
        cb: extern "C" fn(*const u8, usize),
    ) -> *mut core::ffi::c_void;

    #[allow(dead_code)]
    pub fn picoros_rust_srv_create(
        name: *const c_char,
        type_: *const c_char,
        hash: *const c_char,
        cb: extern "C" fn(*const u8, usize, *mut u8, usize) -> usize,
    ) -> *mut core::ffi::c_void;
}

// -----------------------------------------------------------------------------
// C 라이브러리(zenoh-pico)가 호출하는 Rust 콜백 함수들 (Reverse FFI)
// -----------------------------------------------------------------------------

/// 밀리초 단위 시스템 시간 반환
#[no_mangle]
pub extern "C" fn rust_embassy_time_now_ms() -> u64 {
    Instant::now().as_millis()
}

/// UDP 수신 버퍼 큐 (Rust embassy-net 태스크가 채우고 C zenoh-pico가 읽음)
static mut RX_BUF: [u8; 1500] = [0; 1500];
static mut RX_LEN: usize = 0;

pub fn push_rx_packet(data: &[u8]) {
    unsafe {
        let copy_len = data.len().min(1500);
        let rx_ptr = core::ptr::addr_of_mut!(RX_BUF) as *mut u8;
        core::ptr::copy_nonoverlapping(data.as_ptr(), rx_ptr, copy_len);
        RX_LEN = copy_len;
    }
}

#[no_mangle]
pub extern "C" fn rust_embassy_udp_read(ptr: *mut u8, len: usize) -> usize {
    unsafe {
        if RX_LEN == 0 {
            return 0;
        }
        let read_len = RX_LEN.min(len);
        let rx_ptr = core::ptr::addr_of!(RX_BUF) as *const u8;
        core::ptr::copy_nonoverlapping(rx_ptr, ptr, read_len);
        RX_LEN = 0; // 패킷 소비
        read_len
    }
}

/// UDP 송신 버퍼 (C zenoh-pico가 쓰고 Rust embassy-net 태스크가 읽어 실제 전송)
static mut TX_BUF: [u8; 1500] = [0; 1500];
static mut TX_LEN: usize = 0;

pub fn take_tx_packet(out: &mut [u8]) -> usize {
    unsafe {
        if TX_LEN == 0 {
            return 0;
        }
        let len = TX_LEN.min(out.len());
        let tx_ptr = core::ptr::addr_of!(TX_BUF) as *const u8;
        core::ptr::copy_nonoverlapping(tx_ptr, out.as_mut_ptr(), len);
        TX_LEN = 0;
        len
    }
}

#[no_mangle]
pub extern "C" fn rust_embassy_udp_write(
    ptr: *const u8,
    len: usize,
    _addr: *const c_char,
    _port: u16,
) -> usize {
    unsafe {
        let write_len = len.min(1500);
        let tx_ptr = core::ptr::addr_of_mut!(TX_BUF) as *mut u8;
        core::ptr::copy_nonoverlapping(ptr, tx_ptr, write_len);
        TX_LEN = write_len;
        write_len
    }
}

// -----------------------------------------------------------------------------
// Safe Rust Wrapper
// -----------------------------------------------------------------------------

pub struct PicoRosPublisher {
    handle: *mut core::ffi::c_void,
}

unsafe impl Send for PicoRosPublisher {}

impl PicoRosPublisher {
    pub fn publish(&self, data: &[u8]) -> bool {
        if self.handle.is_null() {
            return false;
        }
        unsafe { picoros_rust_pub_send(self.handle, data.as_ptr(), data.len()) == 0 }
    }
}

pub struct PicoRosNode;

impl PicoRosNode {
    pub fn init(locator: &str, node_name: &str, domain_id: u32) -> Option<Self> {
        let mut loc_buf = heapless::Vec::<u8, 64>::new();
        loc_buf.extend_from_slice(locator.as_bytes()).ok()?;
        loc_buf.push(0).ok()?;

        let mut name_buf = heapless::Vec::<u8, 64>::new();
        name_buf.extend_from_slice(node_name.as_bytes()).ok()?;
        name_buf.push(0).ok()?;

        unsafe {
            let res_init = picoros_rust_init(loc_buf.as_ptr() as *const c_char);
            if res_init != 0 {
                defmt::error!("[Pico-ROS] picoros_rust_init error: {=i32}", res_init);
                return None;
            }
            let res_node = picoros_rust_node_init(name_buf.as_ptr() as *const c_char, domain_id);
            if res_node != 0 {
                defmt::error!("[Pico-ROS] picoros_rust_node_init error: {=i32}", res_node);
                return None;
            }
        }
        Some(Self)
    }

    pub fn create_publisher(
        &self,
        name: &str,
        msg_type: &str,
        hash: &str,
    ) -> Option<PicoRosPublisher> {
        let mut name_buf = heapless::Vec::<u8, 128>::new();
        name_buf.extend_from_slice(name.as_bytes()).ok()?;
        name_buf.push(0).ok()?;

        let mut type_buf = heapless::Vec::<u8, 128>::new();
        type_buf.extend_from_slice(msg_type.as_bytes()).ok()?;
        type_buf.push(0).ok()?;

        let mut hash_buf = heapless::Vec::<u8, 128>::new();
        hash_buf.extend_from_slice(hash.as_bytes()).ok()?;
        hash_buf.push(0).ok()?;

        let handle = unsafe {
            picoros_rust_pub_create(
                name_buf.as_ptr() as *const c_char,
                type_buf.as_ptr() as *const c_char,
                hash_buf.as_ptr() as *const c_char,
            )
        };

        if handle.is_null() {
            None
        } else {
            Some(PicoRosPublisher { handle })
        }
    }

    pub fn spin_once(&self) {
        unsafe {
            picoros_rust_spin_once();
        }
    }
}
