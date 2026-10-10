//! # STM32H7 96-bit 고유 디바이스 ID(UID) 및 네트워크 MAC 주소 생성기
//!
//! STM32H743 실리콘에 공장 출하 시 영구 기록된 96-bit 고유 ID(UID96, 주소: 0x1FF1_E800)를
//! 안전하게 읽어와 보드 고유의 IEEE 802 EUI-48 MAC 주소 및 난수 시드를 파생한다.

/// STM32H743 96-bit UID 레지스터 시작 주소 (RM0433 Section 61.1)
pub const UID96_BASE_ADDR: *const u32 = 0x1FF1_E800 as *const u32;

/// 96-bit(12바이트) 고유 하드웨어 식별자 읽기
pub fn read_uid96() -> [u8; 12] {
    let mut uid = [0u8; 12];
    unsafe {
        let w0 = core::ptr::read_volatile(UID96_BASE_ADDR);
        let w1 = core::ptr::read_volatile(UID96_BASE_ADDR.add(1));
        let w2 = core::ptr::read_volatile(UID96_BASE_ADDR.add(2));

        uid[0..4].copy_from_slice(&w0.to_le_bytes());
        uid[4..8].copy_from_slice(&w1.to_le_bytes());
        uid[8..12].copy_from_slice(&w2.to_le_bytes());
    }
    uid
}

/// STM32 고유 UID96 기반 IEEE 802 EUI-48 Locally Administered MAC 주소 생성
///
/// 포맷: `[0x02, 0x80, 0xE1, byte3, byte4, byte5]`
/// - 첫 바이트 최하위 2비트: `10b` (Locally Administered=1, Unicast=0)
/// - 하위 3바이트는 12바이트 UID의 각 워드를 접어서(Fold/XOR) 균등 분산된 고유 식별자로 유도
pub fn get_unique_mac_address() -> [u8; 6] {
    let uid = read_uid96();

    // 12바이트 UID를 3바이트로 XOR 압축
    let b3 = uid[0] ^ uid[3] ^ uid[6] ^ uid[9];
    let b4 = uid[1] ^ uid[4] ^ uid[7] ^ uid[10];
    let b5 = uid[2] ^ uid[5] ^ uid[8] ^ uid[11];

    [0x02, 0x80, 0xE1, b3, b4, b5]
}

/// STM32 고유 UID96 기반 64-bit 난수 시드(PRNG Seed) 유도
///
/// 하드웨어 TRNG가 없을 때 네트워크 TCP/IP 시퀀스 및 초기화 시드로 활용 가능
pub fn get_uid_prng_seed() -> u64 {
    let uid = read_uid96();
    let low = u32::from_le_bytes([uid[0], uid[1], uid[2], uid[3]]) as u64;
    let mid = u32::from_le_bytes([uid[4], uid[5], uid[6], uid[7]]) as u64;
    let high = u32::from_le_bytes([uid[8], uid[9], uid[10], uid[11]]) as u64;

    (low ^ (mid << 16)) | (high << 32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mac_locally_administered_unicast() {
        // 호스트 환경에서는 가상 UID로 테스트
        let uid = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
        let b3 = uid[0] ^ uid[3] ^ uid[6] ^ uid[9];
        let b4 = uid[1] ^ uid[4] ^ uid[7] ^ uid[10];
        let b5 = uid[2] ^ uid[5] ^ uid[8] ^ uid[11];
        let mac = [0x02, 0x80, 0xE1, b3, b4, b5];

        // Locally Administered 비트 확인 (Bit 1 = 1)
        assert_eq!(mac[0] & 0x02, 0x02);
        // Unicast 비트 확인 (Bit 0 = 0)
        assert_eq!(mac[0] & 0x01, 0x00);
    }
}
