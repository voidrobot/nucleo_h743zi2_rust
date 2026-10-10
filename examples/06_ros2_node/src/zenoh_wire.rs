//! # no_std Zenoh 1.0 초경량 와이어 프로토콜 인코더 및 디코더
//!
//! Eclipse Zenoh 1.0 프로토콜 규격에 따라 FRAME(Transport) -> PUSH(Network) -> PUT(Data)
//! 계층 구조를 힙 동적 할당 없이(Zero-Heap Allocation) 고정 버퍼에서 직접 인코딩 및 디코딩한다.


/// Zenoh 프로토콜 메시지 ID
pub mod msg_id {
    pub const FRAME: u8 = 0x05;
    pub const PUSH: u8 = 0x1D;
    pub const PUT: u8 = 0x01;
    pub const QUERY: u8 = 0x07;
    pub const REPLY: u8 = 0x09;
}

/// VLE (Variable-Length Encoding / LEB128) 인코딩
pub fn encode_vle(mut val: usize, buf: &mut [u8]) -> usize {
    let mut offset = 0;
    loop {
        let mut byte = (val & 0x7F) as u8;
        val >>= 7;
        if val != 0 {
            byte |= 0x80;
            if offset < buf.len() {
                buf[offset] = byte;
                offset += 1;
            } else {
                return offset;
            }
        } else {
            if offset < buf.len() {
                buf[offset] = byte;
                offset += 1;
            }
            return offset;
        }
    }
}

/// VLE (Variable-Length Encoding / LEB128) 디코딩 -> (값, 소비된 바이트 수)
pub fn decode_vle(buf: &[u8]) -> Option<(usize, usize)> {
    let mut val: usize = 0;
    let mut shift = 0;
    let mut offset = 0;

    while offset < buf.len() {
        let byte = buf[offset];
        offset += 1;
        val |= ((byte & 0x7F) as usize) << shift;
        if (byte & 0x80) == 0 {
            return Some((val, offset));
        }
        shift += 7;
        if shift > 35 {
            return None; // 오버플로우 방어
        }
    }
    None
}

/// Zenoh 와이어 프레임 빌더
pub struct ZenohWire;

impl ZenohWire {
    /// PUSH / PUT 데이터 발행 프레임 조립 (Transport FRAME + Network PUSH + Data PUT)
    /// Key Expression 예: "0/nucleo/imu/data"
    pub fn build_push_put(
        buf: &mut [u8],
        seq: u32,
        key_expr: &str,
        payload: &[u8],
    ) -> usize {
        let mut offset = 0;

        // 1. FRAME Header
        if offset < buf.len() {
            buf[offset] = msg_id::FRAME;
            offset += 1;
        }
        // Sequence Number (VLE)
        offset += encode_vle(seq as usize, &mut buf[offset..]);

        // 2. Network PUSH Message
        if offset < buf.len() {
            buf[offset] = msg_id::PUSH;
            offset += 1;
        }
        // Key Expression (VLE Length + UTF-8 Bytes)
        offset += encode_vle(key_expr.len(), &mut buf[offset..]);
        let k_len = key_expr.len();
        if offset + k_len <= buf.len() {
            buf[offset..offset + k_len].copy_from_slice(key_expr.as_bytes());
            offset += k_len;
        }

        // 3. Data PUT Sub-message
        if offset < buf.len() {
            buf[offset] = msg_id::PUT;
            offset += 1;
        }
        // Payload (VLE Length + Payload Bytes)
        offset += encode_vle(payload.len(), &mut buf[offset..]);
        let p_len = payload.len();
        if offset + p_len <= buf.len() {
            buf[offset..offset + p_len].copy_from_slice(payload);
            offset += p_len;
        }

        offset
    }

    /// 서비스 Query 응답(REPLY) 프레임 조립
    pub fn build_reply(
        buf: &mut [u8],
        seq: u32,
        query_id: u32,
        payload: &[u8],
    ) -> usize {
        let mut offset = 0;

        // FRAME Header + Seq
        if offset < buf.len() {
            buf[offset] = msg_id::FRAME;
            offset += 1;
        }
        offset += encode_vle(seq as usize, &mut buf[offset..]);

        // REPLY Message
        if offset < buf.len() {
            buf[offset] = msg_id::REPLY;
            offset += 1;
        }
        offset += encode_vle(query_id as usize, &mut buf[offset..]);
        offset += encode_vle(payload.len(), &mut buf[offset..]);

        let p_len = payload.len();
        if offset + p_len <= buf.len() {
            buf[offset..offset + p_len].copy_from_slice(payload);
            offset += p_len;
        }

        offset
    }

    /// 수신된 Zenoh 프레임 디코딩
    /// 반환: Option<(MessageType, KeyExpr, QueryId, Payload)>
    pub fn parse_frame<'a>(
        buf: &'a [u8],
    ) -> Option<(u8, &'a str, u32, &'a [u8])> {
        if buf.is_empty() {
            return None;
        }

        let mut offset = 0;
        let mut msg_type = buf[offset];
        offset += 1;

        // FRAME 컨테이너인 경우 내부 메시지로 전진
        if msg_type == msg_id::FRAME {
            let (_seq, vle_len) = decode_vle(&buf[offset..])?;
            offset += vle_len;
            if offset >= buf.len() {
                return None;
            }
            msg_type = buf[offset];
            offset += 1;
        }

        match msg_type {
            msg_id::PUSH => {
                // Key Expression 파싱
                let (k_len, vle_len) = decode_vle(&buf[offset..])?;
                offset += vle_len;
                if offset + k_len > buf.len() {
                    return None;
                }
                let key_expr = core::str::from_utf8(&buf[offset..offset + k_len]).ok()?;
                offset += k_len;

                // PUT Sub-message 파싱
                if offset >= buf.len() || buf[offset] != msg_id::PUT {
                    return None;
                }
                offset += 1;

                let (p_len, vle_len) = decode_vle(&buf[offset..])?;
                offset += vle_len;
                if offset + p_len > buf.len() {
                    return None;
                }
                let payload = &buf[offset..offset + p_len];

                Some((msg_id::PUSH, key_expr, 0, payload))
            }
            msg_id::QUERY => {
                // Query ID 파싱
                let (q_id, vle_len) = decode_vle(&buf[offset..])?;
                offset += vle_len;

                // Key Expression 파싱
                let (k_len, vle_len) = decode_vle(&buf[offset..])?;
                offset += vle_len;
                if offset + k_len > buf.len() {
                    return None;
                }
                let key_expr = core::str::from_utf8(&buf[offset..offset + k_len]).ok()?;
                offset += k_len;

                // Payload 파싱
                let (p_len, vle_len) = decode_vle(&buf[offset..])?;
                offset += vle_len;
                if offset + p_len > buf.len() {
                    return None;
                }
                let payload = &buf[offset..offset + p_len];

                Some((msg_id::QUERY, key_expr, q_id as u32, payload))
            }
            _ => None,
        }
    }
}
