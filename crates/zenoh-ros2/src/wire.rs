//! # no_std Zenoh 1.0 초경량 와이어 프로토콜 인코더 및 디코더 (wire.rs)
//!
//! Eclipse Zenoh 1.0 프로토콜 규격에 따라 FRAME(Transport) -> PUSH(Network) -> PUT(Data)
//! 계층 구조를 힙 동적 할당 없이(Zero-Heap Allocation) 고정 버퍼에서 직접 인코딩 및 디코딩한다.

/// Zenoh 프로토콜 메시지 ID
pub mod msg_id {
    pub const FRAME: u8 = 0x05;
    pub const PUSH: u8 = 0x1D;
    pub const PUT: u8 = 0x01;
    pub const REQUEST: u8 = 0x1C;
    pub const QUERY: u8 = 0x1C;
    pub const RESPONSE: u8 = 0x1B;
    pub const REPLY: u8 = 0x1B;
    pub const RESPONSE_FINAL: u8 = 0x1A;
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
    /// ROS 2 RMW Attachment 바이트 버퍼 생성 (33바이트)
    pub fn build_rmw_attachment(
        buf: &mut [u8; 33],
        seq: i64,
        time_ns: i64,
        gid: &[u8; 16],
    ) {
        buf[0..8].copy_from_slice(&seq.to_le_bytes());
        buf[8..16].copy_from_slice(&time_ns.to_le_bytes());
        buf[16] = 16; // rmw_gid_size
        buf[17..33].copy_from_slice(gid);
    }

    /// PUSH / PUT 데이터 발행 프레임 조립 (Transport FRAME + Network PUSH + Data PUT)
    pub fn build_push_put(
        buf: &mut [u8],
        seq: u32,
        key_expr: &str,
        payload: &[u8],
    ) -> usize {
        Self::build_push_put_with_attachment(buf, seq, key_expr, None, payload)
    }

    /// PUSH / PUT 데이터 발행 프레임 조립 (RMW Attachment 확장 지원)
    pub fn build_push_put_with_attachment(
        buf: &mut [u8],
        seq: u32,
        key_expr: &str,
        attachment: Option<&[u8]>,
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

        // 2. Network PUSH Message (0x1D | 0x20 = 0x3D: _Z_MID_N_PUSH | _Z_FLAG_N_PUSH_N)
        if offset < buf.len() {
            buf[offset] = msg_id::PUSH | 0x20;
            offset += 1;
        }
        // WireExpr: Numerical ID = 0 (VLE)
        offset += encode_vle(0, &mut buf[offset..]);
        // Key Expression (VLE Length + UTF-8 Bytes)
        offset += encode_vle(key_expr.len(), &mut buf[offset..]);
        let k_len = key_expr.len();
        if offset + k_len <= buf.len() {
            buf[offset..offset + k_len].copy_from_slice(key_expr.as_bytes());
            offset += k_len;
        }

        // 3. Data PUT Sub-message
        if let Some(att) = attachment {
            // PUT header with extension: _Z_MID_Z_PUT (0x01) | _Z_FLAG_Z_Z (0x80) = 0x81
            if offset < buf.len() {
                buf[offset] = msg_id::PUT | 0x80;
                offset += 1;
            }
            // Extension header for Attachment: _Z_MSG_EXT_ENC_ZBUF (0x40) | 0x03 = 0x43
            if offset < buf.len() {
                buf[offset] = 0x43;
                offset += 1;
            }
            // Attachment bytes (VLE Length + Attachment Bytes)
            offset += encode_vle(att.len(), &mut buf[offset..]);
            let a_len = att.len();
            if offset + a_len <= buf.len() {
                buf[offset..offset + a_len].copy_from_slice(att);
                offset += a_len;
            }
        } else {
            // Data PUT without extension: 0x01
            if offset < buf.len() {
                buf[offset] = msg_id::PUT;
                offset += 1;
            }
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
        key_expr: &str,
        attachment: Option<&[u8]>,
        payload: &[u8],
    ) -> usize {
        let mut offset = 0;

        // FRAME Header + Seq
        if offset < buf.len() {
            buf[offset] = msg_id::FRAME;
            offset += 1;
        }
        offset += encode_vle(seq as usize, &mut buf[offset..]);

        // Network RESPONSE (0x1B) with Suffix Flag (0x20) => 0x3B
        if offset < buf.len() {
            buf[offset] = msg_id::RESPONSE | 0x20;
            offset += 1;
        }
        offset += encode_vle(query_id as usize, &mut buf[offset..]);

        // WireExpr: id = 0, suffix = key_expr
        offset += encode_vle(0, &mut buf[offset..]);
        offset += encode_vle(key_expr.len(), &mut buf[offset..]);
        let k_len = key_expr.len();
        if offset + k_len <= buf.len() {
            buf[offset..offset + k_len].copy_from_slice(key_expr.as_bytes());
            offset += k_len;
        }

        // Reply Body (_Z_MID_Z_REPLY = 0x04)
        if offset < buf.len() {
            buf[offset] = 0x04;
            offset += 1;
        }

        // Data PUT
        if let Some(att) = attachment {
            if offset < buf.len() {
                buf[offset] = msg_id::PUT | 0x80;
                offset += 1;
            }
            if offset < buf.len() {
                buf[offset] = 0x43;
                offset += 1;
            }
            offset += encode_vle(att.len(), &mut buf[offset..]);
            let a_len = att.len();
            if offset + a_len <= buf.len() {
                buf[offset..offset + a_len].copy_from_slice(att);
                offset += a_len;
            }
        } else {
            if offset < buf.len() {
                buf[offset] = msg_id::PUT;
                offset += 1;
            }
        }

        offset += encode_vle(payload.len(), &mut buf[offset..]);
        let p_len = payload.len();
        if offset + p_len <= buf.len() {
            buf[offset..offset + p_len].copy_from_slice(payload);
            offset += p_len;
        }

        // Network RESPONSE_FINAL (0x1A)
        if offset < buf.len() {
            buf[offset] = msg_id::RESPONSE_FINAL;
            offset += 1;
        }
        offset += encode_vle(query_id as usize, &mut buf[offset..]);

        offset
    }

    /// Liveliness Token 선언 (Transport FRAME + Network DECLARE + Token)
    pub fn build_declare_token(
        buf: &mut [u8],
        seq: u32,
        token_id: u32,
        key_expr: &str,
    ) -> usize {
        let mut offset = 0;
        buf[offset] = msg_id::FRAME;
        offset += 1;
        offset += encode_vle(seq as usize, &mut buf[offset..]);

        // Network DECLARE (0x1E)
        buf[offset] = 0x1E;
        offset += 1;

        // Decl Token: 0x26 (_Z_DECL_TOKEN_MID(6) | _Z_DECL_SUBSCRIBER_FLAG_N(0x20))
        buf[offset] = 0x26;
        offset += 1;
        offset += encode_vle(token_id as usize, &mut buf[offset..]);

        // WireExpr: id = 0, suffix = key_expr
        offset += encode_vle(0, &mut buf[offset..]);
        offset += encode_vle(key_expr.len(), &mut buf[offset..]);
        let k_len = key_expr.len();
        buf[offset..offset + k_len].copy_from_slice(key_expr.as_bytes());
        offset += k_len;

        offset
    }

    /// Queryable 선언 (Transport FRAME + Network DECLARE + Queryable with complete=true)
    pub fn build_declare_queryable(
        buf: &mut [u8],
        seq: u32,
        q_id: u32,
        key_expr: &str,
    ) -> usize {
        let mut offset = 0;
        buf[offset] = msg_id::FRAME;
        offset += 1;
        offset += encode_vle(seq as usize, &mut buf[offset..]);

        // Network DECLARE (0x1E)
        buf[offset] = 0x1E;
        offset += 1;

        // Decl Queryable: 0xA4 (_Z_FLAG_Z_Z(0x80) | _Z_DECL_SUBSCRIBER_FLAG_N(0x20) | _Z_DECL_QUERYABLE_MID(0x04))
        buf[offset] = 0xA4;
        offset += 1;
        offset += encode_vle(q_id as usize, &mut buf[offset..]);

        // WireExpr: id = 0, suffix = key_expr
        offset += encode_vle(0, &mut buf[offset..]);
        offset += encode_vle(key_expr.len(), &mut buf[offset..]);
        let k_len = key_expr.len();
        buf[offset..offset + k_len].copy_from_slice(key_expr.as_bytes());
        offset += k_len;

        // Ext: Complete = true (0x21, value 1)
        buf[offset] = 0x21;
        offset += 1;
        offset += encode_vle(1, &mut buf[offset..]);

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
        if (msg_type & 0x1F) == msg_id::FRAME {
            let (_seq, vle_len) = decode_vle(&buf[offset..])?;
            offset += vle_len;
            if offset >= buf.len() {
                return None;
            }
            msg_type = buf[offset];
            offset += 1;
        }

        match msg_type & 0x1F {
            msg_id::PUSH => {
                let (_id, vle_len) = decode_vle(&buf[offset..])?;
                offset += vle_len;

                let key_expr = if (msg_type & 0x20) != 0 {
                    let (k_len, vle_len) = decode_vle(&buf[offset..])?;
                    offset += vle_len;
                    if offset + k_len > buf.len() {
                        return None;
                    }
                    let s = core::str::from_utf8(&buf[offset..offset + k_len]).ok()?;
                    offset += k_len;
                    s
                } else {
                    ""
                };

                if offset >= buf.len() || (buf[offset] & 0x1F) != msg_id::PUT {
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
            msg_id::REQUEST => {
                let (q_id, vle_len) = decode_vle(&buf[offset..])?;
                offset += vle_len;

                let (_id, vle_len) = decode_vle(&buf[offset..])?;
                offset += vle_len;

                let key_expr = if (msg_type & 0x20) != 0 {
                    let (k_len, vle_len) = decode_vle(&buf[offset..])?;
                    offset += vle_len;
                    if offset + k_len > buf.len() {
                        return None;
                    }
                    let s = core::str::from_utf8(&buf[offset..offset + k_len]).ok()?;
                    offset += k_len;
                    s
                } else {
                    ""
                };

                let payload = if offset < buf.len() {
                    &buf[offset..]
                } else {
                    &[]
                };

                Some((msg_id::REQUEST, key_expr, q_id as u32, payload))
            }
            _ => None,
        }
    }

    /// Zenoh Transport 계층 세션 개설 메시지 (InitSyn) 생성 (22바이트)
    pub fn build_init_syn(buf: &mut [u8], zid: &[u8; 16]) -> usize {
        buf[0] = 0x01 | 0x40;
        buf[1] = 0x09;
        buf[2] = 0xF2;
        buf[3..19].copy_from_slice(zid);
        buf[19] = 0x0A;
        buf[20] = 0x00;
        buf[21] = 0x08;
        22
    }

    /// Zenoh Transport 계층 InitAck 수신 및 cookie 슬라이스 추출
    pub fn parse_init_ack(buf: &[u8]) -> Option<&[u8]> {
        if buf.len() < 4 {
            return None;
        }
        let header = buf[0];
        if (header & 0x1F) != 0x01 || (header & 0x20) == 0 {
            return None;
        }
        if buf[1] != 0x09 {
            return None;
        }
        let cbyte = buf[2];
        let zidlen = (((cbyte >> 4) & 0x0F) + 1) as usize;
        let mut offset = 3 + zidlen;

        if (header & 0x40) != 0 {
            offset += 3;
        }

        if offset >= buf.len() {
            return None;
        }

        let (cookie_len, vle_len) = decode_vle(&buf[offset..])?;
        offset += vle_len;
        if offset + cookie_len > buf.len() {
            return None;
        }
        Some(&buf[offset..offset + cookie_len])
    }

    /// Zenoh Transport 계층 세션 확립 메시지 (OpenSyn) 생성
    pub fn build_open_syn(buf: &mut [u8], cookie: &[u8]) -> usize {
        buf[0] = 0x02 | 0x40;
        let mut offset = 1;

        offset += encode_vle(10, &mut buf[offset..]);
        offset += encode_vle(0, &mut buf[offset..]);

        offset += encode_vle(cookie.len(), &mut buf[offset..]);
        buf[offset..offset + cookie.len()].copy_from_slice(cookie);
        offset += cookie.len();

        offset
    }

    /// Zenoh Transport 계층 OpenAck 수신 검증
    pub fn is_open_ack(buf: &[u8]) -> bool {
        if buf.is_empty() {
            return false;
        }
        let header = buf[0];
        (header & 0x1F) == 0x02 && (header & 0x20) != 0
    }
}
