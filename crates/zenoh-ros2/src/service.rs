//! # Encapsulated ROS 2 Service Server (service.rs)
//!
//! Queryable 선언, Liveliness Token 제공, 요청 CDR 역직렬화,
//! 응답 CDR 직렬화 및 Zenoh Reply 와이어 프레임 빌드를
//! 단일 객체 내부에서 완결하는 서비스 엔드포인트 엔티티이다.

use core::marker::PhantomData;
use crate::traits::RosService;
use crate::wire::{msg_id, ZenohWire};

pub struct ServiceServer<S: RosService> {
    pub queryable_id: u32,
    pub gid: [u8; 16],
    _marker: PhantomData<S>,
}

impl<S: RosService> ServiceServer<S> {
    pub const fn new(queryable_id: u32, gid: [u8; 16]) -> Self {
        Self {
            queryable_id,
            gid,
            _marker: PhantomData,
        }
    }

    #[inline(always)]
    pub const fn key(&self) -> &'static str {
        S::SERVICE_KEY
    }

    #[inline(always)]
    pub const fn liveliness_token(&self) -> &'static str {
        S::LIVELINESS_TOKEN
    }

    /// 세션 개설 시 Queryable 선언 프레임을 빌드한다.
    pub fn declare_queryable(&self, frame_seq: u32, buf: &mut [u8]) -> usize {
        ZenohWire::build_declare_queryable(buf, frame_seq, self.queryable_id, S::SERVICE_KEY)
    }

    /// 수신된 Zenoh 프레임이 본 서비스에 대한 QUERY/REQUEST인지 판정한다.
    pub fn matches(&self, msg_type: u8, key: &str) -> bool {
        (msg_type == msg_id::QUERY || msg_type == msg_id::REQUEST)
            && (key.contains("set_led") || key == S::SERVICE_KEY || key.is_empty())
    }

    /// 수신 페이로드에서 Request를 역직렬화한다.
    pub fn decode_request(&self, payload: &[u8]) -> Option<S::Request> {
        let actual_payload = if let Some(pos) = payload.windows(4).position(|w| w == [0x00, 0x01, 0x00, 0x00]) {
            &payload[pos..]
        } else {
            payload
        };
        S::decode_request(actual_payload)
    }

    /// 응답(Response) 객체를 CDR 직렬화하고 Zenoh Reply 와이어 프레임을 완성한다.
    pub fn build_reply(
        &self,
        frame_seq: u32,
        q_id: u32,
        res: &S::Response,
        frame_buf: &mut [u8],
        att_buf: &mut [u8; 33],
        cdr_buf: &mut [u8],
    ) -> usize {
        let res_len = S::encode_response(res, cdr_buf);
        ZenohWire::build_rmw_attachment(att_buf, 1, 0, &self.gid);
        ZenohWire::build_reply(
            frame_buf,
            frame_seq,
            q_id,
            S::SERVICE_KEY,
            Some(att_buf),
            &cdr_buf[..res_len],
        )
    }
}
