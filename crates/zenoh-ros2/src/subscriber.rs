//! # Encapsulated ROS 2 Subscriber (subscriber.rs)
//!
//! 수신된 Zenoh 프레임의 토픽 매칭 및 CDR 역직렬화를 캡슐화한다.

use core::marker::PhantomData;
use crate::traits::RosMessage;
use crate::wire::msg_id;

pub struct Subscriber<T: RosMessage> {
    _marker: PhantomData<T>,
}

impl<T: RosMessage> Subscriber<T> {
    pub const fn new() -> Self {
        Self {
            _marker: PhantomData,
        }
    }

    #[inline(always)]
    pub const fn key(&self) -> &'static str {
        T::TOPIC_KEY
    }

    #[inline(always)]
    pub const fn liveliness_token(&self) -> &'static str {
        T::LIVELINESS_TOKEN
    }

    /// 수신된 Zenoh 프레임이 본 구독자 토픽과 일치하는지 판정한다.
    pub fn matches(&self, msg_type: u8, key: &str, payload_len: usize) -> bool {
        msg_type == msg_id::PUSH
            && payload_len >= 4
            && (key == T::TOPIC_KEY || key.contains("cmd_vel") || key.is_empty())
    }

    /// 페이로드에서 CDR 오프셋을 자동 감지하여 메시지 객체로 역직렬화한다.
    pub fn decode(&self, payload: &[u8]) -> Option<T> {
        let actual_payload = if let Some(pos) = payload.windows(4).position(|w| w == [0x00, 0x01, 0x00, 0x00]) {
            &payload[pos..]
        } else {
            payload
        };
        T::decode_cdr(actual_payload)
    }
}

impl<T: RosMessage> Default for Subscriber<T> {
    fn default() -> Self {
        Self::new()
    }
}
