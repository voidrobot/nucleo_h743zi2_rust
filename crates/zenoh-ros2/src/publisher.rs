//! # Encapsulated ROS 2 Publisher (publisher.rs)
//!
//! 시퀀스 카운터(`seq: i64`)와 고유 GID(`[u8; 16]`)를 완전히 캡슐화하여,
//! 외부에서의 임의 조작 및 토픽 간 시퀀스 번호 누수(Sequence collision)를
//! 컴파일 타임에 원천 차단하는 객체이다.

use core::marker::PhantomData;
use crate::traits::RosMessage;
use crate::wire::ZenohWire;

pub struct Publisher<T: RosMessage> {
    gid: [u8; 16],
    seq: i64,
    _marker: PhantomData<T>,
}

impl<T: RosMessage> Publisher<T> {
    /// 지정된 GID로 독립적인 시퀀스 카운터(`seq=0`)를 갖는 퍼블리셔를 생성한다.
    pub const fn new(gid: [u8; 16]) -> Self {
        Self {
            gid,
            seq: 0,
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

    /// 현재 시퀀스 번호 조회 (디버그/감사용)
    #[inline(always)]
    pub fn current_seq(&self) -> i64 {
        self.seq
    }

    /// 메시지를 CDR 직렬화하고, 단조 증가하는 독립 RMW 시퀀스 번호와 함께
    /// 완전한 Zenoh PUSH/PUT 와이어 프레임을 빌드한다.
    ///
    /// - `time_ns`: 발행 시점 타임스탬프 (나노초)
    /// - `frame_seq`: UDP 세션 레벨 전송 시퀀스 번호
    /// - `frame_buf`: 와이어 프레임이 작성될 목적지 버퍼
    /// - `att_buf`: 33바이트 RMW 어태치먼트 임시 버퍼
    /// - `cdr_buf`: CDR 페이로드 임시 버퍼
    ///
    /// 반환값: 빌드된 전체 Zenoh 프레임 바이트 길이
    pub fn build_frame(
        &mut self,
        msg: &T,
        time_ns: i64,
        frame_seq: u32,
        frame_buf: &mut [u8],
        att_buf: &mut [u8; 33],
        cdr_buf: &mut [u8],
    ) -> usize {
        self.seq = self.seq.wrapping_add(1);

        let payload_len = msg.encode_cdr(cdr_buf);
        ZenohWire::build_rmw_attachment(att_buf, self.seq, time_ns, &self.gid);

        ZenohWire::build_push_put_with_attachment(
            frame_buf,
            frame_seq,
            T::TOPIC_KEY,
            Some(att_buf),
            &cdr_buf[..payload_len],
        )
    }
}
