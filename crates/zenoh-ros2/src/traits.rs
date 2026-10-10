//! # ROS 2 Core Abstraction Traits (traits.rs)
//!
//! 토픽 메시지(`RosMessage`) 및 요청/응답 서비스(`RosService`) 규격을 정의한다.

/// ROS 2 토픽 메시지 트레이트
pub trait RosMessage: Sized {
    /// ROS 2 토픽 키 표현식 (Key Expression)
    const TOPIC_KEY: &'static str;

    /// ROS 2 Jazzy Liveliness Token 문자열
    const LIVELINESS_TOKEN: &'static str;

    /// 메시지 내용을 CDR 규격으로 직렬화하고 인코딩된 바이트 수를 반환한다.
    fn encode_cdr(&self, buf: &mut [u8]) -> usize;

    /// 수신된 CDR 바이트 슬라이스로부터 메시지를 역직렬화한다.
    fn decode_cdr(_buf: &[u8]) -> Option<Self> {
        None
    }
}

/// ROS 2 서비스 트레이트
pub trait RosService {
    /// 서비스 요청 타입
    type Request: Sized;
    /// 서비스 응답 타입
    type Response: Sized;

    /// ROS 2 서비스 키 표현식 (Key Expression)
    const SERVICE_KEY: &'static str;

    /// ROS 2 Jazzy Liveliness Token 문자열
    const LIVELINESS_TOKEN: &'static str;

    /// 수신된 페이로드에서 요청(Request)을 역직렬화한다.
    fn decode_request(buf: &[u8]) -> Option<Self::Request>;

    /// 응답(Response)을 CDR 규격으로 직렬화하고 인코딩된 바이트 수를 반환한다.
    fn encode_response(res: &Self::Response, buf: &mut [u8]) -> usize;
}
