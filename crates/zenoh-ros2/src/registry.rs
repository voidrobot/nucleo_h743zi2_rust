//! # ROS 2 Discovery Registry (registry.rs)
//!
//! 개방-폐쇄 원칙(OCP)에 따라 등록된 모든 엔드포인트의 Liveliness Token을
//! 단일 진실 공급원(SSOT)으로 통합 관리하고, 세션 확립 및 1Hz 갱신 시
//! 하드코딩 없는 일괄 순회 선언을 제공한다.

pub struct DiscoveryRegistry<'a> {
    tokens: &'a [&'a str],
}

impl<'a> DiscoveryRegistry<'a> {
    /// 토큰 슬라이스로부터 레지스트리를 생성한다.
    pub const fn new(tokens: &'a [&'a str]) -> Self {
        Self { tokens }
    }

    /// 등록된 전체 Liveliness Token 슬라이스를 반환한다.
    #[inline(always)]
    pub fn tokens(&self) -> &'a [&'a str] {
        self.tokens
    }

    /// 등록된 토큰 개수를 반환한다.
    #[inline(always)]
    pub fn len(&self) -> usize {
        self.tokens.len()
    }

    /// 등록된 토큰이 비어 있는지 여부를 반환한다.
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.tokens.is_empty()
    }
}
