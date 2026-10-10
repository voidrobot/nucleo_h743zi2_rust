//! C++ 레거시 DSP Biquad 필터에 대한 무오버헤드 C-ABI FFI 및 Safe RAII 래퍼 모듈.

/// C++ `BiquadCore` 구조체와 1:1로 완벽히 매핑되는 C-ABI 공용 구조체 (28 바이트)
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BiquadCore {
    pub b0: f32,
    pub b1: f32,
    pub b2: f32,
    pub a1: f32,
    pub a2: f32,
    pub w1: f32,
    pub w2: f32,
}

impl Default for BiquadCore {
    fn default() -> Self {
        Self {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            w1: 0.0,
            w2: 0.0,
        }
    }
}

// 컴파일 타임 무비용 크기 및 정렬 정적 단언 (Zero-Cost Static Assertions)
const _: () = assert!(core::mem::size_of::<BiquadCore>() == 28);
const _: () = assert!(core::mem::align_of::<BiquadCore>() == 4);

extern "C" {
    fn biquad_init_lpf(filter: *mut BiquadCore, sample_rate: f32, cutoff_freq: f32, q: f32);
    fn biquad_process(filter: *mut BiquadCore, input: f32) -> f32;
    fn biquad_reset(filter: *mut BiquadCore);
}

/// C++ `BiquadFilter` 클래스 객체를 동적 힙 할당 없이
/// 스택/정적 영역에 직접 소유하는 안전한 RAII Newtype 래퍼.
#[repr(transparent)]
pub struct SafeBiquadFilter {
    core: BiquadCore,
}

impl SafeBiquadFilter {
    /// 2차 IIR 저주파 통과 필터(LPF) 인스턴스를 초기화하여 생성한다.
    ///
    /// * `sample_rate`: 샘플링 주파수 (Hz)
    /// * `cutoff_freq`: 차단 주파수 (Hz)
    /// * `q`: 품질 팩터 (0.7071f = Butterworth 특성)
    pub fn new_lpf(sample_rate: f32, cutoff_freq: f32, q: f32) -> Self {
        let mut filter = Self {
            core: BiquadCore::default(),
        };
        unsafe {
            biquad_init_lpf(
                &mut filter.core as *mut BiquadCore,
                sample_rate,
                cutoff_freq,
                q,
            );
        }
        filter
    }

    /// 단일 샘플을 필터링 처리하여 평활화된 결과값을 반환한다.
    #[inline]
    pub fn process(&mut self, input: f32) -> f32 {
        unsafe { biquad_process(&mut self.core as *mut BiquadCore, input) }
    }

    /// 내부 Direct Form II 딜레이 버퍼 상태를 0으로 리셋한다.
    pub fn reset(&mut self) {
        unsafe { biquad_reset(&mut self.core as *mut BiquadCore) }
    }

    /// 필터의 내부 C-ABI 코어 상태 참조자를 반환한다.
    #[inline]
    pub fn core(&self) -> &BiquadCore {
        &self.core
    }
}

impl Drop for SafeBiquadFilter {
    fn drop(&mut self) {
        self.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_biquad_core_size_and_alignment() {
        assert_eq!(core::mem::size_of::<BiquadCore>(), 28);
        assert_eq!(core::mem::align_of::<BiquadCore>(), 4);
        assert_eq!(core::mem::size_of::<SafeBiquadFilter>(), 28);
    }

    #[test]
    fn test_biquad_dc_pass_unity_gain() {
        // DC 신호(10.0) 입력 시 LPF는 1.0의 이득을 가지므로 출력도 10.0으로 수렴해야 함
        let mut filter = SafeBiquadFilter::new_lpf(100.0, 5.0, 0.7071);
        let mut out = 0.0;
        for _ in 0..100 {
            out = filter.process(10.0);
        }
        assert!(
            (out - 10.0).abs() < 0.01,
            "DC 입력 수렴 오차 초과: out={}",
            out
        );
    }

    #[test]
    fn test_biquad_high_frequency_attenuation() {
        // Fs = 100Hz, Fc = 5Hz일 때 40Hz 고주파 노이즈 신호는 크게 감쇠되어야 함
        let mut filter = SafeBiquadFilter::new_lpf(100.0, 5.0, 0.7071);
        let pi = core::f32::consts::PI;

        let mut max_in = 0.0f32;
        let mut max_out = 0.0f32;

        for i in 0..200 {
            let t = i as f32 / 100.0;
            // 40Hz 정현파 (진폭 10.0)
            let in_val = 10.0 * libm::sinf(2.0 * pi * 40.0 * t);
            let out_val = filter.process(in_val);

            // 초기 과도 응답 이후 100샘플 이후 피크 측정
            if i > 100 {
                if in_val.abs() > max_in {
                    max_in = in_val.abs();
                }
                if out_val.abs() > max_out {
                    max_out = out_val.abs();
                }
            }
        }

        let attenuation_ratio = max_out / max_in;
        // 40Hz는 5Hz 차단 주파수보다 훨씬 높으므로 최소 85% 이상 감쇠(비율 < 0.15)되어야 함
        assert!(
            attenuation_ratio < 0.15,
            "고주파 감쇠 실패: ratio = {}",
            attenuation_ratio
        );
    }

    #[test]
    fn test_biquad_reset_clears_state() {
        let mut filter = SafeBiquadFilter::new_lpf(100.0, 5.0, 0.7071);
        for _ in 0..20 {
            filter.process(50.0);
        }
        filter.reset();
        assert_eq!(filter.core().w1, 0.0);
        assert_eq!(filter.core().w2, 0.0);

        let out = filter.process(0.0);
        assert_eq!(out, 0.0);
    }
}
