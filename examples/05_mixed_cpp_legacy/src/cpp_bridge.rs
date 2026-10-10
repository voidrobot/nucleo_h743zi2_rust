//! C++ 레거시 DSP Biquad 필터에 대한 무오버헤드 C-ABI FFI 및 Safe RAII 래퍼 모듈.

use core::ffi::c_void;

extern "C" {
    fn biquad_get_instance_size() -> u32;
    fn biquad_init_lpf(filter_mem: *mut c_void, sample_rate: f32, cutoff_freq: f32, q: f32);
    fn biquad_process(filter_mem: *mut c_void, input: f32) -> f32;
    fn biquad_reset(filter_mem: *mut c_void);
}

/// C++ `BiquadFilter` 클래스 객체를 동적 힙 할당 없이
/// 스택/정적 영역에 직접 품는 안전한 RAII Newtype 래퍼.
#[repr(C, align(4))]
pub struct SafeBiquadFilter {
    // C++ sizeof(BiquadFilter) = 28 bytes를 안전하게 수용하는 32바이트 인라인 버퍼
    storage: [u8; 32],
}

impl SafeBiquadFilter {
    /// 2차 IIR 저주파 통과 필터(LPF) 인스턴스를 초기화하여 생성한다.
    ///
    /// * `sample_rate`: 샘플링 주파수 (Hz)
    /// * `cutoff_freq`: 차단 주파수 (Hz)
    /// * `q`: 품질 팩터 (0.7071f = Butterworth 특성)
    pub fn new_lpf(sample_rate: f32, cutoff_freq: f32, q: f32) -> Self {
        let mut filter = Self { storage: [0u8; 32] };
        unsafe {
            let actual_size = biquad_get_instance_size() as usize;
            debug_assert!(
                actual_size <= core::mem::size_of_val(&filter.storage),
                "C++ BiquadFilter 크기가 Rust 인라인 저장소(32B)를 초과함: {}",
                actual_size
            );

            biquad_init_lpf(
                filter.storage.as_mut_ptr() as *mut c_void,
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
        unsafe { biquad_process(self.storage.as_mut_ptr() as *mut c_void, input) }
    }

    /// 내부 Direct Form II 딜레이 버퍼 상태를 0으로 리셋한다.
    pub fn reset(&mut self) {
        unsafe { biquad_reset(self.storage.as_mut_ptr() as *mut c_void) }
    }
}

impl Drop for SafeBiquadFilter {
    fn drop(&mut self) {
        // 인라인 저장소이므로 동적 힙 메모리 해제는 불필요하나,
        // 상태를 안전하게 정리(Zeroing)한다.
        self.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_biquad_instance_size_fits_storage() {
        let size = unsafe { biquad_get_instance_size() };
        assert!(size <= 32, "C++ 인스턴스 크기({})가 32바이트 이하이어야 함", size);
        assert_eq!(size, 28, "C++ float 7개 = 28바이트이어야 함");
    }

    #[test]
    fn test_biquad_dc_pass_unity_gain() {
        // DC 신호(10.0) 입력 시 LPF는 1.0의 이득을 가지므로 출력도 10.0으로 수렴해야 함
        let mut filter = SafeBiquadFilter::new_lpf(100.0, 5.0, 0.7071);
        let mut out = 0.0;
        for _ in 0..100 {
            out = filter.process(10.0);
        }
        assert!((out - 10.0).abs() < 0.01, "DC 입력 수렴 오차 초과: out={}", out);
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
                if in_val.abs() > max_in { max_in = in_val.abs(); }
                if out_val.abs() > max_out { max_out = out_val.abs(); }
            }
        }

        let attenuation_ratio = max_out / max_in;
        // 40Hz는 5Hz 차단 주파수보다 훨씬 높으므로 최소 80% 이상 감쇠(비율 < 0.2)되어야 함
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
        // 리셋 직후 0.0 입력 시 출력은 0.0이어야 함
        let out = filter.process(0.0);
        assert_eq!(out, 0.0);
    }
}
