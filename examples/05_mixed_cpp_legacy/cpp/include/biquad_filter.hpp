#pragma once

#include <stdint.h>

#ifdef __cplusplus

// C++ 2차 IIR Biquad 저주파 통과 필터 (Direct Form II Transposed)
class BiquadFilter {
public:
    BiquadFilter();

    // 2차 저주파 통과 필터(LPF) 계수 초기화
    // sample_rate: 샘플링 주파수 (Hz)
    // cutoff_freq: 차단 주파수 (Hz)
    // q: 품질 팩터 (일반적으로 0.7071f = Butterworth)
    void init_lpf(float sample_rate, float cutoff_freq, float q);

    // 단일 샘플 실시간 필터링 연산
    float process(float in);

    // 딜레이 버퍼 상태 초기화
    void reset();

private:
    float b0_, b1_, b2_; // 정규화된 분자 계수 (피드포워드)
    float a1_, a2_;     // 정규화된 분모 계수 (피드백)
    float w1_, w2_;     // Direct Form II Transposed 상태 변수
};

#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

// Rust와의 C-ABI 연동 인터페이스 (Zero-Allocation 정적/스택 메모리 방식)
uint32_t biquad_get_instance_size(void);
void biquad_init_lpf(void* filter_mem, float sample_rate, float cutoff_freq, float q);
float biquad_process(void* filter_mem, float input);
void biquad_reset(void* filter_mem);

#ifdef __cplusplus
}
#endif
