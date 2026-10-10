#pragma once

#include <stdint.h>

// Rust와 C-ABI로 1:1 매핑되는 2차 IIR Biquad 필터 코어 구조체 (28 바이트)
typedef struct {
    float b0, b1, b2; // 정규화된 분자 계수 (피드포워드)
    float a1, a2;     // 정규화된 분모 계수 (피드백)
    float w1, w2;     // Direct Form II Transposed 지연 소자 상태 변수
} BiquadCore;

#ifdef __cplusplus

// C++ 2차 IIR Biquad 저주파 통과 필터 (Direct Form II Transposed)
// BiquadCore를 상속하되 가상 함수가 없으므로 vtable 없는 완벽한 Standard Layout(28B) 유지
class BiquadFilter : public BiquadCore {
public:
    BiquadFilter();

    // 2차 저주파 통과 필터(LPF) 계수 초기화
    // sample_rate: 샘플링 주파수 (Hz)
    // cutoff_freq: 차단 주파수 (Hz)
    // q: 품질 팩터 (0.7071f = Butterworth 특성)
    void init_lpf(float sample_rate, float cutoff_freq, float q);

    // 단일 샘플 실시간 필터링 연산
    float process(float in);

    // 딜레이 버퍼 상태 초기화
    void reset();
};

#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

// Rust와의 C-ABI 연동 인터페이스 (BiquadCore 구조체 직접 바인딩)
void biquad_init_lpf(BiquadCore* filter, float sample_rate, float cutoff_freq, float q);
float biquad_process(BiquadCore* filter, float input);
void biquad_reset(BiquadCore* filter);

#ifdef __cplusplus
}
#endif
