#include "biquad_filter.hpp"

// 자립형(Freestanding) 환경을 위한 고정밀 삼각함수 다항식 근사 (libm 의존성 0%)
static constexpr float PI = 3.14159265358979323846f;
static constexpr float TWO_PI = 6.28318530717958647692f;

static inline float local_sinf(float x) {
    // 0 ~ 2*PI 범위로 정규화
    while (x < 0.0f) x += TWO_PI;
    while (x >= TWO_PI) x -= TWO_PI;

    // 0 ~ PI 범위 대칭
    float sign = 1.0f;
    if (x > PI) {
        x -= PI;
        sign = -1.0f;
    }
    if (x > PI * 0.5f) {
        x = PI - x;
    }

    // 7차 테일러/미니맥스 급수 근사: x - x^3/6 + x^5/120 - x^7/5040
    float x2 = x * x;
    float x3 = x * x2;
    float x5 = x3 * x2;
    float x7 = x5 * x2;

    float s = x - (x3 * (1.0f / 6.0f)) + (x5 * (1.0f / 120.0f)) - (x7 * (1.0f / 5040.0f));
    return sign * s;
}

static inline float local_cosf(float x) {
    return local_sinf(x + (PI * 0.5f));
}

BiquadFilter::BiquadFilter()
    : b0_(1.0f), b1_(0.0f), b2_(0.0f), a1_(0.0f), a2_(0.0f), w1_(0.0f), w2_(0.0f) {
}

void BiquadFilter::init_lpf(float sample_rate, float cutoff_freq, float q) {
    if (sample_rate <= 0.0f || cutoff_freq <= 0.0f || q <= 0.0f) {
        reset();
        b0_ = 1.0f;
        b1_ = 0.0f;
        b2_ = 0.0f;
        a1_ = 0.0f;
        a2_ = 0.0f;
        return;
    }

    // 나이퀴스트 주파수(Fs / 2) 초과 방지
    if (cutoff_freq > sample_rate * 0.49f) {
        cutoff_freq = sample_rate * 0.49f;
    }

    float omega = TWO_PI * (cutoff_freq / sample_rate);
    float sin_w = local_sinf(omega);
    float cos_w = local_cosf(omega);
    float alpha = sin_w / (2.0f * q);

    // Audio EQ Cookbook LPF 표준 공식
    float b0 = (1.0f - cos_w) * 0.5f;
    float b1 = 1.0f - cos_w;
    float b2 = (1.0f - cos_w) * 0.5f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w;
    float a2 = 1.0f - alpha;

    // a0 정규화
    float inv_a0 = 1.0f / a0;
    b0_ = b0 * inv_a0;
    b1_ = b1 * inv_a0;
    b2_ = b2 * inv_a0;
    a1_ = a1 * inv_a0;
    a2_ = a2 * inv_a0;

    reset();
}

float BiquadFilter::process(float in) {
    // Direct Form II Transposed 구조 (수치적 오버플로우 방어)
    float out = b0_ * in + w1_;
    w1_ = b1_ * in - a1_ * out + w2_;
    w2_ = b2_ * in - a2_ * out;
    return out;
}

void BiquadFilter::reset() {
    w1_ = 0.0f;
    w2_ = 0.0f;
}

// --- C-ABI 브리지 구현 (Rust FFI 연동용) ---

extern "C" {

uint32_t biquad_get_instance_size(void) {
    return static_cast<uint32_t>(sizeof(BiquadFilter));
}

void biquad_init_lpf(void* filter_mem, float sample_rate, float cutoff_freq, float q) {
    if (!filter_mem) return;
    // Placement New와 동일하게 기존 메모리 공간에 객체 초기화
    auto* filter = reinterpret_cast<BiquadFilter*>(filter_mem);
    filter->init_lpf(sample_rate, cutoff_freq, q);
}

float biquad_process(void* filter_mem, float input) {
    if (!filter_mem) return input;
    auto* filter = reinterpret_cast<BiquadFilter*>(filter_mem);
    return filter->process(input);
}

void biquad_reset(void* filter_mem) {
    if (!filter_mem) return;
    auto* filter = reinterpret_cast<BiquadFilter*>(filter_mem);
    filter->reset();
}

} // extern "C"
