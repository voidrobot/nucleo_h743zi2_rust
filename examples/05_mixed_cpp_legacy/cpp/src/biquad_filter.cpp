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

BiquadFilter::BiquadFilter() {
    b0 = 1.0f;
    b1 = 0.0f;
    b2 = 0.0f;
    a1 = 0.0f;
    a2 = 0.0f;
    w1 = 0.0f;
    w2 = 0.0f;
}

void BiquadFilter::init_lpf(float sample_rate, float cutoff_freq, float q) {
    if (sample_rate <= 0.0f || cutoff_freq <= 0.0f || q <= 0.0f) {
        reset();
        b0 = 1.0f;
        b1 = 0.0f;
        b2 = 0.0f;
        a1 = 0.0f;
        a2 = 0.0f;
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
    float num_b0 = (1.0f - cos_w) * 0.5f;
    float num_b1 = 1.0f - cos_w;
    float num_b2 = (1.0f - cos_w) * 0.5f;
    float num_a0 = 1.0f + alpha;
    float num_a1 = -2.0f * cos_w;
    float num_a2 = 1.0f - alpha;

    // a0 정규화
    float inv_a0 = 1.0f / num_a0;
    b0 = num_b0 * inv_a0;
    b1 = num_b1 * inv_a0;
    b2 = num_b2 * inv_a0;
    a1 = num_a1 * inv_a0;
    a2 = num_a2 * inv_a0;

    reset();
}

float BiquadFilter::process(float in) {
    // Direct Form II Transposed 구조 (수치적 오버플로우 방어)
    float out = b0 * in + w1;
    w1 = b1 * in - a1 * out + w2;
    w2 = b2 * in - a2 * out;
    return out;
}

void BiquadFilter::reset() {
    w1 = 0.0f;
    w2 = 0.0f;
}

// --- C-ABI 브리지 구현 (BiquadCore 타입 직접 바인딩) ---

extern "C" {

void biquad_init_lpf(BiquadCore* filter, float sample_rate, float cutoff_freq, float q) {
    if (!filter) return;
    auto* f = static_cast<BiquadFilter*>(filter);
    f->init_lpf(sample_rate, cutoff_freq, q);
}

float biquad_process(BiquadCore* filter, float input) {
    if (!filter) return input;
    auto* f = static_cast<BiquadFilter*>(filter);
    return f->process(input);
}

void biquad_reset(BiquadCore* filter) {
    if (!filter) return;
    auto* f = static_cast<BiquadFilter*>(filter);
    f->reset();
}

} // extern "C"
