#include "zenoh-pico/system/common/platform.h"
#include <string.h>

// Rust FFI 콜백 함수 선언
extern uint64_t rust_embassy_time_now_ms(void);
extern void *rust_embassy_alloc(size_t size);
extern void rust_embassy_free(void *ptr);

// 1. Random 인터페이스
static uint32_t s_rand_seed = 0x12345678;

static uint32_t simple_rand(void) {
    s_rand_seed = s_rand_seed * 1103515245 + 12345;
    return (uint32_t)(s_rand_seed >> 16);
}

uint8_t z_random_u8(void) {
    return (uint8_t)simple_rand();
}

uint16_t z_random_u16(void) {
    return (uint16_t)simple_rand();
}

uint32_t z_random_u32(void) {
    return simple_rand();
}

uint64_t z_random_u64(void) {
    return ((uint64_t)simple_rand() << 32) | simple_rand();
}

void z_random_fill(void *buf, size_t len) {
    uint8_t *p = (uint8_t *)buf;
    for (size_t i = 0; i < len; i++) {
        p[i] = z_random_u8();
    }
}

// 2. Memory 인터페이스 (정적 8KB 풀)
#define ZENOH_HEAP_SIZE (16 * 1024)
static uint8_t s_zenoh_heap[ZENOH_HEAP_SIZE] __attribute__((aligned(8)));
static size_t s_heap_offset = 0;

void *z_malloc(size_t size) {
    size = (size + 7) & ~7; // 8-byte align
    if (s_heap_offset + size <= ZENOH_HEAP_SIZE) {
        void *p = &s_zenoh_heap[s_heap_offset];
        s_heap_offset += size;
        return p;
    }
    return NULL;
}

void *z_realloc(void *ptr, size_t size) {
    if (!ptr) return z_malloc(size);
    // 간단한 정적 풀에서는 새 블록 할당 후 복사
    void *new_p = z_malloc(size);
    if (new_p) {
        memcpy(new_p, ptr, size); // 보수적 복사
    }
    return new_p;
}

void z_free(void *ptr) {
    (void)ptr;
    // 베어메탈 풀 단순화: 런타임 누수 최소화 설계
}

// 3. Clock 인터페이스
z_clock_t z_clock_now(void) {
    uint64_t ms = rust_embassy_time_now_ms();
    z_clock_t c;
    c.tv_sec = (uint32_t)(ms / 1000);
    c.tv_nsec = (uint32_t)((ms % 1000) * 1000000);
    return c;
}

unsigned long z_clock_elapsed_ms(z_clock_t *start) {
    if (!start) return 0;
    z_clock_t now = z_clock_now();
    uint64_t now_ms = (uint64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
    uint64_t start_ms = (uint64_t)start->tv_sec * 1000 + start->tv_nsec / 1000000;
    if (now_ms >= start_ms) {
        return (unsigned long)(now_ms - start_ms);
    }
    return 0;
}

unsigned long zp_clock_elapsed_ms_since(z_clock_t *instant, z_clock_t *epoch) {
    if (!instant || !epoch) return 0;
    long elapsed = (1000 * (instant->tv_sec - epoch->tv_sec) + (instant->tv_nsec - epoch->tv_nsec) / 1000000);
    return elapsed > 0 ? (unsigned long)elapsed : 0;
}

unsigned long zp_clock_elapsed_us_since(z_clock_t *instant, z_clock_t *epoch) {
    if (!instant || !epoch) return 0;
    long elapsed = (1000000 * (instant->tv_sec - epoch->tv_sec) + (instant->tv_nsec - epoch->tv_nsec) / 1000);
    return elapsed > 0 ? (unsigned long)elapsed : 0;
}

void z_clock_advance_ms(z_clock_t *clock, unsigned long duration) {
    if (!clock) return;
    clock->tv_sec += duration / 1000;
    clock->tv_nsec += (duration % 1000) * 1000000;
    if (clock->tv_nsec >= 1000000000) {
        clock->tv_sec += 1;
        clock->tv_nsec -= 1000000000;
    }
}

void __assert_func(const char *file, int line, const char *func, const char *failedexpr) {
    (void)file; (void)line; (void)func; (void)failedexpr;
    while (1) {
        __asm__("bkpt 0");
    }
}

z_result_t z_sleep_ms(size_t ms) {
    (void)ms;
    return _Z_RES_OK;
}

z_result_t z_sleep_us(size_t us) {
    (void)us;
    return _Z_RES_OK;
}

// Newlib 힙 메모리 시스템 콜 스텁 (정적 풀 z_malloc 사용으로 미사용)
void *_sbrk(ptrdiff_t incr) {
    (void)incr;
    return (void *)-1;
}
