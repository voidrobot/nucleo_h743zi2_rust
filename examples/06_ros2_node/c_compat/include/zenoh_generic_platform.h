#ifndef _ZENOH_GENERIC_PLATFORM_H_
#define _ZENOH_GENERIC_PLATFORM_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 소켓 핸들 타입
typedef int _z_sys_net_socket_t;

// 엔드포인트 타입
typedef struct {
    char address[64];
    uint16_t port;
} _z_sys_net_endpoint_t;

// 시간 및 시계 타입 (Rust embassy-time과 연결)
typedef struct {
    uint32_t tv_sec;
    uint32_t tv_nsec;
} z_clock_t;

typedef struct {
    uint32_t tv_sec;
    uint32_t tv_usec;
} z_time_t;

#ifdef __cplusplus
}
#endif

#endif /* _ZENOH_GENERIC_PLATFORM_H_ */
