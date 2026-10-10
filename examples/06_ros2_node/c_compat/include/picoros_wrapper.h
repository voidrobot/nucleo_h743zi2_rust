#ifndef _PICOROS_WRAPPER_H_
#define _PICOROS_WRAPPER_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 콜백 함수 타입 정의
typedef void (*rust_sub_cb_t)(const uint8_t *data, size_t len);
typedef size_t (*rust_srv_cb_t)(const uint8_t *req, size_t req_len, uint8_t *resp, size_t max_resp_len);

// 시스템 및 노드 초기화
int picoros_rust_init(const char *locator);
int picoros_rust_node_init(const char *name, uint32_t domain_id);
int picoros_rust_spin_once(void);

// 퍼블리셔 API
void *picoros_rust_pub_create(const char *name, const char *type, const char *hash);
int picoros_rust_pub_send(void *handle, const uint8_t *payload, size_t len);

// 서브스크라이버 API
void *picoros_rust_sub_create(const char *name, const char *type, const char *hash, rust_sub_cb_t cb);

// 서비스 서버 API
void *picoros_rust_srv_create(const char *name, const char *type, const char *hash, rust_srv_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* _PICOROS_WRAPPER_H_ */
