#include "zenoh-pico/link/transport/udp_unicast.h"
#include <string.h>

// Rust FFI 콜백 함수 선언 (embassy-net UDP 소켓과 연결)
extern size_t rust_embassy_udp_read(uint8_t *ptr, size_t len);
extern size_t rust_embassy_udp_write(const uint8_t *ptr, size_t len, const char *addr, uint16_t port);

z_result_t _z_udp_unicast_address_valid(const _z_string_t *address) {
    (void)address;
    return _Z_RES_OK;
}

z_result_t _z_udp_unicast_endpoint_init(_z_sys_net_endpoint_t *ep, const char *address, const char *port) {
    (void)ep;
    (void)address;
    (void)port;
    return _Z_RES_OK;
}

void _z_udp_unicast_endpoint_clear(_z_sys_net_endpoint_t *ep) {
    (void)ep;
}

z_result_t _z_udp_unicast_endpoint_init_from_address(_z_sys_net_endpoint_t *ep, const _z_string_t *address) {
    (void)ep;
    (void)address;
    return _Z_RES_OK;
}

z_result_t _z_udp_unicast_open(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t endpoint, uint32_t tout) {
    (void)sock;
    (void)endpoint;
    (void)tout;
    return _Z_RES_OK;
}

z_result_t _z_udp_unicast_listen(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t endpoint, uint32_t tout) {
    (void)sock;
    (void)endpoint;
    (void)tout;
    return _Z_RES_OK;
}

void _z_udp_unicast_close(_z_sys_net_socket_t *sock) {
    (void)sock;
}

size_t _z_udp_unicast_read(_z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    (void)sock;
    return rust_embassy_udp_read(ptr, len);
}

size_t _z_udp_unicast_read_exact(_z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    (void)sock;
    return rust_embassy_udp_read(ptr, len);
}

size_t _z_udp_unicast_write(_z_sys_net_socket_t sock, const uint8_t *ptr, size_t len,
                            const _z_sys_net_endpoint_t endpoint) {
    (void)sock;
    (void)endpoint;
    return rust_embassy_udp_write(ptr, len, "192.168.0.100", 7447);
}

// Socket 유틸리티 스텁 (zenoh-pico 베어메탈 단일 UDP 세션)
z_result_t _z_socket_set_blocking(const _z_sys_net_socket_t *sock, bool blocking) {
    (void)sock;
    (void)blocking;
    return _Z_RES_OK;
}

z_result_t _z_socket_wait_readable(const _z_sys_net_socket_t *sock, uint32_t timeout_ms) {
    (void)sock;
    (void)timeout_ms;
    return _Z_RES_OK;
}

void _z_socket_close(_z_sys_net_socket_t *sock) {
    (void)sock;
}

// 미사용 프로토콜 링크/트랜스포트 스텁
z_result_t _z_endpoint_tcp_valid(const void *ep) {
    (void)ep;
    return _Z_ERR_GENERIC;
}

z_result_t _z_raweth_send_n_msg(void *zn, void *msg, int rel, int cong) {
    (void)zn;
    (void)msg;
    (void)rel;
    (void)cong;
    return _Z_ERR_GENERIC;
}

z_result_t _z_multicast_open_client(void *p, void *l, void *z) {
    (void)p;
    (void)l;
    (void)z;
    return _Z_ERR_GENERIC;
}

z_result_t _z_multicast_open_peer(void *p, void *l, void *z) {
    (void)p;
    (void)l;
    (void)z;
    return _Z_ERR_GENERIC;
}

z_result_t _z_multicast_transport_create(void *zt, void *zl, void *p) {
    (void)zt;
    (void)zl;
    (void)p;
    return _Z_ERR_GENERIC;
}

void _z_multicast_transport_clear(void *zt) {
    (void)zt;
}

z_result_t _z_new_peer_tcp(void *ep, void *sock) {
    (void)ep; (void)sock; return _Z_ERR_GENERIC;
}

z_result_t _z_new_link_tcp(void *zl, void *ep) {
    (void)zl; (void)ep; return _Z_ERR_GENERIC;
}

z_result_t _z_endpoint_raweth_valid(void *ep) {
    (void)ep; return _Z_ERR_GENERIC;
}

z_result_t _z_new_link_raweth(void *zl, void *ep) {
    (void)zl; (void)ep; return _Z_ERR_GENERIC;
}

z_result_t _z_raweth_config_from_strn(void *cfg, const char *s, size_t n) {
    (void)cfg; (void)s; (void)n; return _Z_ERR_GENERIC;
}
