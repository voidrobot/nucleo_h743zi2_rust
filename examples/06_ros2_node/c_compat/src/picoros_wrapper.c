#include "picoros_wrapper.h"
#include "picoros.h"
#include <string.h>

#define MAX_PUBS 8
#define MAX_SUBS 4
#define MAX_SRVS 2

static picoros_interface_t s_ifx;
static picoros_node_t s_node;
static bool s_initialized = false;

static picoros_publisher_t s_pubs[MAX_PUBS];
static size_t s_pub_count = 0;

static picoros_subscriber_t s_subs[MAX_SUBS];
static rust_sub_cb_t s_sub_cbs[MAX_SUBS];
static size_t s_sub_count = 0;

static picoros_srv_server_t s_srvs[MAX_SRVS];
static rust_srv_cb_t s_srv_cbs[MAX_SRVS];
static size_t s_srv_count = 0;

static uint8_t s_srv_resp_buf[256];

int picoros_rust_init(const char *locator) {
    (void)memset(&s_ifx, 0, sizeof(s_ifx));
    s_ifx.mode = "client";
    s_ifx.locator = (char *)locator;

    int res = picoros_interface_init(&s_ifx);
    if (res != PICOROS_OK) {
        return res;
    }
    s_initialized = true;
    return 0;
}

int picoros_rust_node_init(const char *name, uint32_t domain_id) {
    if (!s_initialized) return -100;

    (void)memset(&s_node, 0, sizeof(s_node));
    s_node.name = name;
    s_node.domain_id = domain_id;

    int res = picoros_node_init(&s_node);
    if (res != PICOROS_OK) {
        return res;
    }
    return 0;
}

int picoros_rust_spin_once(void) {
    if (!s_initialized) return -1;
    return (picoros_single_threaded_loop(&s_ifx) == PICOROS_OK) ? 0 : -1;
}

void *picoros_rust_pub_create(const char *name, const char *type, const char *hash) {
    if (s_pub_count >= MAX_PUBS) return NULL;

    picoros_publisher_t *p = &s_pubs[s_pub_count++];
    (void)memset(p, 0, sizeof(*p));
    p->topic.name = name;
    p->topic.type = type;
    p->topic.rihs_hash = hash;

    if (picoros_publisher_declare(&s_node, p) != PICOROS_OK) {
        s_pub_count--;
        return NULL;
    }
    return (void *)p;
}

int picoros_rust_pub_send(void *handle, const uint8_t *payload, size_t len) {
    if (!handle) return -1;
    picoros_publisher_t *p = (picoros_publisher_t *)handle;
    return (picoros_publish(p, (uint8_t *)payload, len) == PICOROS_OK) ? 0 : -1;
}

static void internal_sub_trampoline(uint8_t *rx_data, size_t data_len) {
    // 서브스크라이버 콜백 전달 (단일 서브스크라이버 지원)
    if (s_sub_count > 0 && s_sub_cbs[0]) {
        s_sub_cbs[0](rx_data, data_len);
    }
}

void *picoros_rust_sub_create(const char *name, const char *type, const char *hash, rust_sub_cb_t cb) {
    if (s_sub_count >= MAX_SUBS) return NULL;

    size_t idx = s_sub_count++;
    picoros_subscriber_t *s = &s_subs[idx];
    s_sub_cbs[idx] = cb;

    (void)memset(s, 0, sizeof(*s));
    s->topic.name = name;
    s->topic.type = type;
    s->topic.rihs_hash = hash;
    s->user_callback = internal_sub_trampoline;

    if (picoros_subscriber_declare(&s_node, s) != PICOROS_OK) {
        s_sub_count--;
        return NULL;
    }
    return (void *)s;
}

static picoros_service_reply_t internal_srv_trampoline(struct picoros_srv_server_s *server,
                                                       uint8_t *request_data,
                                                       size_t request_size) {
    (void)server;
    picoros_service_reply_t reply;
    reply.data = NULL;
    reply.length = 0;
    reply.free_callback = NULL;

    if (s_srv_count > 0 && s_srv_cbs[0]) {
        size_t resp_len = s_srv_cbs[0](request_data, request_size, s_srv_resp_buf, sizeof(s_srv_resp_buf));
        reply.data = s_srv_resp_buf;
        reply.length = resp_len;
    }
    return reply;
}

void *picoros_rust_srv_create(const char *name, const char *type, const char *hash, rust_srv_cb_t cb) {
    if (s_srv_count >= MAX_SRVS) return NULL;

    size_t idx = s_srv_count++;
    picoros_srv_server_t *srv = &s_srvs[idx];
    s_srv_cbs[idx] = cb;

    (void)memset(srv, 0, sizeof(*srv));
    srv->topic.name = name;
    srv->topic.type = type;
    srv->topic.rihs_hash = hash;
    srv->user_callback = internal_srv_trampoline;

    if (picoros_service_declare(&s_node, srv) != PICOROS_OK) {
        s_srv_count--;
        return NULL;
    }
    return (void *)srv;
}
