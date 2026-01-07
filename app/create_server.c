#include "common/socket.h"
#include "common/protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>

static void die(const char *msg) { perror(msg); exit(1); }

static int validate_create(const rw_create_sim_req_t *r) {
    if (r->w == 0 || r->h == 0) return 0;
    if (r->w > RW_MAX_W || r->h > RW_MAX_H) return 0;

    // probability sum must match RW_PROB_SCALE
    uint64_t sum = (uint64_t)r->p_up + r->p_down + r->p_left + r->p_right;
    if (sum != RW_PROB_SCALE) return 0;

    if (r->rep_total == 0) return 0;
    if (r->K == 0) return 0;

    if (r->world_type != RW_WORLD_WRAP && r->world_type != RW_WORLD_OBSTACLES) return 0;
    if (r->initial_mode != RW_MODE_INTERACTIVE && r->initial_mode != RW_MODE_SUMMARY) return 0;

    if (r->obstacle_density_permille > 1000) return 0;

    // out_file should be non-empty (optional, but useful)
    if (r->out_file[0] == '\0') return 0;

    return 1;
}

static int send_error(int fd, int32_t code, const char *msg) {
    rw_error_msg_t e;
    memset(&e, 0, sizeof(e));
    e.code = code;
    snprintf(e.msg, sizeof(e.msg), "%s", msg);
    return rw_send_msg(fd, RW_MSG_ERROR, &e, (uint16_t)sizeof(e));
}

int main(void) {
    const uint16_t port = 12345;

    int listen_fd = rw_tcp_listen(NULL, port, 16);
    if (listen_fd < 0) die("rw_tcp_listen");

    printf("create_server: listening on %u...\n", (unsigned)port);

    int client_fd = accept(listen_fd, NULL, NULL);
    if (client_fd < 0) die("accept");

    // ---- HELLO ----
    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(client_fd, &type, &len) < 0) die("rw_recv_hdr(HELLO)");
    if (type != RW_MSG_HELLO || len != 0) {
        send_error(client_fd, 1, "Expected HELLO");
        close(client_fd); close(listen_fd);
        return 1;
    }

    rw_hello_ack_t hello = {.client_id = 1};
    if (rw_send_msg(client_fd, RW_MSG_HELLO_ACK, &hello, (uint16_t)sizeof(hello)) < 0)
        die("rw_send_msg(HELLO_ACK)");

    // ---- CREATE_SIM ----
    if (rw_recv_hdr(client_fd, &type, &len) < 0) die("rw_recv_hdr(CREATE)");
    if (type != RW_MSG_CREATE_SIM || len != sizeof(rw_create_sim_req_t)) {
        send_error(client_fd, 2, "Expected CREATE_SIM");
        close(client_fd); close(listen_fd);
        return 1;
    }

    rw_create_sim_req_t req;
    if (rw_recv_all(client_fd, &req, sizeof(req)) < 0) die("rw_recv_all(create_req)");

    if (!validate_create(&req)) {
        send_error(client_fd, 3, "CREATE_SIM validation failed");
        rw_create_ack_t ack = {.ok = 0, .sim_id = 0};
        rw_send_msg(client_fd, RW_MSG_CREATE_ACK, &ack, (uint16_t)sizeof(ack));
        close(client_fd); close(listen_fd);
        return 1;
    }

    printf("create_server: CREATE ok w=%u h=%u reps=%u K=%u mode=%u world=%u out=%s\n",
           req.w, req.h, req.rep_total, req.K, (unsigned)req.initial_mode,
           (unsigned)req.world_type, req.out_file);

    rw_create_ack_t ack = {.ok = 1, .sim_id = 1};
    if (rw_send_msg(client_fd, RW_MSG_CREATE_ACK, &ack, (uint16_t)sizeof(ack)) < 0)
        die("rw_send_msg(CREATE_ACK)");

    // ---- STATE streaming (fake progress) ----
    rw_state_msg_t st;
    memset(&st, 0, sizeof(st));
    st.w = req.w;
    st.h = req.h;
    st.mode = req.initial_mode;
    st.rep_total = req.rep_total;

    for (uint32_t i = 0; i <= st.rep_total; i++) {
        st.rep_done = i;
        st.finished = (i == st.rep_total) ? 1u : 0u;

        // dummy fill (later will be real stats)
        for (uint32_t y = 0; y < st.h; y++) {
            for (uint32_t x = 0; x < st.w; x++) {
                st.cell_value[y * RW_MAX_W + x] = i;
                st.obstacle[y * RW_MAX_W + x] = 0;
            }
        }

        if (rw_send_msg(client_fd, RW_MSG_STATE, &st, (uint16_t)sizeof(st)) < 0) {
            die("rw_send_msg(STATE)");
        }
        usleep(200000);
    }

    printf("create_server: done\n");
    close(client_fd);
    close(listen_fd);
    return 0;
}

