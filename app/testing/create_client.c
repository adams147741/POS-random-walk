#include "common/socket.h"
#include "common/protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void die(const char *msg) { perror(msg); exit(1); }

int main(void) {
    int fd = rw_tcp_connect("127.0.0.1", 12345);
    if (fd < 0) die("rw_tcp_connect");

    // HELLO
    if (rw_send_msg(fd, RW_MSG_HELLO, NULL, 0) < 0) die("rw_send_msg(HELLO)");

    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr(HELLO_ACK)");
    if (type != RW_MSG_HELLO_ACK || len != sizeof(rw_hello_ack_t)) {
        fprintf(stderr, "create_client: expected HELLO_ACK\n");
        close(fd);
        return 1;
    }

    rw_hello_ack_t hello;
    if (rw_recv_all(fd, &hello, sizeof(hello)) < 0) die("rw_recv_all(hello)");
    printf("create_client: connected client_id=%u\n", hello.client_id);

    // CREATE_SIM (zatim natvrdo)
    rw_create_sim_req_t req;
    memset(&req, 0, sizeof(req));

    req.w = 10;
    req.h = 6;
    req.rep_total = 20;
    req.K = 200;

    req.p_up = 250000;
    req.p_down = 250000;
    req.p_left = 250000;
    req.p_right = 250000;

    req.world_type = RW_WORLD_WRAP;
    req.initial_mode = RW_MODE_SUMMARY;
    req.obstacle_density_permille = 0;

    snprintf(req.out_file, sizeof(req.out_file), "data/results/out.txt");

    if (rw_send_msg(fd, RW_MSG_CREATE_SIM, &req, (uint16_t)sizeof(req)) < 0)
        die("rw_send_msg(CREATE_SIM)");

    // CREATE_ACK
    if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr(CREATE_ACK)");
    if (type == RW_MSG_ERROR && len == sizeof(rw_error_msg_t)) {
        rw_error_msg_t e;
        rw_recv_all(fd, &e, sizeof(e));
        fprintf(stderr, "ERROR: code=%d msg=%s\n", e.code, e.msg);
        close(fd);
        return 1;
    }
    if (type != RW_MSG_CREATE_ACK || len != sizeof(rw_create_ack_t)) {
        fprintf(stderr, "create_client: expected CREATE_ACK\n");
        close(fd);
        return 1;
    }

    rw_create_ack_t ack;
    if (rw_recv_all(fd, &ack, sizeof(ack)) < 0) die("rw_recv_all(create_ack)");
    printf("create_client: CREATE_ACK ok=%u sim_id=%u\n", ack.ok, ack.sim_id);
    if (!ack.ok) {
        close(fd);
        return 1;
    }

    // Receive STATE stream
    while (1) {
        if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr(loop)");

        if (type == RW_MSG_STATE && len == sizeof(rw_state_msg_t)) {
            rw_state_msg_t st;
            if (rw_recv_all(fd, &st, sizeof(st)) < 0) die("rw_recv_all(state)");

            printf("STATE: %u/%u finished=%u w=%u h=%u mode=%u\n",
                   st.rep_done, st.rep_total, st.finished, st.w, st.h, (unsigned)st.mode);

            if (st.finished) break;
        } else if (type == RW_MSG_ERROR && len == sizeof(rw_error_msg_t)) {
            rw_error_msg_t e;
            if (rw_recv_all(fd, &e, sizeof(e)) < 0) die("rw_recv_all(error)");
            fprintf(stderr, "ERROR: code=%d msg=%s\n", e.code, e.msg);
            break;
        } else {
            // skip unknown payload
            char tmp[1024];
            uint16_t remaining = len;
            while (remaining > 0) {
                uint16_t chunk = remaining > sizeof(tmp) ? sizeof(tmp) : remaining;
                if (rw_recv_all(fd, tmp, chunk) < 0) die("rw_recv_all(skip)");
                remaining -= chunk;
            }
        }
    }

    close(fd);
    return 0;
}

