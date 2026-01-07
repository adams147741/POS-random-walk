#include "common/socket.h"
#include "common/protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static void die(const char *msg) { perror(msg); exit(1); }

int main(void) {
    int fd = rw_tcp_connect("127.0.0.1", 12345);
    if (fd < 0) die("rw_tcp_connect");

    if (rw_send_msg(fd, RW_MSG_HELLO, NULL, 0) < 0) die("rw_send_msg(HELLO)");

    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr");
    if (type != RW_MSG_HELLO_ACK || len != sizeof(rw_hello_ack_t)) {
        fprintf(stderr, "state_client: expected HELLO_ACK, got type=%u len=%u\n", type, len);
        close(fd);
        return 1;
    }

    rw_hello_ack_t ack;
    if (rw_recv_all(fd, &ack, sizeof(ack)) < 0) die("rw_recv_all(ack)");
    printf("state_client: connected, client_id=%u\n", ack.client_id);

    while (1) {
        if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr(loop)");

        if (type == RW_MSG_STATE && len == sizeof(rw_state_msg_t)) {
            rw_state_msg_t st;
            if (rw_recv_all(fd, &st, sizeof(st)) < 0) die("rw_recv_all(state)");

            printf("STATE: %u/%u finished=%u mode=%u w=%u h=%u\n",
                   st.rep_done, st.rep_total, st.finished,
                   (unsigned)st.mode, st.w, st.h);

            if (st.finished) break;
        } else if (type == RW_MSG_ERROR && len == sizeof(rw_error_msg_t)) {
            rw_error_msg_t e;
            if (rw_recv_all(fd, &e, sizeof(e)) < 0) die("rw_recv_all(error)");
            fprintf(stderr, "ERROR: code=%d msg=%s\n", e.code, e.msg);
            break;
        } else {
            fprintf(stderr, "state_client: unexpected msg type=%u len=%u\n", type, len);
            // consume payload so stream stays aligned
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

