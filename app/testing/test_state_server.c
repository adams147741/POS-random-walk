#include "common/socket.h"
#include "common/protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>

static void die(const char *msg) { perror(msg); exit(1); }

int main(void) {
    const uint16_t port = 12345;

    int listen_fd = rw_tcp_listen(NULL, port, 16);
    if (listen_fd < 0) die("rw_tcp_listen");

    printf("state_server: listening on %u...\n", (unsigned)port);

    int client_fd = accept(listen_fd, NULL, NULL);
    if (client_fd < 0) die("accept");

    // Expect HELLO
    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(client_fd, &type, &len) < 0) die("rw_recv_hdr");
    if (type != RW_MSG_HELLO || len != 0) {
        fprintf(stderr, "state_server: expected HELLO, got type=%u len=%u\n", type, len);
        close(client_fd);
        close(listen_fd);
        return 1;
    }

    // Send HELLO_ACK
    rw_hello_ack_t ack = {.client_id = 1};
    if (rw_send_msg(client_fd, RW_MSG_HELLO_ACK, &ack, (uint16_t)sizeof(ack)) < 0)
        die("rw_send_msg(HELLO_ACK)");

    // Periodically send STATE
    rw_state_msg_t st;
    memset(&st, 0, sizeof(st));
    st.w = 10;
    st.h = 6;
    st.mode = RW_MODE_SUMMARY;
    st.rep_total = 20;

    for (uint32_t i = 0; i <= st.rep_total; i++) {
        st.rep_done = i;
        st.finished = (i == st.rep_total) ? 1u : 0u;

        // Fill some dummy values just so client sees changes
        for (uint32_t y = 0; y < st.h; y++) {
            for (uint32_t x = 0; x < st.w; x++) {
                st.cell_value[y * RW_MAX_W + x] = i; // simple counter
                st.obstacle[y * RW_MAX_W + x] = 0;
            }
        }

        if (rw_send_msg(client_fd, RW_MSG_STATE, &st, (uint16_t)sizeof(st)) < 0) {
            die("rw_send_msg(STATE)");
        }

        usleep(200000); // 200 ms
    }

    printf("state_server: done\n");
    close(client_fd);
    close(listen_fd);
    return 0;
}

