#include "common/socket.h"
#include "common/protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>

static void die(const char *msg) {
    perror(msg);
    exit(1);
}

int main(void) {
    const uint16_t port = 12345;

    int listen_fd = rw_tcp_listen(NULL, port, 16);
    if (listen_fd < 0) die("rw_tcp_listen");

    printf("test_server: listening on port %u...\n", (unsigned)port);

    int client_fd = accept(listen_fd, NULL, NULL);
    if (client_fd < 0) die("accept");

    printf("test_server: client connected\n");

    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(client_fd, &type, &len) < 0) die("rw_recv_hdr");

    if (type != RW_MSG_HELLO || len != 0) {
        fprintf(stderr, "test_server: unexpected msg type=%u len=%u\n", type, len);
        close(client_fd);
        close(listen_fd);
        return 1;
    }

    rw_hello_ack_t ack = { .client_id = 1 };
    if (rw_send_msg(client_fd, RW_MSG_HELLO_ACK, &ack, (uint16_t)sizeof(ack)) < 0)
        die("rw_send_msg(HELLO_ACK)");

    printf("test_server: sent HELLO_ACK (client_id=1)\n");

    close(client_fd);
    close(listen_fd);
    return 0;
}

