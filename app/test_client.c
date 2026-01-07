#include "common/socket.h"
#include "common/protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static void die(const char *msg) {
    perror(msg);
    exit(1);
}

int main(void) {
    int fd = rw_tcp_connect("127.0.0.1", 12345);
    if (fd < 0) die("rw_tcp_connect");

    if (rw_send_msg(fd, RW_MSG_HELLO, NULL, 0) < 0) die("rw_send_msg(HELLO)");

    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr");

    if (type != RW_MSG_HELLO_ACK || len != sizeof(rw_hello_ack_t)) {
        fprintf(stderr, "test_client: unexpected msg type=%u len=%u\n", type, len);
        close(fd);
        return 1;
    }

    rw_hello_ack_t ack;
    if (rw_recv_all(fd, &ack, sizeof(ack)) < 0) die("rw_recv_all(ack)");

    printf("test_client: HELLO_ACK received, client_id=%u\n", ack.client_id);

    close(fd);
    return 0;
}

