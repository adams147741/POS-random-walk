// src/common/socket.c
#include "common/socket.h"
#include "common/protocol.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>

static int set_reuseaddr(int fd) {
    int yes = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) return -1;
    return 0;
}

// Returns 0 on success, -1 on error.
int rw_send_all(int fd, const void *buf, size_t len) {
    const unsigned char *p = (const unsigned char *)buf;
    size_t sent = 0;

    while (sent < len) {
        ssize_t n = send(fd, p + sent, len - sent, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) { // should not happen for send(), but handle defensively
            errno = ECONNRESET;
            return -1;
        }
        sent += (size_t)n;
    }
    return 0;
}

// Returns 0 on success, -1 on error.
int rw_recv_all(int fd, void *buf, size_t len) {
    unsigned char *p = (unsigned char *)buf;
    size_t recvd = 0;

    while (recvd < len) {
        ssize_t n = recv(fd, p + recvd, len - recvd, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) { // peer closed
            errno = ECONNRESET;
            return -1;
        }
        recvd += (size_t)n;
    }
    return 0;
}

int rw_send_msg(int fd, uint16_t type, const void *payload, uint16_t payload_len) {
    rw_msg_hdr_t hdr;
    // Put header into network byte order so it works across machines too
    hdr.type = htons(type);
    hdr.length = htons(payload_len);

    if (rw_send_all(fd, &hdr, sizeof(hdr)) < 0) return -1;

    if (payload_len > 0) {
        if (payload == NULL) {
            errno = EINVAL;
            return -1;
        }
        if (rw_send_all(fd, payload, payload_len) < 0) return -1;
    }
    return 0;
}

int rw_recv_hdr(int fd, uint16_t *type_out, uint16_t *len_out) {
    rw_msg_hdr_t hdr;
    if (rw_recv_all(fd, &hdr, sizeof(hdr)) < 0) return -1;

    uint16_t type = ntohs(hdr.type);
    uint16_t len  = ntohs(hdr.length);

    if (type_out) *type_out = type;
    if (len_out)  *len_out  = len;
    return 0;
}

int rw_tcp_listen(const char *listen_host, uint16_t port, int backlog) {
    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%u", (unsigned)port);

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_UNSPEC;     // allow IPv4 or IPv6
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags    = AI_PASSIVE;    // for bind()

    struct addrinfo *res = NULL;
    int rc = getaddrinfo(listen_host, port_str, &hints, &res);
    if (rc != 0) {
        errno = EINVAL;
        return -1;
    }

    int listen_fd = -1;
    for (struct addrinfo *ai = res; ai != NULL; ai = ai->ai_next) {
        int fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) continue;

        (void)set_reuseaddr(fd);

        if (bind(fd, ai->ai_addr, ai->ai_addrlen) < 0) {
            close(fd);
            continue;
        }
        if (listen(fd, backlog) < 0) {
            close(fd);
            continue;
        }

        listen_fd = fd;
        break;
    }

    freeaddrinfo(res);

    if (listen_fd < 0) {
        // errno should already be set by the last failure, but set a generic one if not
        if (errno == 0) errno = EADDRINUSE;
        return -1;
    }

    return listen_fd;
}

int rw_tcp_connect(const char *connect_host, uint16_t port) {
    if (connect_host == NULL || connect_host[0] == '\0') {
        errno = EINVAL;
        return -1;
    }

    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%u", (unsigned)port);

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_UNSPEC;   // allow IPv4 or IPv6
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo *res = NULL;
    int rc = getaddrinfo(connect_host, port_str, &hints, &res);
    if (rc != 0) {
        errno = EINVAL;
        return -1;
    }

    int fd_out = -1;
    for (struct addrinfo *ai = res; ai != NULL; ai = ai->ai_next) {
        int fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) continue;

        if (connect(fd, ai->ai_addr, ai->ai_addrlen) < 0) {
            close(fd);
            continue;
        }

        fd_out = fd;
        break;
    }

    freeaddrinfo(res);

    if (fd_out < 0) {
        if (errno == 0) errno = ECONNREFUSED;
        return -1;
    }

    return fd_out;
}
