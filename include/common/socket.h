#include <stddef.h>
#include <stdint.h>

#ifndef SOCKET_H
#define SOCKET_H

// Returns 0 on success, -1 on error (errno is set).
int rw_send_all(int fd, const void *buf, size_t len);
int rw_recv_all(int fd, void *buf, size_t len);

// Convenience helpers
int rw_send_msg(int fd, uint16_t type, const void *payload, uint16_t payload_len);
int rw_recv_hdr(int fd, uint16_t *type_out, uint16_t *len_out);

// TCP helpers
// listen_host: if NULL -> bind to INADDR_ANY
int rw_tcp_listen(const char *listen_host, uint16_t port, int backlog);
// connect_host: hostname or IP
int rw_tcp_connect(const char *connect_host, uint16_t port);

#endif

