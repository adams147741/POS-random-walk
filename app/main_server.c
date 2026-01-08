#include "common/socket.h"
#include "common/protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/select.h>

static void die(const char *msg) { perror(msg); exit(1); }

static int send_error(int fd, int32_t code, const char *msg) {
    rw_error_msg_t e;
    memset(&e, 0, sizeof(e));
    e.code = code;
    snprintf(e.msg, sizeof(e.msg), "%s", msg);
    return rw_send_msg(fd, RW_MSG_ERROR, &e, (uint16_t)sizeof(e));
}

static void skip_payload(int fd, uint16_t len) {
    char tmp[1024];
    uint16_t remaining = len;
    while (remaining > 0) {
        uint16_t chunk = remaining > sizeof(tmp) ? sizeof(tmp) : remaining;
        if (rw_recv_all(fd, tmp, chunk) < 0) return;
        remaining -= chunk;
    }
}

static int validate_create(const rw_create_sim_req_t *r) {
    if (r->w == 0 || r->h == 0) return 0;
    if (r->w > RW_MAX_W || r->h > RW_MAX_H) return 0;

    uint64_t sum = (uint64_t)r->p_up + r->p_down + r->p_left + r->p_right;
    if (sum != RW_PROB_SCALE) return 0;

    if (r->rep_total == 0) return 0;
    if (r->K == 0) return 0;

    if (r->world_type != RW_WORLD_WRAP && r->world_type != RW_WORLD_OBSTACLES) return 0;
    if (r->initial_mode != RW_MODE_INTERACTIVE && r->initial_mode != RW_MODE_SUMMARY) return 0;

    if (r->obstacle_density_permille > 1000) return 0;
    if (r->out_file[0] == '\0') return 0;

    return 1;
}

static int recv_exact(int fd, uint16_t expected_type, void *payload, uint16_t expected_len) {
    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(fd, &type, &len) < 0) return -1;
    if (type != expected_type || len != expected_len) {
        if (len) skip_payload(fd, len);
        return -1;
    }
    if (expected_len) {
        if (rw_recv_all(fd, payload, expected_len) < 0) return -1;
    }
    return 0;
}

static void gen_path_dummy(rw_state_msg_t *st) {
    // bezpečne obmedz na RW_MAX_PATH
    uint32_t L = 0;

    // napr. nech rastie s rep_done, max RW_MAX_PATH
    uint32_t want = 10 + (st->rep_done % 30); // len aby sa menilo
    if (want > RW_MAX_PATH) want = RW_MAX_PATH;

    // jednoduchá "prechádzka": diagonála + odrazy v rámci w/h
    int x = (int)(st->w / 2);
    int y = (int)(st->h / 2);

    for (uint32_t i = 0; i < want; i++) {
        st->path_x[i] = (int16_t)x;
        st->path_y[i] = (int16_t)y;
        L++;

        // posun (len demo)
        x += (i % 2 == 0) ? 1 : -1;
        y += 1;

        // wrap do hraníc 0..w-1 / 0..h-1
        if (st->w > 0) {
            if (x < 0) x += (int)st->w;
            if (x >= (int)st->w) x -= (int)st->w;
        }
        if (st->h > 0) {
            if (y < 0) y += (int)st->h;
            if (y >= (int)st->h) y -= (int)st->h;
        }
    }

    st->path_len = L;
}


static void fill_state_dummy(rw_state_msg_t *st, rw_local_view_t view) {
    // obstacles = 0
    for (uint32_t y = 0; y < st->h; y++) {
        for (uint32_t x = 0; x < st->w; x++) {
            st->obstacle[y * RW_MAX_W + x] = 0;

            // len “cell_value” podľa view, aby bolo vidno rozdiel po stlačení 'v'
            if (view == RW_VIEW_AVG_STEPS) {
                // napr. avg*1000 = rep_done*1000 (dummy)
                st->cell_value[y * RW_MAX_W + x] = st->rep_done * 1000u;
            } else {
                // napr. "pravdepodobnosť" (dummy) = rep_done/rep_total * RW_PROB_SCALE
                uint32_t prob = 0;
                if (st->rep_total > 0) {
                    prob = (uint32_t)((uint64_t)st->rep_done * RW_PROB_SCALE / st->rep_total);
                }
                st->cell_value[y * RW_MAX_W + x] = prob;
            }
        }
    }

    // interactive: pošli aj nejakú ukážkovú "trajektóriu"
    if (st->mode == RW_MODE_INTERACTIVE) {
        gen_path_dummy(st);
        /*
        st->path_len = 6;
        for (uint32_t i = 0; i < st->path_len; i++) {
            st->path_x[i] = (int16_t)i;
            st->path_y[i] = (int16_t)(i % (st->h ? st->h : 1));
        }
        */
    } else {
        st->path_len = 0;
    }
}

static int handle_one_command(int fd, rw_global_mode_t *server_mode, rw_local_view_t *client_view) {
    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(fd, &type, &len) < 0) return -1;

    if (type == RW_MSG_SET_MODE && len == sizeof(rw_set_mode_req_t)) {
        rw_set_mode_req_t req;
        if (rw_recv_all(fd, &req, sizeof(req)) < 0) return -1;

        if (req.mode != RW_MODE_INTERACTIVE && req.mode != RW_MODE_SUMMARY) {
            send_error(fd, 10, "Invalid mode");
            return 0;
        }
        *server_mode = req.mode;
        printf("server: SET_MODE -> %u\n", (unsigned)*server_mode);
        return 0;
    }

    if (type == RW_MSG_SET_VIEW && len == sizeof(rw_set_view_req_t)) {
        rw_set_view_req_t req;
        if (rw_recv_all(fd, &req, sizeof(req)) < 0) return -1;

        if (req.view != RW_VIEW_AVG_STEPS && req.view != RW_VIEW_PROB_K) {
            send_error(fd, 11, "Invalid view");
            return 0;
        }
        *client_view = req.view;
        printf("server: SET_VIEW -> %u\n", (unsigned)*client_view);
        return 0;
    }

    // Zatiaľ ostatné ignorujeme, ale payload musíme dočítať
    if (len) skip_payload(fd, len);
    printf("server: ignored msg type=%u len=%u\n", type, len);
    return 0;
}

int main(void) {
    const uint16_t port = 12345;

    int listen_fd = rw_tcp_listen(NULL, port, 16);
    if (listen_fd < 0) die("rw_tcp_listen");

    printf("server: listening on %u...\n", (unsigned)port);

    int client_fd = accept(listen_fd, NULL, NULL);
    if (client_fd < 0) die("accept");
    printf("server: client connected\n");

    // HELLO
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

    // CREATE_SIM
    rw_create_sim_req_t req;
    if (recv_exact(client_fd, RW_MSG_CREATE_SIM, &req, (uint16_t)sizeof(req)) < 0) {
        send_error(client_fd, 2, "Expected CREATE_SIM");
        close(client_fd); close(listen_fd);
        return 1;
    }

    if (!validate_create(&req)) {
        send_error(client_fd, 3, "CREATE_SIM validation failed");
        rw_create_ack_t nack = {.ok = 0, .sim_id = 0};
        (void)rw_send_msg(client_fd, RW_MSG_CREATE_ACK, &nack, (uint16_t)sizeof(nack));
        close(client_fd); close(listen_fd);
        return 1;
    }

    printf("server: CREATE ok w=%u h=%u reps=%u K=%u\n", req.w, req.h, req.rep_total, req.K);

    rw_create_ack_t ack = {.ok = 1, .sim_id = 1};
    if (rw_send_msg(client_fd, RW_MSG_CREATE_ACK, &ack, (uint16_t)sizeof(ack)) < 0)
        die("rw_send_msg(CREATE_ACK)");

    // ---- Runtime state (this is what we control via SET_MODE/SET_VIEW) ----
    rw_global_mode_t server_mode = req.initial_mode;     // global (for all)
    rw_local_view_t  client_view = RW_VIEW_AVG_STEPS;    // local (per client)

    rw_state_msg_t st;
    memset(&st, 0, sizeof(st));
    st.w = req.w;
    st.h = req.h;
    st.rep_total = req.rep_total;

    // Main loop: every ~200ms send STATE; in-between handle incoming commands
    while (1) {
        // 1) handle incoming commands if any (non-blocking via select)
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(client_fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000; // 200ms "tick"

        int rc = select(client_fd + 1, &rfds, NULL, NULL, &tv);
        if (rc < 0) {
            if (errno == EINTR) continue;
            die("select");
        }

        if (rc > 0 && FD_ISSET(client_fd, &rfds)) {
            // read exactly one command message (could be SET_MODE/SET_VIEW)
            if (handle_one_command(client_fd, &server_mode, &client_view) < 0) {
                printf("server: client disconnected (read error)\n");
                break;
            }
        } else {
            // timeout => tick
            if (st.rep_done < st.rep_total) st.rep_done++;
            st.finished = (st.rep_done >= st.rep_total) ? 1u : 0u;

            st.mode = server_mode;
            fill_state_dummy(&st, client_view);

            if (rw_send_msg(client_fd, RW_MSG_STATE, &st, (uint16_t)sizeof(st)) < 0) {
                printf("server: client disconnected (send error)\n");
                break;
            }

            if (st.finished) {
                printf("server: finished\n");
                break;
            }
        }
    }

    close(client_fd);
    close(listen_fd);
    return 0;
}

