#include "common/socket.h"
#include "common/protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>

static void die(const char *msg) { perror(msg); exit(1); }

static void skip_payload(int fd, uint16_t len) {
    char tmp[1024];
    uint16_t remaining = len;
    while (remaining > 0) {
        uint16_t chunk = remaining > sizeof(tmp) ? sizeof(tmp) : remaining;
        if (rw_recv_all(fd, tmp, chunk) < 0) return;
        remaining -= chunk;
    }
}

static int read_u32(const char *prompt, uint32_t *out) {
    char line[128];
    printf("%s", prompt);
    fflush(stdout);

    if (!fgets(line, sizeof(line), stdin)) return 0;

    errno = 0;
    char *end = NULL;
    unsigned long v = strtoul(line, &end, 10);
    if (errno != 0 || end == line) return 0;

    *out = (uint32_t)v;
    return 1;
}

static void read_string(const char *prompt, char *dst, size_t dst_size) {
    char line[256];
    printf("%s", prompt);
    fflush(stdout);

    if (!fgets(line, sizeof(line), stdin)) {
        dst[0] = '\0';
        return;
    }

    size_t n = strcspn(line, "\r\n");
    line[n] = '\0';
    snprintf(dst, dst_size, "%s", line);
}

static int read_yes_no(const char *prompt) {
    printf("%s", prompt);
    fflush(stdout);
    int c = getchar();
    while (c == '\n' || c == '\r') c = getchar();
    return (c == 'y' || c == 'Y');
}

static void print_defaults(void) {
    printf("\n--- Defaults (press Enter by typing the same value manually for now) ---\n");
    printf("w=10 h=6 rep_total=20 K=200\n");
    printf("p_up=p_down=p_left=p_right=250000 (sum=%u)\n", (unsigned)RW_PROB_SCALE);
    printf("world_type=1 (wrap), mode=2 (summary), obstacles=0\n");
    printf("out_file=data/results/out.txt\n");
    printf("---------------------------------------------------------------\n\n");
}

static int build_create_req_from_input(rw_create_sim_req_t *req) {
    memset(req, 0, sizeof(*req));

    printf("\n--- CREATE_SIM input ---\n");
    print_defaults();

    if (!read_u32("World width w (<=60): ", &req->w)) req->w = 10;
    if (!read_u32("World height h (<=30): ", &req->h)) req->h = 6;
    if (!read_u32("Replications rep_total: ", &req->rep_total)) req->rep_total = 20;
    if (!read_u32("K (max steps for prob): ", &req->K)) req->K = 200;

    if (!read_u32("p_up: ", &req->p_up)) req->p_up = 250000;
    if (!read_u32("p_down: ", &req->p_down)) req->p_down = 250000;
    if (!read_u32("p_left: ", &req->p_left)) req->p_left = 250000;
    if (!read_u32("p_right: ", &req->p_right)) req->p_right = 250000;

    uint32_t wt = 1, mode = 2;
    if (!read_u32("world_type (1=wrap, 2=obstacles): ", &wt)) wt = 1;
    if (!read_u32("mode (1=interactive, 2=summary): ", &mode)) mode = 2;

    req->world_type = (rw_world_type_t)wt;
    req->initial_mode = (rw_global_mode_t)mode;

    if (!read_u32("obstacle density permille (0..1000): ", &req->obstacle_density_permille))
        req->obstacle_density_permille = 0;

    read_string("out_file path: ", req->out_file, sizeof(req->out_file));
    if (req->out_file[0] == '\0') strcpy(req->out_file, "data/results.output.txt");

    // basic validation
    if (req->w == 0 || req->h == 0 || req->w > RW_MAX_W || req->h > RW_MAX_H) {
        fprintf(stderr, "Invalid w/h.\n");
        return 0;
    }
    if (req->rep_total == 0 || req->K == 0) {
        fprintf(stderr, "rep_total and K must be > 0.\n");
        return 0;
    }
    uint64_t sum = (uint64_t)req->p_up + req->p_down + req->p_left + req->p_right;
    if (sum != RW_PROB_SCALE) {
        fprintf(stderr, "Probabilities must sum to %u.\n", (unsigned)RW_PROB_SCALE);
        return 0;
    }
    if (!(req->world_type == RW_WORLD_WRAP || req->world_type == RW_WORLD_OBSTACLES)) {
        fprintf(stderr, "world_type must be 1 or 2.\n");
        return 0;
    }
    if (!(req->initial_mode == RW_MODE_INTERACTIVE || req->initial_mode == RW_MODE_SUMMARY)) {
        fprintf(stderr, "mode must be 1 or 2.\n");
        return 0;
    }
    if (req->obstacle_density_permille > 1000) {
        fprintf(stderr, "obstacle_density_permille must be 0..1000.\n");
        return 0;
    }
    if (req->out_file[0] == '\0') {
        fprintf(stderr, "out_file must be non-empty.\n");
        return 0;
    }
    return 1;
}

typedef struct {
    int fd;
    atomic_int mode;       // rw_global_mode_t
    atomic_int view;       // rw_local_view_t
    atomic_bool stop;
} client_ctx_t;

static void *receiver_thread(void *arg) {
    client_ctx_t *ctx = (client_ctx_t *)arg;

    while (!atomic_load(&ctx->stop)) {
        uint16_t type = 0, len = 0;
        if (rw_recv_hdr(ctx->fd, &type, &len) < 0) {
            fprintf(stderr, "receiver: disconnected\n");
            atomic_store(&ctx->stop, 1);
            break;
        }

        if (type == RW_MSG_STATE && len == sizeof(rw_state_msg_t)) {
            rw_state_msg_t st;
            if (rw_recv_all(ctx->fd, &st, sizeof(st)) < 0) {
                fprintf(stderr, "receiver: read state failed\n");
                atomic_store(&ctx->stop, 1);
                break;
            }

            printf("STATE: %u/%u finished=%u mode=%u w=%u h=%u cell[0]=%u\n",
                   st.rep_done, st.rep_total, st.finished,
                   (unsigned)st.mode, st.w, st.h,
                   st.cell_value[0]);

            if (st.mode == RW_MODE_INTERACTIVE) {
                printf("PATH len=%u: ", st.path_len);
                uint32_t show = st.path_len;
                if (show > 12) show = 12;
                for (uint32_t i = 0; i < show; i++) {
                    printf("(%d,%d) ", (int)st.path_x[i], (int)st.path_y[i]);
                }
                if (st.path_len > show) printf("...");
                printf("\n");
            }

            if (st.finished) {
                atomic_store(&ctx->stop, 1);
                break;
            }
        } else if (type == RW_MSG_ERROR && len == sizeof(rw_error_msg_t)) {
            rw_error_msg_t e;
            if (rw_recv_all(ctx->fd, &e, sizeof(e)) < 0) {
                fprintf(stderr, "receiver: read error failed\n");
                atomic_store(&ctx->stop, 1);
                break;
            }

            // code==0 -> INFO, do not stop
            if (e.code == 0) {
                printf("server info: %s\n", e.msg);
                continue;
            }

            fprintf(stderr, "ERROR: code=%d msg=%s\n", e.code, e.msg);
            atomic_store(&ctx->stop, 1);
            break;
        } else {
            if (len) skip_payload(ctx->fd, len);
        }
    }

    return NULL;
}

static void *input_thread(void *arg) {
    client_ctx_t *ctx = (client_ctx_t *)arg;

    printf("\nControls: [m]=toggle mode  [v]=toggle view [s]=stop sim [q]=quit\n");

    while (!atomic_load(&ctx->stop)) {
        int c = getchar();
        if (c == EOF) {
            atomic_store(&ctx->stop, 1);
            break;
        }

        if (c == '\n' || c == '\r') continue;

        if (c == 'm') {
            int cur = atomic_load(&ctx->mode);
            int next = (cur == RW_MODE_SUMMARY) ? RW_MODE_INTERACTIVE : RW_MODE_SUMMARY;
            atomic_store(&ctx->mode, next);

            rw_set_mode_req_t req = { .mode = (rw_global_mode_t)next };
            if (rw_send_msg(ctx->fd, RW_MSG_SET_MODE, &req, (uint16_t)sizeof(req)) < 0) {
                fprintf(stderr, "input: send SET_MODE failed\n");
                atomic_store(&ctx->stop, 1);
                break;
            }
            printf("client: sent SET_MODE -> %d\n", next);
        }

        if (c == 'v') {
            int cur = atomic_load(&ctx->view);
            int next = (cur == RW_VIEW_AVG_STEPS) ? RW_VIEW_PROB_K : RW_VIEW_AVG_STEPS;
            atomic_store(&ctx->view, next);

            rw_set_view_req_t req = { .view = (rw_local_view_t)next };
            if (rw_send_msg(ctx->fd, RW_MSG_SET_VIEW, &req, (uint16_t)sizeof(req)) < 0) {
                fprintf(stderr, "input: send SET_VIEW failed\n");
                atomic_store(&ctx->stop, 1);
                break;
            }
            printf("client: sent SET_VIEW -> %d\n", next);
        }

        if (c == 'q') {
            atomic_store(&ctx->stop, 1);
            shutdown(ctx->fd, SHUT_RDWR);
            printf("client: quitting...\n");
            break;
        }

        if (c == 's') {
          rw_stop_req_t req = { .reason = 1};
          if (rw_send_msg(ctx->fd, RW_MSG_STOP_SIM, &req, (uint16_t)sizeof(req)) < 0) {
            fprintf(stderr, "input: send STOP_SIM failed\n");
            atomic_store(&ctx->stop, 1);
            break;
          }
          printf("client: sent STOP_SIM\n");
        }
    }

    return NULL;
}

// returns 1 if server says sim exists/running, 0 if not, -1 if no info
static int recv_server_info(int fd, rw_error_msg_t *out_info) {
    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(fd, &type, &len) < 0) return -1;

    if (type == RW_MSG_ERROR && len == sizeof(rw_error_msg_t)) {
        rw_error_msg_t e;
        if (rw_recv_all(fd, &e, sizeof(e)) < 0) return -1;
        if (out_info) *out_info = e;

        if (e.code == 0) {
            // heuristic: message contains "running"
            return (strstr(e.msg, "running") != NULL) ? 1 : 0;
        }
        // real error
        fprintf(stderr, "server error: code=%d msg=%s\n", e.code, e.msg);
        return -1;
    }

    // unknown, ignore
    if (len) skip_payload(fd, len);
    return -1;
}

int main(void) {
    int fd = rw_tcp_connect("127.0.0.1", 12345);
    if (fd < 0) die("rw_tcp_connect");

    // HELLO
    if (rw_send_msg(fd, RW_MSG_HELLO, NULL, 0) < 0) die("rw_send_msg(HELLO)");

    // HELLO_ACK
    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr(HELLO_ACK)");
    if (type != RW_MSG_HELLO_ACK || len != sizeof(rw_hello_ack_t)) {
        fprintf(stderr, "client: expected HELLO_ACK, got type=%u len=%u\n", type, len);
        if (len) skip_payload(fd, len);
        close(fd);
        return 1;
    }

    rw_hello_ack_t hello;
    if (rw_recv_all(fd, &hello, sizeof(hello)) < 0) die("rw_recv_all(hello)");
    printf("client: connected client_id=%u\n", hello.client_id);

    // INFO (server sends as RW_MSG_ERROR with code=0)
    rw_error_msg_t info;
    printf("waiting for server info");
    memset(&info, 0, sizeof(info));
    int sim_running = recv_server_info(fd, &info);
    if (sim_running >= 0 && info.code == 0) {
        printf("server info: %s\n", info.msg);
    }
    printf("server info arrived!");
    rw_global_mode_t start_mode = RW_MODE_SUMMARY;

    if (sim_running == 0) {
        // no simulation -> must create
        if (!read_yes_no("No active simulation. Create it now? (y/n): ")) {
            close(fd);
            return 0;
        }

        rw_create_sim_req_t req;
        if (!build_create_req_from_input(&req)) {
            fprintf(stderr, "client: invalid input\n");
            close(fd);
            return 1;
        }

        if (rw_send_msg(fd, RW_MSG_CREATE_SIM, &req, (uint16_t)sizeof(req)) < 0)
            die("rw_send_msg(CREATE_SIM)");

        if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr(CREATE_ACK)");
        if (type != RW_MSG_CREATE_ACK || len != sizeof(rw_create_ack_t)) {
            fprintf(stderr, "client: expected CREATE_ACK, got type=%u len=%u\n", type, len);
            if (len) skip_payload(fd, len);
            close(fd);
            return 1;
        }

        rw_create_ack_t ack;
        if (rw_recv_all(fd, &ack, sizeof(ack)) < 0) die("rw_recv_all(create_ack)");
        printf("client: CREATE_ACK ok=%u sim_id=%u\n", ack.ok, ack.sim_id);
        if (!ack.ok) { close(fd); return 1; }

        start_mode = req.initial_mode;
    } else if (sim_running == 1) {
        // sim exists -> confirm join
        if (!read_yes_no("Simulation is running. Join it now? (y/n): ")) {
            close(fd);
            return 0;
        }

        rw_join_req_t jr = { .sim_id = 1 };
        if (rw_send_msg(fd, RW_MSG_JOIN_SIM, &jr, (uint16_t)sizeof(jr)) < 0)
            die("rw_send_msg(JOIN_SIM)");

        if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr(JOIN_ACK)");
        if (type != RW_MSG_JOIN_ACK || len != sizeof(rw_join_ack_t)) {
            fprintf(stderr, "client: expected JOIN_ACK, got type=%u len=%u\n", type, len);
            if (len) skip_payload(fd, len);
            close(fd);
            return 1;
        }

        rw_join_ack_t ja;
        if (rw_recv_all(fd, &ja, sizeof(ja)) < 0) die("rw_recv_all(JOIN_ACK)");
        if (!ja.ok) {
            fprintf(stderr, "JOIN denied by server\n");
            close(fd);
            return 1;
        }

        printf("client: joined sim (w=%u h=%u rep_total=%u K=%u mode_now=%u rep_done=%u)\n",
               ja.w, ja.h, ja.rep_total, ja.K, (unsigned)ja.mode_now, ja.rep_done);

        start_mode = ja.mode_now;
    } else {
        // no info / older server -> fallback to create (old behavior)
        fprintf(stderr, "client: server did not send info; falling back to CREATE\n");

        rw_create_sim_req_t req;
        if (!build_create_req_from_input(&req)) {
            fprintf(stderr, "client: invalid input\n");
            close(fd);
            return 1;
        }

        if (rw_send_msg(fd, RW_MSG_CREATE_SIM, &req, (uint16_t)sizeof(req)) < 0)
            die("rw_send_msg(CREATE_SIM)");

        if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr(CREATE_ACK)");
        if (type != RW_MSG_CREATE_ACK || len != sizeof(rw_create_ack_t)) {
            fprintf(stderr, "client: expected CREATE_ACK, got type=%u len=%u\n", type, len);
            if (len) skip_payload(fd, len);
            close(fd);
            return 1;
        }

        rw_create_ack_t ack;
        if (rw_recv_all(fd, &ack, sizeof(ack)) < 0) die("rw_recv_all(create_ack)");
        printf("client: CREATE_ACK ok=%u sim_id=%u\n", ack.ok, ack.sim_id);
        if (!ack.ok) { close(fd); return 1; }

        start_mode = req.initial_mode;
    }

    // Threads
    client_ctx_t ctx;
    ctx.fd = fd;
    atomic_init(&ctx.mode, (int)start_mode);
    atomic_init(&ctx.view, (int)RW_VIEW_AVG_STEPS);
    atomic_init(&ctx.stop, 0);

    pthread_t th_recv, th_in;
    if (pthread_create(&th_recv, NULL, receiver_thread, &ctx) != 0) die("pthread_create(recv)");
    if (pthread_create(&th_in, NULL, input_thread, &ctx) != 0) die("pthread_create(input)");

    pthread_join(th_recv, NULL);
    atomic_store(&ctx.stop, 1);
    shutdown(fd, SHUT_RDWR);
    pthread_join(th_in, NULL);

    close(fd);
    return 0;
}

