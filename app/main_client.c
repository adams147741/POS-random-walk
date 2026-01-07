#include "common/socket.h"
#include "common/protocol.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

static void die(const char *msg) { perror(msg); exit(1); }

static void skip_payload(int fd, uint16_t len) {
    char tmp[1024];
    uint16_t remaining = len;
    while (remaining > 0) {
        uint16_t chunk = remaining > sizeof(tmp) ? sizeof(tmp) : remaining;
        if (rw_recv_all(fd, tmp, chunk) < 0) die("rw_recv_all(skip)");
        remaining -= chunk;
    }
}

static int recv_error_if_any(int fd, uint16_t type, uint16_t len) {
    if (type == RW_MSG_ERROR && len == sizeof(rw_error_msg_t)) {
        rw_error_msg_t e;
        if (rw_recv_all(fd, &e, sizeof(e)) < 0) die("rw_recv_all(error)");
        fprintf(stderr, "ERROR: code=%d msg=%s\n", e.code, e.msg);
        return 1;
    }
    return 0;
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

    // remove trailing \n
    size_t n = strcspn(line, "\r\n");
    line[n] = '\0';

    snprintf(dst, dst_size, "%s", line);
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

    print_defaults();
    
    uint32_t defaults = 0;

    if (!read_u32("Use defaults? (1 = yes, 0 = no)", &defaults)) return 0;

    if (!read_u32("World width w (<=60): ", &req->w)) return 0;
    if (!read_u32("World height h (<=30): ", &req->h)) return 0;
    if (!read_u32("Replications rep_total: ", &req->rep_total)) return 0;
    if (!read_u32("K (max steps for prob): ", &req->K)) return 0;

    if (!read_u32("p_up: ", &req->p_up)) return 0;
    if (!read_u32("p_down: ", &req->p_down)) return 0;
    if (!read_u32("p_left: ", &req->p_left)) return 0;
    if (!read_u32("p_right: ", &req->p_right)) return 0;

    uint32_t wt = 0, mode = 0;
    if (!read_u32("world_type (1=wrap, 2=obstacles): ", &wt)) return 0;
    if (!read_u32("mode (1=interactive, 2=summary): ", &mode)) return 0;

    req->world_type = (rw_world_type_t)wt;
    req->initial_mode = (rw_global_mode_t)mode;

    if (!read_u32("obstacle density permille (0..1000): ", &req->obstacle_density_permille)) return 0;

    read_string("out_file path: ", req->out_file, sizeof(req->out_file));

    // --- client-side validation (same idea as server) ---
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
        fprintf(stderr, "Probabilities must sum to %u, got %llu.\n",
                (unsigned)RW_PROB_SCALE, (unsigned long long)sum);
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

int main(void) {
    // connect
    int fd = rw_tcp_connect("127.0.0.1", 12345);
    if (fd < 0) die("rw_tcp_connect");

    // HELLO
    if (rw_send_msg(fd, RW_MSG_HELLO, NULL, 0) < 0) die("rw_send_msg(HELLO)");

    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr(HELLO_ACK)");
    if (recv_error_if_any(fd, type, len)) { close(fd); return 1; }

    if (type != RW_MSG_HELLO_ACK || len != sizeof(rw_hello_ack_t)) {
        fprintf(stderr, "client: expected HELLO_ACK, got type=%u len=%u\n", type, len);
        skip_payload(fd, len);
        close(fd);
        return 1;
    }

    rw_hello_ack_t hello;
    if (rw_recv_all(fd, &hello, sizeof(hello)) < 0) die("rw_recv_all(hello)");
    printf("client: connected client_id=%u\n", hello.client_id);

    // MENU (simple: just create once)
    rw_create_sim_req_t req;
    if (!build_create_req_from_input(&req)) {
        fprintf(stderr, "client: invalid input, exiting.\n");
        close(fd);
        return 1;
    }

    // CREATE_SIM
    if (rw_send_msg(fd, RW_MSG_CREATE_SIM, &req, (uint16_t)sizeof(req)) < 0)
        die("rw_send_msg(CREATE_SIM)");

    // CREATE_ACK
    if (rw_recv_hdr(fd, &type, &len) < 0) die("rw_recv_hdr(CREATE_ACK)");
    if (recv_error_if_any(fd, type, len)) { close(fd); return 1; }

    if (type != RW_MSG_CREATE_ACK || len != sizeof(rw_create_ack_t)) {
        fprintf(stderr, "client: expected CREATE_ACK, got type=%u len=%u\n", type, len);
        skip_payload(fd, len);
        close(fd);
        return 1;
    }

    rw_create_ack_t ack;
    if (rw_recv_all(fd, &ack, sizeof(ack)) < 0) die("rw_recv_all(create_ack)");
    printf("client: CREATE_ACK ok=%u sim_id=%u\n", ack.ok, ack.sim_id);
    if (!ack.ok) { close(fd); return 1; }

    // STATE loop
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
            fprintf(stderr, "client: unexpected msg type=%u len=%u (skipping)\n", type, len);
            skip_payload(fd, len);
        }
    }

    close(fd);
    return 0;
}

