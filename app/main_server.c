// app/main_server.c (multi-client via poll)
#include "common/socket.h"
#include "common/protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <poll.h>
#include <time.h>

#define MAX_CLIENTS 16
#define TICK_MS 200

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

static inline uint32_t idx(uint32_t x, uint32_t y) { return y * RW_MAX_W + x; }

// ---- Random walk core (wrap) ----
static int pick_dir(uint32_t p_up, uint32_t p_down, uint32_t p_left, uint32_t p_right, unsigned int *seed) {
    uint32_t r = (uint32_t)(rand_r(seed) % RW_PROB_SCALE);
    uint32_t c = 0;
    c += p_up;   if (r < c) return 0;
    c += p_down; if (r < c) return 1;
    c += p_left; if (r < c) return 2;
    (void)p_right;
    return 3;
}

static void step_wrap(uint32_t w, uint32_t h, int *x, int *y, int dir) {
    int nx = *x, ny = *y;
    switch (dir) {
        case 0: ny -= 1; break;
        case 1: ny += 1; break;
        case 2: nx -= 1; break;
        case 3: nx += 1; break;
        default: break;
    }
    if (w > 0) {
        if (nx < 0) nx += (int)w;
        if (nx >= (int)w) nx -= (int)w;
    }
    if (h > 0) {
        if (ny < 0) ny += (int)h;
        if (ny >= (int)h) ny -= (int)h;
    }
    *x = nx; *y = ny;
}

// ---- Simulation state (one global sim) ----
typedef struct {
    int created;

    // config
    uint32_t w, h, K, rep_total;
    uint32_t p_up, p_down, p_left, p_right;
    unsigned int rng_seed;

    // global control
    rw_global_mode_t mode_global;

    // progress in summary sense: completed full-grid replications
    uint32_t rep_done;

    // accumulators
    uint64_t steps_sum[RW_MAX_W * RW_MAX_H];
    uint32_t hit_k_count[RW_MAX_W * RW_MAX_H];

    // current replication scanning
    uint32_t cur_cell_x, cur_cell_y;
    int tx, ty;
    uint32_t t_steps;
    int traj_active;

    // last path for interactive
    uint32_t path_len;
    int16_t path_x[RW_MAX_PATH];
    int16_t path_y[RW_MAX_PATH];

    // creator id
    uint32_t creator_id;

    char out_file[RW_PATH_MAX];
    int stop_requested;
    int results_written;
} sim_t;

static void sim_init(sim_t *S, const rw_create_sim_req_t *req, uint32_t creator_id) {
    memset(S, 0, sizeof(*S));
    S->created = 1;

    S->w = req->w; S->h = req->h; S->K = req->K; S->rep_total = req->rep_total;
    S->p_up = req->p_up; S->p_down = req->p_down; S->p_left = req->p_left; S->p_right = req->p_right;
    S->rng_seed = (unsigned int)time(NULL) ^ (unsigned int)getpid();
    S->mode_global = req->initial_mode;

    S->rep_done = 0;
    S->cur_cell_x = 0; S->cur_cell_y = 0;
    S->traj_active = 0;
    S->path_len = 0;

    S->creator_id = creator_id;

    snprintf(S->out_file, sizeof(S->out_file), "%s", req->out_file);
    S->stop_requested = 0;
    S->results_written = 0;
}

static void start_traj(sim_t *S) {
    S->tx = (int)S->cur_cell_x;
    S->ty = (int)S->cur_cell_y;
    S->t_steps = 0;
    S->traj_active = 1;

    S->path_len = 0;
    if (S->path_len < RW_MAX_PATH) {
        S->path_x[S->path_len] = (int16_t)S->tx;
        S->path_y[S->path_len] = (int16_t)S->ty;
        S->path_len++;
    }
}

static void finish_traj_advance(sim_t *S, uint32_t steps_to_hit, int hit_within_k) {
    uint32_t i = idx(S->cur_cell_x, S->cur_cell_y);
    S->steps_sum[i] += steps_to_hit;
    if (hit_within_k) S->hit_k_count[i]++;

    // advance cell
    S->cur_cell_x++;
    if (S->cur_cell_x >= S->w) {
        S->cur_cell_x = 0;
        S->cur_cell_y++;
        if (S->cur_cell_y >= S->h) {
            S->cur_cell_y = 0;
            S->rep_done++; // one full-grid replication finished
        }
    }

    S->traj_active = 0;
}

static void sim_do_steps(sim_t *S, uint32_t budget) {
    if (!S->created) return;
    if (S->rep_done >= S->rep_total) return;

    if (!S->traj_active) start_traj(S);

    // if already at [0,0]
    if (S->tx == 0 && S->ty == 0) {
        finish_traj_advance(S, 0, 1);
        return;
    }

    for (uint32_t n = 0; n < budget; n++) {
        int dir = pick_dir(S->p_up, S->p_down, S->p_left, S->p_right, &S->rng_seed);
        step_wrap(S->w, S->h, &S->tx, &S->ty, dir);
        S->t_steps++;

        if (S->path_len < RW_MAX_PATH) {
            S->path_x[S->path_len] = (int16_t)S->tx;
            S->path_y[S->path_len] = (int16_t)S->ty;
            S->path_len++;
        }

        if (S->tx == 0 && S->ty == 0) {
            int hitK = (S->t_steps <= S->K) ? 1 : 0;
            finish_traj_advance(S, S->t_steps, hitK);
            return;
        }
    }
}

// build state for a given view
static void build_state_for_view(sim_t *S, rw_local_view_t view, rw_state_msg_t *st_out) {
    rw_state_msg_t st;
    memset(&st, 0, sizeof(st));

    st.w = S->w;
    st.h = S->h;
    st.rep_done = S->rep_done;
    st.rep_total = S->rep_total;
    st.mode = S->mode_global;
    st.finished = (S->rep_done >= S->rep_total) ? 1u : 0u;

    // interactive path is same for everyone (last/ongoing traj)
    if (S->mode_global == RW_MODE_INTERACTIVE) {
        st.path_len = S->path_len;
        for (uint32_t i = 0; i < st.path_len; i++) {
            st.path_x[i] = S->path_x[i];
            st.path_y[i] = S->path_y[i];
        }
    } else {
        st.path_len = 0;
    }

    // obstacles none now
    for (uint32_t y = 0; y < st.h; y++) {
        for (uint32_t x = 0; x < st.w; x++) {
            st.obstacle[idx(x, y)] = 0;
        }
    }

    if (S->rep_done == 0) {
        // nothing yet
        for (uint32_t y = 0; y < st.h; y++)
            for (uint32_t x = 0; x < st.w; x++)
                st.cell_value[idx(x, y)] = 0;
    } else if (view == RW_VIEW_AVG_STEPS) {
        for (uint32_t y = 0; y < st.h; y++) {
            for (uint32_t x = 0; x < st.w; x++) {
                uint32_t i = idx(x, y);
                uint64_t avg = S->steps_sum[i] / (uint64_t)S->rep_done;
                st.cell_value[i] = (uint32_t)(avg * 1000ULL);
            }
        }
    } else {
        for (uint32_t y = 0; y < st.h; y++) {
            for (uint32_t x = 0; x < st.w; x++) {
                uint32_t i = idx(x, y);
                uint32_t prob = (uint32_t)((uint64_t)S->hit_k_count[i] * RW_PROB_SCALE / S->rep_done);
                st.cell_value[i] = prob;
            }
        }
    }

    *st_out = st;
}


static int write_results_to_file(sim_t *S) {
    if (!S->created) return -1;
    if (S->results_written) return 0;

    FILE *f = fopen(S->out_file, "w");
    if (!f) return -1;

    // Header
    fprintf(f, "# Random Walk results\n");
    fprintf(f, "# w=%u h=%u K=%u rep_done=%u rep_total=%u\n",
            S->w, S->h, S->K, S->rep_done, S->rep_total);
    fprintf(f, "# Prob scale: %u\n\n", (unsigned)RW_PROB_SCALE);

    // AVG_STEPS (plain steps, not *1000)
    fprintf(f, "[AVG_STEPS]\n");
    for (uint32_t y = 0; y < S->h; y++) {
        for (uint32_t x = 0; x < S->w; x++) {
            uint32_t i = idx(x, y);
            double avg = 0.0;
            if (S->rep_done > 0) {
                avg = (double)S->steps_sum[i] / (double)S->rep_done;
            }
            fprintf(f, "%.3f%s", avg, (x + 1 == S->w) ? "" : " ");
        }
        fprintf(f, "\n");
    }

    // PROB_K (0..1)
    fprintf(f, "\n[PROB_K]\n");
    for (uint32_t y = 0; y < S->h; y++) {
        for (uint32_t x = 0; x < S->w; x++) {
            uint32_t i = idx(x, y);
            double p = 0.0;
            if (S->rep_done > 0) {
                p = (double)S->hit_k_count[i] / (double)S->rep_done;
            }
            fprintf(f, "%.6f%s", p, (x + 1 == S->w) ? "" : " ");
        }
        fprintf(f, "\n");
    }

    fclose(f);
    S->results_written = 1;
    return 0;
}


// ---- Clients ----
typedef struct {
    int active;
    int fd;
    uint32_t client_id;
    int hello_done;
    int joined;
    rw_local_view_t view;
} client_t;

static void client_close(client_t *c) {
    if (c->active) close(c->fd);
    memset(c, 0, sizeof(*c));
}

static int validate_create(const rw_create_sim_req_t *r) {
    if (r->w == 0 || r->h == 0) return 0;
    if (r->w > RW_MAX_W || r->h > RW_MAX_H) return 0;
    uint64_t sum = (uint64_t)r->p_up + r->p_down + r->p_left + r->p_right;
    if (sum != RW_PROB_SCALE) return 0;
    if (r->rep_total == 0 || r->K == 0) return 0;
    if (r->initial_mode != RW_MODE_INTERACTIVE && r->initial_mode != RW_MODE_SUMMARY) return 0;
    return 1;
}

static int handle_one_msg(client_t *c, sim_t *S) {
    uint16_t type = 0, len = 0;
    if (rw_recv_hdr(c->fd, &type, &len) < 0) return -1;

    // HELLO
    if (type == RW_MSG_HELLO && len == 0) {
      if (c->hello_done) return 0;
      c->hello_done = 1;        

      rw_hello_ack_t ack = {.client_id = c->client_id};
      if (rw_send_msg(c->fd, RW_MSG_HELLO_ACK, &ack, (uint16_t)sizeof(ack)) < 0) return -1;
      
      rw_error_msg_t info;
      memset(&info.msg, 0, sizeof(info.msg));
      info.code = 0;

      if (!S->created) {
      snprintf(info.msg, sizeof(info.msg), "No active simulation, needs to be created!");
      } else {
        snprintf(info.msg, sizeof(info.msg), "Simulation is running. Can be joined!");
      }

      if (rw_send_msg(c->fd, RW_MSG_ERROR, &info, (uint16_t)sizeof(info)) < 0 ) return -1;
        
      return 0;
    }

    
    // CREATE_SIM (only if sim not created yet)
    if (type == RW_MSG_CREATE_SIM && len == sizeof(rw_create_sim_req_t)) {
        rw_create_sim_req_t req;
        if (rw_recv_all(c->fd, &req, sizeof(req)) < 0) return -1;

        if (S->created) {
            send_error(c->fd, 20, "Simulation already created; use JOIN_SIM");
            rw_create_ack_t nack = {.ok = 0, .sim_id = 0};
            (void)rw_send_msg(c->fd, RW_MSG_CREATE_ACK, &nack, (uint16_t)sizeof(nack));
            return 0;
        }
        if (!validate_create(&req)) {
            send_error(c->fd, 21, "CREATE_SIM validation failed");
            rw_create_ack_t nack = {.ok = 0, .sim_id = 0};
            (void)rw_send_msg(c->fd, RW_MSG_CREATE_ACK, &nack, (uint16_t)sizeof(nack));
            return 0;
        }

        sim_init(S, &req, c->client_id);
        c->joined = 1;               // creator auto-joins
        c->view = RW_VIEW_AVG_STEPS;

        rw_create_ack_t ack = {.ok = 1, .sim_id = 1};
        if (rw_send_msg(c->fd, RW_MSG_CREATE_ACK, &ack, (uint16_t)sizeof(ack)) < 0) return -1;

        printf("server: sim created by client_id=%u\n", c->client_id);
        return 0;
    }

    // JOIN_SIM
    if (type == RW_MSG_JOIN_SIM && len == sizeof(rw_join_req_t)) {
        rw_join_req_t jr;
        if (rw_recv_all(c->fd, &jr, sizeof(jr)) < 0) return -1;

        if (!S->created) {
            send_error(c->fd, 30, "No simulation yet; wait for creator to CREATE_SIM");
            rw_join_ack_t nack;
            memset(&nack, 0, sizeof(nack));
            nack.ok = 0;
            (void)rw_send_msg(c->fd, RW_MSG_JOIN_ACK, &nack, (uint16_t)sizeof(nack));
            return 0;
        }

        c->joined = 1;
        c->view = RW_VIEW_AVG_STEPS;

        rw_join_ack_t ack;
        memset(&ack, 0, sizeof(ack));
        ack.ok = 1;
        ack.w = S->w; ack.h = S->h; ack.rep_total = S->rep_total; ack.K = S->K;
        ack.world_type = RW_WORLD_WRAP;
        ack.mode_now = S->mode_global;
        ack.rep_done = S->rep_done;
        if (rw_send_msg(c->fd, RW_MSG_JOIN_ACK, &ack, (uint16_t)sizeof(ack)) < 0) return -1;

        printf("server: client_id=%u joined\n", c->client_id);
        return 0;
    }

    // SET_MODE
    if (type == RW_MSG_SET_MODE && len == sizeof(rw_set_mode_req_t)) {
        rw_set_mode_req_t sm;
        if (rw_recv_all(c->fd, &sm, sizeof(sm)) < 0) return -1;

        if (!S->created) { send_error(c->fd, 40, "No simulation yet"); return 0; }
        //if (c->client_id != S->creator_id) { send_error(c->fd, 41, "Only creator may SET_MODE"); return 0; }

        if (sm.mode != RW_MODE_INTERACTIVE && sm.mode != RW_MODE_SUMMARY) {
            send_error(c->fd, 42, "Invalid mode");
            return 0;
        }
        S->mode_global = sm.mode;
        printf("server: SET_MODE -> %u (by creator %u)\n", (unsigned)S->mode_global, S->creator_id);
        return 0;
    }

    //STOP_SIM (creator only)
    if (type == RW_MSG_STOP_SIM && len == sizeof(rw_stop_req_t)) {
    rw_stop_req_t sr;
    if (rw_recv_all(c->fd, &sr, sizeof(sr)) < 0) return -1;

    if (!S->created) { send_error(c->fd, 70, "No simulation yet"); return 0; }
    if (c->client_id != S->creator_id) { send_error(c->fd, 71, "Only creator may STOP_SIM"); return 0; }

    S->stop_requested = 1;
    printf("server: STOP_SIM requested by creator=%u (reason=%u)\n",
           S->creator_id, sr.reason);
    return 0;
}

    // SET_VIEW (any joined client)
    if (type == RW_MSG_SET_VIEW && len == sizeof(rw_set_view_req_t)) {
        rw_set_view_req_t sv;
        if (rw_recv_all(c->fd, &sv, sizeof(sv)) < 0) return -1;

        if (sv.view != RW_VIEW_AVG_STEPS && sv.view != RW_VIEW_PROB_K) {
            send_error(c->fd, 50, "Invalid view");
            return 0;
        }
        c->view = sv.view;
        return 0;
    }

    // unknown -> skip payload to keep stream aligned
    if (len) skip_payload(c->fd, len);
    return 0;
}

int main(void) {
    const uint16_t port = 12345;
    int listen_fd = rw_tcp_listen(NULL, port, 16);
    if (listen_fd < 0) die("rw_tcp_listen");

    printf("server: listening on %u...\n", (unsigned)port);

    client_t clients[MAX_CLIENTS];
    memset(clients, 0, sizeof(clients));
    uint32_t next_id = 1;

    sim_t sim;
    memset(&sim, 0, sizeof(sim));

    while (1) {
        // build pollfds: [listen] + active clients
        struct pollfd pfds[1 + MAX_CLIENTS];
        int map_idx[1 + MAX_CLIENTS]; // poll index -> client index, -1 for listen
        int nfds = 0;

        pfds[nfds].fd = listen_fd;
        pfds[nfds].events = POLLIN;
        map_idx[nfds] = -1;
        nfds++;

        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (!clients[i].active) continue;
            pfds[nfds].fd = clients[i].fd;
            pfds[nfds].events = POLLIN;
            map_idx[nfds] = i;
            nfds++;
        }

        int rc = poll(pfds, nfds, TICK_MS);
        if (rc < 0) {
            if (errno == EINTR) continue;
            die("poll");
        }

        // 1) new connections
        if (pfds[0].revents & POLLIN) {
            int cfd = accept(listen_fd, NULL, NULL);
            if (cfd >= 0) {
                int slot = -1;
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (!clients[i].active) { slot = i; break; }
                }
                if (slot < 0) {
                    send_error(cfd, 60, "Server full");
                    close(cfd);
                } else {
                    clients[slot].active = 1;
                    clients[slot].fd = cfd;
                    clients[slot].client_id = next_id++;
                    clients[slot].hello_done = 0;
                    clients[slot].joined = 0;
                    clients[slot].view = RW_VIEW_AVG_STEPS;
                    printf("server: accepted client slot=%d id=%u\n", slot, clients[slot].client_id);
                }
            }
        }

        // 2) handle incoming messages
        for (int pi = 1; pi < nfds; pi++) {
            int ci = map_idx[pi];
            if (ci < 0) continue;
            if (pfds[pi].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                printf("server: client %u disconnected\n", clients[ci].client_id);
                client_close(&clients[ci]);
                continue;
            }
            if (pfds[pi].revents & POLLIN) {
                if (handle_one_msg(&clients[ci], &sim) < 0) {
                    printf("server: client %u read error/disconnect\n", clients[ci].client_id);
                    client_close(&clients[ci]);
                }
            }
        }

        // 3) tick simulation + (optional) finish + broadcast state

        // run simulation only if not stopped and not finished
        if (sim.created && !sim.stop_requested && sim.rep_done < sim.rep_total) {
          uint32_t budget = (sim.mode_global == RW_MODE_INTERACTIVE) ? 1u : 500u;
          sim_do_steps(&sim, budget);
        }

        // determine finished state
        int finished_now = 0;
        if (sim.created) {
          if (sim.stop_requested) finished_now = 1;
          if (sim.rep_done >= sim.rep_total) finished_now = 1;
        }

        // if finished for the first time -> write results once
        if (sim.created && finished_now && !sim.results_written) {
          if (write_results_to_file(&sim) == 0) {
          printf("server: results saved to %s\n", sim.out_file);
        } else {
          perror("server: write_results_to_file");
          // aj keď uloženie zlyhá, simuláciu aj tak ukončíme (finished=1)
          sim.results_written = 1; // aby sa to nepokúšalo zapisovať furt dokola
        }
        }

        // broadcast to all joined clients
        if (sim.created) {
          for (int i = 0; i < MAX_CLIENTS; i++) {
            if (!clients[i].active || !clients[i].joined) continue;

            rw_state_msg_t st;
            build_state_for_view(&sim, clients[i].view, &st);

            // NOTE: st.finished už bude 1, ak stop_requested alebo rep_done>=rep_total
            if (rw_send_msg(clients[i].fd, RW_MSG_STATE, &st, (uint16_t)sizeof(st)) < 0) {
              printf("server: drop client %u (send failed)\n", clients[i].client_id);
              client_close(&clients[i]);
             }
          }
      }
  }

    close(listen_fd);
    return 0;
}

