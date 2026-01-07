#include <stdint.h>
#include "types.h"

#ifndef PROTOCOL_H
#define PROTOCOL_H

// ---- Message types ----
typedef enum {
    RW_MSG_HELLO = 1,       // client -> server (no payload)
    RW_MSG_HELLO_ACK,       // server -> client (hello_ack_t)

    RW_MSG_CREATE_SIM,      // client -> server (create_sim_req_t)
    RW_MSG_CREATE_ACK,      // server -> client (create_ack_t)

    RW_MSG_JOIN_SIM,        // client -> server (join_req_t)
    RW_MSG_JOIN_ACK,        // server -> client (join_ack_t)

    RW_MSG_SET_MODE,        // client -> server (set_mode_req_t)  [global for all]
    RW_MSG_SET_VIEW,        // client -> server (set_view_req_t)  [local per client]

    RW_MSG_STOP_SIM,        // client -> server (stop_req_t)

    RW_MSG_STATE,           // server -> client (state_msg_t) streamed periodically

    RW_MSG_ERROR            // either direction (error_msg_t)
} rw_msg_type_t;

// ---- Common header ----
// Note: length is payload length (bytes), not including header.
typedef struct {
    uint16_t type;    // rw_msg_type_t
    uint16_t length;  // payload bytes
} rw_msg_hdr_t;

// ---- HELLO ----
typedef struct {
    uint32_t client_id; // assigned by server, non-zero
} rw_hello_ack_t;

// ---- CREATE SIM ----
typedef struct {
    uint32_t w;                // <= RW_MAX_W
    uint32_t h;                // <= RW_MAX_H
    uint32_t rep_total;        // total replications to run
    uint32_t K;                // max steps per trajectory for prob_K

    // probabilities in fixed-point, sum must be RW_PROB_SCALE
    uint32_t p_up;
    uint32_t p_down;
    uint32_t p_left;
    uint32_t p_right;

    rw_world_type_t world_type;
    rw_global_mode_t initial_mode;

    // For simplicity: obstacles generated randomly on server (if world_type == OBSTACLES)
    uint32_t obstacle_density_permille; // 0..1000 (e.g., 200 = 20%)

    // output file where server stores result after finish
    char out_file[RW_PATH_MAX];
} rw_create_sim_req_t;

typedef struct {
    uint32_t ok;       // 1 ok, 0 fail
    uint32_t sim_id;   // server-assigned id (can be 1 if single sim)
} rw_create_ack_t;

// ---- JOIN SIM ----
typedef struct {
    uint32_t sim_id; // which simulation to join; if you run one sim per server use 1
} rw_join_req_t;

typedef struct {
    uint32_t ok;

    // current sim config so client can render correctly
    uint32_t w;
    uint32_t h;
    uint32_t rep_total;
    uint32_t K;

    rw_world_type_t world_type;
    rw_global_mode_t mode_now;

    uint32_t rep_done;
} rw_join_ack_t;

// ---- SET MODE (global for all clients) ----
typedef struct {
    rw_global_mode_t mode;
} rw_set_mode_req_t;

// ---- SET VIEW (local per-client in summary) ----
typedef struct {
    rw_local_view_t view;
} rw_set_view_req_t;

// ---- STOP SIM ----
typedef struct {
    uint32_t reason; // 0=unspecified, 1=user_stop
} rw_stop_req_t;

// ---- STATE (server -> client) ----
// A single bounded message carrying everything client needs to render.
typedef struct {
    // progress
    uint32_t rep_done;
    uint32_t rep_total;

    // global mode
    rw_global_mode_t mode;

    // dimensions (redundant but convenient)
    uint32_t w;
    uint32_t h;

    // 0 = running, 1 = finished
    uint32_t finished;

    // INTERACTIVE: last path (up to RW_MAX_PATH)
    // server may send path_len=0 when in summary mode
    uint32_t path_len; // 0..RW_MAX_PATH
    int16_t  path_x[RW_MAX_PATH];
    int16_t  path_y[RW_MAX_PATH];

    // WORLD: obstacles bitmap for rendering (0/1), only first w*h used
    uint8_t  obstacle[RW_MAX_W * RW_MAX_H];

    // SUMMARY: cell values, only first w*h used.
    // Meaning depends on the client's local view (AVG_STEPS or PROB_K).
    // Convention:
    // - AVG_STEPS: value = avg_steps * 1000 (fixed-point)
    // - PROB_K:    value = probability * RW_PROB_SCALE (0..RW_PROB_SCALE)
    uint32_t cell_value[RW_MAX_W * RW_MAX_H];
} rw_state_msg_t;

// ---- ERROR ----
typedef struct {
    int32_t code;          // your internal error codes
    char msg[96];          // short human-readable message
} rw_error_msg_t;

#endif

