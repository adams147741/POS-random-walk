#include <stdint.h>

#ifndef TYPES_H
#define TYPES_H

// --- Limits (keep STATE message bounded) ---
#define RW_MAX_W     60
#define RW_MAX_H     30
#define RW_MAX_PATH  128
#define RW_PATH_MAX  128

// Probabilities are fixed-point in [0 .. RW_PROB_SCALE], sum must be RW_PROB_SCALE
#define RW_PROB_SCALE 1000000u

typedef struct {
    int32_t x;
    int32_t y;
} rw_pos_t;

typedef enum {
    RW_WORLD_WRAP      = 1, // no obstacles (or obstacles ignored), wrap-around edges
    RW_WORLD_OBSTACLES = 2  // obstacles present, no wrap unless you decide otherwise
} rw_world_type_t;

typedef enum {
    RW_MODE_INTERACTIVE = 1,
    RW_MODE_SUMMARY     = 2
} rw_global_mode_t;

typedef enum {
    RW_VIEW_AVG_STEPS = 1,  // average steps to reach [0,0]
    RW_VIEW_PROB_K    = 2   // probability to reach [0,0] within K steps
} rw_local_view_t;

#endif

