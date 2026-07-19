#include "output_manager.h"

static ts_position_t position_output = {0};
static ts_map_t map_output = {0};

void set_map(ts_map_t map) {
    map_output = map;
}

ts_map_t get_map(void) {
    return map_output;
}

void set_position(ts_position_t position) {
    position_output = position;
}

ts_position_t get_position(void) {
    return position_output;
}
