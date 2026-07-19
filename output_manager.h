#ifndef OUTPUT_MANAGER_H
#define OUTPUT_MANAGER_H

#include "CoreSLAM.h"

void set_map(ts_map_t *map);
ts_map_t *get_map(void);

void set_position(ts_position_t position);
ts_position_t get_position(void);

#endif
