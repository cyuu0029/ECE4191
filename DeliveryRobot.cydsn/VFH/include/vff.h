#ifndef VFF_H
#define VFF_H
    
#include "polar_histogram.h"
    int vff_local(grid * active_grid, float objective_direction);
    
    int vff_global(histogram * certainty_grid, int curr_x, int curr_y);
    
    
    
#endif