#include <stdlib.h>
#include <math.h>

#include "histogram_grid.h"
#include "polar_histogram.h"

#include "vff.h"

int vff_local(grid * active_grid, float objective_direction){
    double vector[2] = {0,0}; //sum x and y vectors
    double distance = 0; //distance from robot position to cell
    for(int i = 0; i < active_grid->height; ++i){
        for(int j = 0; j < active_grid->width; ++j){
                distance = sqrt(pow(i - ((active_grid->height - 1)/2),2)+ pow(j - ((active_grid->width - 1)/2),2));
                vector[0] += active_grid->cells[i* active_grid->width + j]*(i - ((active_grid->height - 1)/2))/distance/distance ;
                vector[1] += active_grid->cells[i* active_grid->width + j]*(j - ((active_grid->height - 1)/2))/distance/distance;
        };
    };
    
    vector[0] += 10*cos(objective_direction);
    vector[1] += 10*sin(objective_direction);
    
    return atan2(vector[1], vector[0]);
    
};

int vff_global(histogram * certainty_grid, int curr_x, int curr_y);