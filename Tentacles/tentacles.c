#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "..\Tentacles\tentacles.h"

bool check_collision(Sensor * sensors) {
    // Calculate whether or not the robot collides with obstacle based on ultrasonic reading
    int front_threshold = 10;
    float other_threshold = 0.1;
    for (int i=0; i < N_SENSORS; i++) {
        if (sensors->direction[i] < M_PI || sensors->direction[i] > 3*M_PI / 4) {
            if (sensors->distance[i] < front_threshold) {
                return true;
            }
        } else {
            if (sensors->distance[i] < other_threshold) {
                return true;
            }
        }
        
    }

    return false;
}


double tentacles_cost_function(Tentacles * octopussy, Sensor * sensors, double v, double w, double goal_x, double goal_y, double goal_th, double pos_x, double pos_y, double pos_theta) {

    // Extracting variables
    double dt = octopussy->dt;
    double steps = octopussy->steps;
    double alpha = octopussy->alpha;
    double beta = octopussy->beta;

    // Define variables

    // Allocate space for different trajectories
    for (int i=0; i < steps; i++) {
        pos_x = pos_x + dt * v * cos(pos_theta);
        pos_y = pos_y + dt * v * sin(pos_theta);
        pos_theta = pos_theta + w * dt;

        if (check_collision(sensors)) {
            return 10000000000;
        }
    }

    // Wrap angle error -pi, pi
    double e_th = goal_th - pos_theta;
    e_th = atan2(sin(e_th), cos(e_th));

    // Calculate cost
    double cost = alpha * ( pow(goal_x - pos_x, 2) + pow(goal_y - pos_y, 2)) + beta * (e_th);
    return cost;
}

/*
double * planner(Tentacles * octopussy, Sensor * sensors, int goal_x, int goal_y, int goal_th, int pos_x, int pos_y, int pos_theta) {
    // Store cost values
    double cost[octopussy->ten_count];

    // Tentacles oooooh lala
    int t_c = 0;
    double tentacles_bby = octopussy->ten;
    int min_combo;
    double min_cost = 1000000;

    for (int i = 0; i < octopussy->ten_count; i++) {
        cost[i] = cost_function(octopussy, sensors, tentacles_bby[t_c], tentacles_bby[t_c++], goal_x, goal_y, goal_th, pos_x, pos_y, pos_theta);
        if (cost[i] < min_cost) {
            min_cost = cost[i];
            min_combo = t_c;
        }
        t_c += 2;
    }

    // Identify best tentacle SHEEEEEEEEEESH
    return [octopussy->ten[min_combo], octopussy->ten[min_combo++]];
}
*/