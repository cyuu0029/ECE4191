/*
** Helper Functions
**
** Host all generic helper functions
*/

/*===========================================================================*/
#include <stdlib.h>
#include <math.h>
#include "helper.h"

long double calculate_angle_modulo(long double angle) {
    return angle - M_TWOPI * floor(angle / M_TWOPI);  
}

double calculate_distance_from_goal(double pos_x, double pos_y, double goal_x, double goal_y) {
  // Calculate distance between current position and end goal
  double distance = sqrt( pow( goal_x - pos_x, 2 ) + pow( goal_y - pos_y, 2 ) ); 

  return distance;
}

double calculate_goal_angle(double pos_x, double pos_y, double pos_yaw, double goal_x, double goal_y) {
  // Calculate the angle from current position to goal
  double goal_angle = atan2(goal_y - pos_y, goal_x - pos_x);

  // Calculate robot rotation to point towards goal with respect to current heading
  double rotation;
  double alpha_rot = goal_angle - pos_yaw;
  double beta_rot = alpha_rot + 2 * M_PI;
  double gamma_rot = alpha_rot - 2 * M_PI;

  // Find shortest rotation required
  if (alpha_rot < beta_rot && alpha_rot < gamma_rot) {
    rotation = alpha_rot;
  } else if (beta_rot < gamma_rot) {
    rotation = beta_rot;
  } else {
    rotation = gamma_rot;
  }

  if (rotation < 0) {
    rotation = rotation + 2 * M_PI;
  }

  return rotation;
  
}

int detect_obstacle(double angle, double distance, double pos_x, double pos_y, double pos_theta, double threshold) {
    // Checks if detected item is obstacle or wall
    int obstacles;
    double rad = M_PI / 180;
    
    // Distance between wall and robot based on robot orientation
    double max_distance;;
    
    // Find if obstacle in specified angle
    double angle_modulo;
    switch ((int) angle) {
        case 0:
            max_distance = (ARENA - pos_x) / cos(pos_theta) - threshold;
            break;
            
        case 90:
            angle_modulo = calculate_angle_modulo(M_PI / 2 + pos_theta);
            max_distance = (ARENA - pos_y) / sin(angle_modulo) - threshold;
            break;
            
            
        case 270:
            angle_modulo = calculate_angle_modulo(3*M_PI / 2 + pos_theta);
            max_distance = (pos_y) / sin(angle_modulo) - threshold;
            break;
            
        default:
            angle_modulo = calculate_angle_modulo(angle * rad + pos_theta);
            max_distance = pos_x / cos(angle_modulo) - threshold;
    }
    
    if (distance < max_distance) {
        obstacles = 1;
    } else {
        obstacles = 0;
    }
    

  return obstacles;
}