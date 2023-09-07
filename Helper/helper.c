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

  return goal_angle;
  
}