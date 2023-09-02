/*
** Virtual Field Histogram
**
**
*/

#include <stdlib.h>
#include <math.h>
#include "vfh.h"
#include "..\Helper\helper.h"

double calculate_avoidance_angle(histogram * hist, grid * map, int * candidate_lst, double pos_x, double pos_y, double pos_yaw, double goal_x, double goal_y) {
  /* Retrieves the angle that the robot must drive towards. */
  int s_max = 18;
  int valley_threshold = 3; // NEEDS TO BE ADJUSTED
  int lst_length = sizeof(candidate_lst) / sizeof(int);

  double goal_angle = calculate_goal_angle(pos_x, pos_y, pos_yaw, goal_x, goal_y);
  int goal_sector = round(goal_angle / hist->alpha);
  int abs_min = 1000;
  int k_n, k_f;

  for (int i = 0; i < lst_length; i++) {
    int idx = candidate_lst[i];

    // Calculating the minimum distance between the goal sector and the candidate valley
    int min_distance;
    if (abs(idx - goal_sector) < abs(abs(idx - goal_sector) - hist->sectors)) {
      min_distance = abs(idx - goal_sector);
    } else {
      min_distance = (abs(idx - goal_sector) - hist->sectors);
    }

    if (min_distance < abs_min) {
      abs_min = min_distance;
      k_n = idx;
    }
  }

  if (k_n == goal_sector) {
    return goal_sector * hist->alpha;

  } else if (k_n > goal_sector) {
    // See how big the valley is and then select the middle
    for (int i=1; i < s_max; i++) {
      if ( (hist->densities[(k_n + i) % hist->sectors] < valley_threshold) && ( ((k_n + i) % hist->sectors) * hist->alpha) <= M_PI) {
        k_f = k_n + i;
      } else {
        break;
      }
    }

    return ((k_n + k_f) / 2 % hist->sectors) * hist->alpha;

  } else {
    // See how big the valley is and then select the middle
    for (int i=1; i < s_max; i++) {
      if ( (hist->densities[(k_n + i) % hist->sectors] < valley_threshold) && ( ((k_n + i) % hist->sectors) * hist->alpha) <= (3 * M_PI / 4)) {
        k_f = k_n - i;
      } else {
        break;
      }
    }

    return ((k_n + k_f) / 2 % hist->sectors) * hist->alpha;
  }
}

double velocity_control(histogram * hist, double direction) {
  // Max velocity
  int V_MAX = 10;

  // Convert the direction of travel into sector index
  int h_idx = floor(direction / hist->alpha);

  // Retrieve polar histogram density at this sector
  int h_c = hist->densities[h_idx];

  // NOTE: If h_c > 0, that indicates that an obstacles lies ahead of the robot

  // Define h_m which is an empirically determined constant. h_cc later must be less than h_m
  int h_m = 123;

  int h_cc;
  if (h_c > h_m) {
    h_cc = h_m;
  } else {
    h_cc = h_c;
  }
  double speed_reduction = V_MAX * (1 - (h_cc / h_m));

  // Further reduce speed to anticipate obstacles
  
  return speed_reduction;
}
