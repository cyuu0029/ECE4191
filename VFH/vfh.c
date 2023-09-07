/*
** Vector Field Histogram Implementation
**  
**
*/

#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "vfh.h"

/*=======================================================================*/    
// Histogram Grid Creation Code

grid * grid_create(int width, int height, int resolution) {
  /* Creates a map of our arena. */

  grid * map = malloc(sizeof(grid));  // Allocate memory for grid map

  // Return NULL if not enough memory
  if (map == NULL) {
    free(map);
    return NULL;
  }

  // Define variables
  map->width = width;
  map->height = height;
  map->resolution = resolution;
  map->cells = malloc(width * height * sizeof(int));

  // Return NULL if not enough memory
  if (map->cells == NULL) {
    free(map);
    free(map->cells);
    return NULL;
  }

  // Define all initial grid values to be 0 as there are no obstacles
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      map->cells[i * width + j] = 0;    // Map grid is defined as a 1D array
    }
  }

  return map;
}

int grid_update(grid * map, Sensor * sensors, Robot * robot) {
  /*
   * Updates Histogram with detected obstacles.
   * Takes robot positioning and sensor measurements to determine obstacles location.
   *
   * Remember that cos() and sin() expect angles in RADIANS, not DEGREES.
  */

  // Check if grid map exists
  if (map == NULL) return 0;
  if (map->cells == NULL) return 0;

  // Extract needed variables from structure
  double pos_x = robot->x;    // cm
  double pos_y = robot->y;    // cm
  double yaw = robot->theta;  // radians

  // Determine distance of returned ultrasonic signal
  double cell_distance[N_SENSORS];
  for (int i = 0; i < N_SENSORS; ++i) {
    cell_distance[i] = sensors->distance[i] / map->resolution;
  }

  // Determine cell location of obstacles
  double cell_location[N_SENSORS];
  double theta; 
  double sensor_radians;

  for (int i = 0; i < N_SENSORS; ++i){
    // Convert sensor angle into radians
    sensor_radians = M_PI * sensors->direction[i] / 180;
    if (yaw + sensor_radians < 0) {
      theta = yaw + sensor_radians + M_TWOPI;
    } else if (yaw + sensor_radians >= M_TWOPI) {
      theta = yaw + sensor_radians - M_TWOPI;
    } else {
      theta = yaw + sensor_radians;
    }
    
    double new_x = pos_x/map->resolution + floor(cell_distance[i] * cos(theta));
    double new_y = pos_y/map->resolution + floor(cell_distance[i] * sin(theta));
    
    // Check if point is within grid to avoid overflow
    if (new_x < map->width && new_y < map->height && new_x >= 0 && new_y >= 0) {
      map->cells[(int) new_x * map->width + (int) new_y] += 1;
    } 
  }

  return 1;
}

void active_window(grid * map, grid * active, Robot * robot) {
  /*
  * Creates an active window surrounding robot.
  */

  // Robot variables
  int x = robot->x;
  int y = robot->y;
  double theta = robot->y;

  // Create active window based on generated map
  for (int i = 0; i < active->width; ++i) {
      for (int j = 0; j < active->height; ++j) {

        /* x and y are the center coordinates of the body with sensors. */
        int grid_i = i + x/map->resolution - (active->width - 1) / 2;
        int grid_j = j + y/map->resolution - (active->height - 1) / 2;

        /* Copy the information from the grid to the moving window. */
        if (grid_i < map->width && grid_j < map->height && grid_i >= 0 && grid_j >= 0) {
          active->cells[i * active->width + j] = map->cells[grid_i * map->width + grid_j];
        }
      }
    }
}

/*=======================================================================*/
/*                             END OF GRID                               */
/*=======================================================================*/
// Polar Histogram

POD * pod_create(double alpha) {
  POD * pod = malloc(sizeof(POD));  // Allocate memory for grid map

  // Return NULL if not enough memory
  if (pod == NULL) {
    free(pod);
    return NULL;
  }

  // Define variables
  pod->nsectors = 360 / alpha;
  pod->density = malloc((360 / alpha) * sizeof(double));

  // Return NULL if not enough memory
  if (pod->density == NULL) {
    free(pod);
    free(pod->density);
    return NULL;
  }

  // Define all initial grid values to be 0 as there are no obstacles
  for (int i = 0; i < pod->nsectors; ++i) {
    pod->density[i] = 0;    // Map grid is defined as a 1D array
  }

  return pod;
}
void smoothed_POD_histogram(POD * smoothed_POD, grid *active, double alpha, double l) {
  int sectors = smoothed_POD->nsectors;
  
  double POD_hist[72];

  // Create POD histogram
  for (int i=0; i < sectors; i++) {
    int left = alpha * i;
    int right = alpha * (i + 1);
    double sum = 0;

    for (int j=left; j < right; j++) {
      sum += active->cells[j];
    }

    POD_hist[i] = sum;
  }
  
  

  // Smoothing POD histogram
  for (int i=0; i < sectors; i++) {
    double sum_element = POD_hist[i] * l;

    for (int j=0; j < l; j++) {
      sum_element += (POD_hist[i-(j+1)] + POD_hist[(i+(j+1)) % sectors]) * (l - j);
    }

    double smoothed_element = sum_element / ((2 * l) + 1);
    smoothed_POD->density[i] = smoothed_element;
  }

}


/*=======================================================================*/


double calculate_avoidance_angle(POD *smoothed_POD, Robot * robot, int * candidate_lst, double alpha, double s_max, double valley_threshold) {
  /* Retrieves the angle that the robot must drive towards. */
  int candidates_len = sizeof(candidate_lst)/sizeof(int);
  int nsectors = smoothed_POD->nsectors;

  // Retrive useful variables
  double pos_x = robot->x;
  double pos_y = robot->y;
  double pos_yaw = robot->theta;
  double goal_x = robot->goal_x;
  double goal_y = robot->goal_y;

  double goal_angle = calculate_goal_angle(pos_x, pos_y, pos_yaw, goal_x, goal_y);
  int goal_sector = round((180 * goal_angle / M_PI) / nsectors);
  int abs_min = 1000;
  int k_n = -1;
  int k_f;

  // Find angle, note we are working in degrees here
  for (int i = 1; i < candidates_len; i++) {
    int idx = *(candidate_lst + i);

    // Calculating the minimum distance between the goal sector and the candidate valley
    int min_distance;
    if (abs(idx - goal_sector) < abs(abs(idx - goal_sector) - nsectors)) {
      min_distance = abs(idx - goal_sector);
    } else {
      min_distance = (abs(idx - goal_sector) - nsectors);
    }

    if (min_distance < abs_min) {
      if (idx * alpha < 90 || idx * alpha > 270) {
        abs_min = min_distance;
        k_n = idx;
      }
    }
  }

  // If no valleys after filtering
  abs_min = 100000;
  if (k_n == -1) {
    for (int i = 1; i < candidates_len; i++) {
        int idx = *(candidate_lst + i);

        // Calculating the minimum distance between the goal sector and the candidate valley
        int min_distance;
        if (abs(idx - goal_sector) < abs(abs(idx - goal_sector) - nsectors)) {
          min_distance = abs(idx - goal_sector);
        } else {
          min_distance = (abs(idx - goal_sector) - nsectors);
        }

        if (min_distance < abs_min) {
          abs_min = min_distance;
          k_n = idx;
        }
    }
    }

  k_f = k_n;
  
  if (k_n == goal_sector) {
    return goal_sector * alpha;

  } else if (k_n > goal_sector) {
    // See how big the valley is and then select the middle
    for (int i=1; i < s_max + 1; i++) {
      int pod_idx = (k_n + i) % nsectors;
      double pod_val = smoothed_POD->density[pod_idx];
      if ( (pod_val < valley_threshold) && ( pod_idx * alpha) <= 90) {
        k_f = k_n + i;
      } else {
        break;
      }
    }

    return ((k_n + k_f) / 2 % nsectors) * alpha;

  } else {
    // See how big the valley is and then select the middle
    for (int i=1; i < s_max + 1; i++) {
      int pod_idx = (k_n + i) % nsectors;
      double pod_val = smoothed_POD->density[pod_idx];
      if ( (pod_val < valley_threshold) && ( pod_idx * alpha) <= 270) {
        k_f = k_n - i;
      } else {
        break;
      }
    }
}

    return ((k_n + k_f) / 2 % nsectors) * alpha;
  }



double velocity_control(double * smoothed_POD, double direction, double alpha, double h_m) {
  // Max velocity
  double V_MAX = 10;

  // Convert the direction of travel into sector index
  int h_idx = floor((180 * direction / M_PI)  / alpha);

  // Retrieve polar histogram density at this sector
  double h_c = *(smoothed_POD + h_idx);

  // NOTE: If h_c > 0, that indicates that an obstacles lies ahead of the robot

  // Define h_m which is an empirically determined constant. h_cc later must be less than h_m

  double h_cc;
  if (h_c > h_m) {
    h_cc = h_m;
  } else {
    h_cc = h_c;
  }
  double speed_reduction = V_MAX * (1 - (h_cc / h_m));

  // Further reduce speed to anticipate obstacles
  
  return speed_reduction;
}
