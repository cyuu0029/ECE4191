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
  double robot_radians;

  for (int i = 0; i < N_SENSORS; ++i){
    // Convert sensor angle into radians
    robot_radians = M_PI * sensors->direction[i] / 180;
    if (yaw + robot_radians < 0) {
      theta = yaw + robot_radians + M_TWOPI;
    } else if (yaw + robot_radians >= M_TWOPI) {
      theta = yaw + robot_radians - M_TWOPI;
    } else {
      theta = yaw + robot_radians;
    }
    
    double new_x = pos_x/map->resolution + floor(cell_distance[i] * cos(theta));
    double new_y = pos_y/map->resolution + floor(cell_distance[i] * sin(theta));
    
    // Check if point is within grid to avoid overflow
    if (new_x < map->width && new_y < map->height) {
      map->cells[(int) new_x * map->width + (int) new_y] += 1;
    } 
  }

  return 1;
}

double * active_window(double * active, Robot * robot, Sensor * sensors, double a, double) {
  /*
    * Creates an active window surrounding robot.
  */

  double max_dist = a / b;
  double ws = sqrt(2) * max_dist;

  int counter = 0;
  if (active != NULL) {

    // Retrieve sensor values
    for (int i=0; i < N_SENSORS; i++) {
      double reading_x = sensors->distance[i] * cos(sensors->direction[i] * DEG2RAD);
      double reading_y = sensors->distance[i] * sin(sensors->direction[i] * DEG2RAD);

      double max_width = ws / 2;

      if ( max( max(max_width, reading_x), reading_y ) == (int) max_width) {
        double confidence = 1;

        if (max_width != 0) {
          confidence = sensors->distance[i] / (2 * max_width);
          confidence = abs(confidence - 1);
        }
        active[counter] = (confidence ** 2) * (a - (b * sensors->distance[i]));
      
      } else {
        active[counter] = 0;
      }

      counter++;
      counter = counter % 360;

    }

  } else {
    free(active);
    return NULL;
  }

  return active;
}

/*=======================================================================*/
/*                             END OF GRID                               */
/*=======================================================================*/
// Polar Histogram 
histogram * polar_histogram_create(int alpha, double threshold, double density_a, double density_b) {
  /* Create a histogram pointer and allocate memory to it. */
  histogram * hist = malloc(sizeof(histogram));

  /* Is there enough memory for the histogram? */
  if (NULL == hist) {
    free(hist);
    return NULL;
  }

  /* Initialize the histogram parameters. */
  hist->alpha = alpha;
  hist->sectors = 360 / alpha;
  hist->threshold = threshold;
  hist->densities = (int *)malloc(hist->sectors * sizeof(int));
  hist->density_a = density_a;
  hist->density_b = density_b;

  if (hist->densities == NULL) {
    free(hist);
    return NULL;
  }

  /* Initialize all densities to 0. */
  for (int i = 0; i < hist->sectors; ++i) {
    hist->densities[i] = 0;
  }

  return hist;
}

double * smoothed_POD_histogram(double * POD_hist, double * active, double alpha, double l) {

  double nsectors = 360 / alpha;

  // Calculate sum of active window
  double active_sum = 0;
  for (int i=0; i < 360; i++) {
    active_sum = active_sum + active[i];
  }

  // Create POD histogram
  for (int i=0; i < nsectors; i++) {
    int left = alpha * i;
    int right = alpha * (i + 1);
    double sum = 0;

    for (int j=left; j < right; j++) {
      double sum += active[j];
    }

    POD_hist[i] = sum;
  }

  // Smoothing POD histogram
  double smoothed_POD[nsectors];
  for (int i=0; i < nsectors; i++) {
    double sum_element = POD_hist[i] * l;

    for (int j=0; j < l; j++) {
      sum_element += (POD_hist[i-(j+1)] + POD_hist[(i+(j+1)) % nsectors]) * (l - j);
    }

    double smoothed_element = sum_element / ((2 * l) + 1);
    smoothed_POD[i] = smoothed_element;
  }
  
  return POD_hist;

}

int * candidate_valley(double * smoothed_POD, double valley_threshold) {
  /* Selects the candidate valley based on the produced Polar Histogram */
  int lst_length = sizeof(smoothed_POD) / sizeof(double);
  int * candidate_idx = malloc(lst_length * sizeof(int));
  int idx_counter;

  // Loop through densities and select candidate positions
  for (int i = 0; i < lst_length; i++) {

    if (smoothed_POD[i] < valley_threshold) {
      candidate_idx[i] = 1;
      idx_counter++;
    } else {
      candidate_idx[i] = NULL;
    }
  }

  // Clean list and return indexes of valleys in histogram
  int * candidate_lst = malloc(idx_counter * sizeof(int));
  int temp = 0;
  for (int i = 0; i < lst_length; i++) {
    if (candidate_idx[i] != NULL) {
      candidate_lst[temp] = candidate_idx[i];
      temp++;
    }
  }
  
  free(candidate_idx);

  return candidate_lst;
}

/*=======================================================================*/


double calculate_avoidance_angle(double *smoothed_POD, Robot * robot, int * candidate_lst, double alpha, double s_max, double valley_threshold) {
  /* Retrieves the angle that the robot must drive towards. */
  int candidates_len = sizeof(candidate_lst) / sizeof(int);
  int nsectors = 360/alpha;

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
  for (int i = 0; i < candidates_len; i++) {
    int idx = candidate_lst[i];

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
    for (int i = 0; i < candidates_len; i++) {
    int idx = candidate_lst[i];

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

  k_f = k_n;
  
  if (k_n == goal_sector) {
    return goal_sector * alpha;

  } else if (k_n > goal_sector) {
    // See how big the valley is and then select the middle
    for (int i=1; i < s_max + 1; i++) {
      if ( (smoothed_POD[(k_n + i) % nsectors] < valley_threshold) && ( ((k_n + i) % nsectors) * alpha) <= 90) {
        k_f = k_n + i;
      } else {
        break;
      }
    }

    return ((k_n + k_f) / 2 % nsectors) * alpha;

  } else {
    // See how big the valley is and then select the middle
    for (int i=1; i < s_max + 1; i++) {
      if ( (smoothed_POD[(k_n + i) % nsectors] < valley_threshold) && ( ((k_n + i) % nsectors) * alpha) <= 270) {
        k_f = k_n - i;
      } else {
        break;
      }
    }

    return ((k_n + k_f) / 2 % nsectors) * alpha;
  }
}

double velocity_control(histogram * hist, double direction) {
  // Max velocity
  double V_MAX = 10;

  // Convert the direction of travel into sector index
  int h_idx = floor((180 * direction / M_PI)  / hist->alpha);

  // Retrieve polar histogram density at this sector
  double h_c = hist->densities[h_idx];

  // NOTE: If h_c > 0, that indicates that an obstacles lies ahead of the robot

  // Define h_m which is an empirically determined constant. h_cc later must be less than h_m
  double h_m = 123;

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
