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

  // Determine distance of returned ultrasonic signal and convert into grid 
  double cell_distance[N_SENSORS];
  for (int i = 0; i < N_SENSORS; ++i) {
    cell_distance[i] = sensors->distance[i] / map->resolution;
  }

  // Determine cell location of obstacles
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
    
    double new_x = (pos_x/map->resolution - 1) + floor(cell_distance[i] * cos(theta));
    double new_y = (pos_y/map->resolution - 1) + floor(cell_distance[i] * sin(theta));
    
    //char out[60];
    //sprintf(out, "new x: %lf, new y: %lf\n\n", new_x, new_y);
    //UART_PutString(out);
    
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
void smoothed_POD_histogram(POD * smoothed_POD, grid *active, double alpha, double l, double a, double b) {
  int sectors = smoothed_POD->nsectors;
  int width = active->width;
  int height = active->height;
  double POD_hist[72];

  // Create POD histogram
  for (int i=0; i< 72; i++) {
    POD_hist[i] = 0;
}

  for (int i=0; i < width; i++) {
    for (int j=0; j < height; j++) {
        /* Calculate the angular position (beta) of this cell. */
        double beta = 180 * atan2((double)(j - height/2), (double)(i - width/2))/M_PI;
        if( beta < 0 ) {
            beta += 360;
        }

      /* Calculate the obstacle density of this cell. */
      double density = pow(active->cells[i * width + j], 2);
      density *= a - b * sqrt(pow(i - width/2, 2) + pow(j - height/2, 2));

      /* Add density to respective point in the histogram. */
      POD_hist[(int) floor(beta / alpha)] += density;
    }
    
  }
  
  

  // Smoothing POD histogram
  for (int i=0; i < sectors; i++) {
    double sum_element = POD_hist[i] * l;

    for (int j=0; j < l; j++) {
        if (i - (j+1) < 0) {
            sum_element += (POD_hist[i-(j+1)+sectors] + POD_hist[(i+(j+1)) % sectors]) * (l - j);
        } else {
            sum_element += (POD_hist[i-(j+1)] + POD_hist[(i+(j+1)) % sectors]) * (l - j);
        }
    }

    double smoothed_element = sum_element / ((2 * l) + 1);
    smoothed_POD->density[i] = smoothed_element;
  }

}


/*=======================================================================*/


double calculate_avoidance_angle(POD *smoothed_POD, Robot * robot, int * candidate_lst, double alpha, int s_max) {
  /* Retrieves the angle that the robot must drive towards. */
  int candidates_len = 72;
  int nsectors = smoothed_POD->nsectors;

  // Retrive useful variables
  double pos_x = robot->x;
  double pos_y = robot->y;
  double pos_yaw = robot->theta;
  double goal_x = robot->goal_x;
  double goal_y = robot->goal_y;

  double goal_angle = calculate_goal_angle(pos_x, pos_y, pos_yaw, goal_x, goal_y);
  int goal_sector = round((180 * calculate_angle_modulo(goal_angle) / M_PI) / alpha);
  int abs_min = 1000;
  int k_n, k_f;
  int valley_flag = 0;
  int abs_sec = 0;      // Sector where the absolute minimum distance is
  int min_val = 1000;
  double best_angle = 1000;  // Degrees

  // Find angle, note we are working in degrees here
  for (int i = 0; i < candidates_len; i++) {
    int idx = candidate_lst[i];
  
    // Calculating the minimum distance between the goal sector and the candidate valley
    int min_distance;
    //int knf_flag;       // Check if min distance is from the nearest or farthest edge
    double angle;       // Degrees
    
    switch(idx) {
        case 0:
            // Early exit case if goal sector is clear
            if (i == goal_sector) {
                //return goal_sector * alpha;
            }
            
            // Calculate the 'distance' between current sector and the goal sector
            if (abs(i - goal_sector) < abs(abs(i - goal_sector) - nsectors)) {
                min_distance = abs(i - goal_sector);
            } else {
                min_distance = abs(abs(i - goal_sector) - nsectors);
            }
            
                
            // Start entering a valley
            if (valley_flag) {
                k_f++;
                // Check if new sector is closer to the goal 
                if (min_distance < abs_min){
                    abs_min = min_distance;
                    abs_sec = i;
                    //knf_flag = k_f;
                }
                
            } else { 
                // Always select closest to goal sector
                abs_min = min_distance;
                abs_sec = i;
                k_n = i;
                k_f = k_n;
                
                // Start valley
                valley_flag = 1;
                //knf_flag = 0;
            }
            break;
        
        case 1:
            // Exiting a valley
            if (valley_flag) {
                // Update Valley flag
                valley_flag = 0;
                
                // Check for wide and narrow valleys
                if (abs(k_n - k_f) < s_max) {   // Narrow Valley
                    angle = ((k_n + k_f) / 2 % nsectors) * alpha; // Degrees
                        
                } else {                        // Wide Valley
                    int first = abs(k_n - goal_sector);
                    int last = abs(k_f - goal_sector);
                    if (first <= abs_min) {
                        angle = ((k_n + (s_max / 2)) / 2 % nsectors) * alpha;
                    } else if (last < abs_min) {
                        angle = ((k_f - (s_max / 2)) / 2 % nsectors) * alpha;
                    }

                }
                    
                // Check if this angle is closer to goal sector than current best
                if (abs(round((180 * calculate_angle_modulo(goal_angle) / M_PI) - goal_sector) < min_val)) {
                    best_angle = angle;
                    min_val = abs(round((180 * calculate_angle_modulo(goal_angle) / M_PI) - goal_sector));
                }
            }
            break;
        
        default:
            return 9000;
    }
  }    
    if (best_angle == 1000) {
        int first = abs((k_n + (s_max / 2)) - goal_sector);
        int last = abs((k_f - (s_max / 2)) - goal_sector);
        if (first < abs_min) {
            best_angle = ((k_n + (s_max / 2)) / 2 % nsectors) * alpha;
        } else if (last < abs_min) {
            best_angle = ((k_f - (s_max / 2)) / 2 % nsectors) * alpha;
        }
        else {
            best_angle = abs_sec * alpha;
        }
    }
    return best_angle;
  
}


double velocity_control(POD * smoothed_POD, double direction, double alpha, double h_m) {
  // Max velocity
  double V_MAX = 3;

  // Convert the direction of travel into sector index
  int h_idx = floor((180 * calculate_angle_modulo(direction) / M_PI)  / alpha);
  
  // Retrieve polar histogram density at this sector
  double h_c = smoothed_POD->density[h_idx];

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
