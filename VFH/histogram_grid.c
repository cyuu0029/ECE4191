#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include "vfh.h"

// Histogram Grid for VFH implementation

grid * grid_create(int width, int height, int resolution) {
  /* Source code mentions assertions for poorly defined resolution values
   * This assertion shouldn't be necessary given how well defined our environment is
  */

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
  double pos_x = robot->x;
  double pos_y = robot->y;
  double yaw = robot->theta;

  // Determine distance of obstacle
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

/* TODO: Finish implementing get_moving_window. */
grid * active_window(grid * map, int curr_x, int curr_y, int dimension) {
  /*
   * Creates an active window surrounding robot.
   *
   * If grid_init returns NULL, exit the function.
  */
  grid * active = grid_create(dimension, dimension, map->resolution);

  if (active != NULL) {

    /* Populate active window with existing values in grid map 
     * Original author suggests using pointers to point directly to grid map values.
     * This can be done but whether or not it is necessary is up for debate.
    
    */
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {

        /* x and y are the center coordinates of the robot body with sensors. */
        int grid_i = i + curr_x/map->resolution + (dimension - 1) / 2;
        int grid_j = j + curr_y/map->resolution + (dimension - 1) / 2;

        /* Copy the information from the grid to the moving window. */
        if (grid_i < map->width && grid_j < map->height) {
          active->cells[i * dimension + j] = map->cells[grid_i * map->width + grid_j];
        }
      }
    }
  } else {
    free(active);
    return NULL;
  }

  return active;
}
