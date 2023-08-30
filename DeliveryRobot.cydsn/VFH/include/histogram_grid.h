/**
 * @file histogram_grid.h
 * @brief Define what a Histogram Grid is and how to manipulate it.
 */

#ifndef HISTOGRAM_GRID_H
#define HISTOGRAM_GRID_H


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Global tracker for number of sensors
#ifndef N_SENSORS
#define N_SENSORS 5
#endif


/* Histogram grid.
 * Define Histogram Grid Strcuture
 * 
 */
typedef struct {
  int width;      // Width of grid
  int height;     // Height of grid
  int resolution; // Size of cell in grid
  unsigned long *cells;     // Obstacle density of each cell
} grid;

/* Sensor Measurement Data
 * Stores the distance measurement from ultrasonic sensor
 * Organised as: [N, NE, E, SE, S, SW, W, NW]
 * where N is the front of the robot regardless of real world orientation
 */
typedef struct {
  int direction[N_SENSORS]; /* [degrees] */
  unsigned long distance[N_SENSORS]; /* [cm] */
} sensor_data;

/* initial_grid: Return a pointer to an empty (all zeros) grid. NULL otherwise. */
grid * initial_grid(int width, int height, int resolution);

/* active_window: Get a sub-grid of the grid that represents the current active window.
 * It is centered around (x, y), most likely robot coordinates in our scenario */
grid * active_window(grid * map, int pos_x, int pos_y, int dimension);

/* update_grid: Update histogram grid cells with sensor readings. */
int update_grid(grid * map, int pos_x, int pos_y, int yaw, sensor_data data);

#endif
