/*
** Virtual Field Histogram (VFH)
**
** Host all header files for VFH functionality
*/

/*===========================================================================*/
#include "..\Helper\helper.h"
#include "..\Robot\robot.h"

// HISTOGRAM GRID
#ifndef HISTOGRAM_GRID_H
#define HISTOGRAM_GRID_H
/* Histogram grid.
 * Define Histogram Grid Strcuture
 * 
 */
typedef struct {
  int width;      // Width of grid
  int height;     // Height of grid
  int resolution; // Size of cell in grid
  int *cells;     // Obstacle density of each cell
} grid;

/* Sensor Measurement Data
 * Stores the distance measurement from ultrasonic sensor
 * Organised as: [N, NE, E, SE, S, SW, W, NW]
 * where N is the front of the robot regardless of real world orientation
 */

/* initial_grid: Return a pointer to an empty (all zeros) grid. NULL otherwise. */
grid * grid_create(int width, int height, int resolution);

/* active_window: Get a sub-grid of the grid that represents the current active window.
 * It is centered around (x, y), most likely robot coordinates in our scenario */
double * active_window(Sensor * sensors, double a, double b);

/* update_grid: Update histogram grid cells with sensor readings. */
int grid_update(grid * map, Sensor * sensors, Robot * robot);

#endif
/*===========================================================================*/

// POLAR HISTOGRAM
#ifndef POLAR_HISTOGRAM_H
#define POLAR_HISTOGRAM_H

/* Polar histogram. */
typedef struct {
  int alpha;
  int sectors;
  double threshold;
  double damping_constant;
  double density_a;
  double density_b;
  int * densities;
} histogram;

/* hist_init: Return a pointer to a new hist. NULL otherwise. */
histogram * polar_histogram_create(int alpha, double threshold, double density_a, double density_b);

/* hist_update: Update hist with grid's information. */
double * smoothed_POD_histogram(double * active, double alpha, double l);

/* candidate_valley: Identifies the the candidate valleys that the robot and drive through */
int * candidate_valley(double * smoothed_POD, double valley_threshold);

#endif
/*===========================================================================*/

// VFH
#ifndef VFH_H
#define VFH_H

/* Helpers. */

/* modulo: Return x modulo m. */
int modulo(int x, int m);

/* modular_dist: Return the distance between a and b in modular arithmetic. */
int modular_dist(int a, int b, int m);

/* Control signals. */

/*
** calculate_direction: Return the sector in hist closest to the objective
** direction, given that its obstacle density is less than the threshold
** specified in hist. The objective_direction is given in DEGREES.
*/
int calculate_direction(histogram * hist, int objective_direction);

/* calculate_avoidance_angle: Returns the angle that the robot should drive towards */
double calculate_avoidance_angle(double *smoothed_POD, Robot * robot, int * candidate_lst, double alpha, double s_max, double valley_threshold);

/* velocity control: Returns a velocity value based on the distance between objects. */
double velocity_control(histogram * hist, double direction);

/* calculate_dsiatnce_from_goal: Returns the distance from robot to goal */
double calculate_distance_from_goal(double pos_x, double pos_y, double goal_x, double goal_y);

#endif