/*
** Virtual Field Histogram (VFH)
**
** Host all header files for VFH functionality
*/

/*===========================================================================*/

// HISTOGRAM GRID

#define HISTOGRAM_GRID_H
#define M_PI 3.14159265358979323846

// Global tracker for number of sensors
#define N_SENSORS 5


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
typedef struct {
  int direction[N_SENSORS]; /* [degrees] */
  int distance[N_SENSORS]; /* [cm] */
} sensor_data;

/* read_sensors: Retrieves ultrasonic sensor readings*/
sensor_data * read_sensor();

/* initial_grid: Return a pointer to an empty (all zeros) grid. NULL otherwise. */
grid * grid_create(int width, int height, int resolution);

/* active_window: Get a sub-grid of the grid that represents the current active window.
 * It is centered around (x, y), most likely robot coordinates in our scenario */
grid * active_window(grid * map, int pos_x, int pos_y, int dimension);

/* update_grid: Update histogram grid cells with sensor readings. */
int grid_update(grid * map, int pos_x, int pos_y, int yaw, sensor_data data);


/*===========================================================================*/

// POLAR HISTOGRAM

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
void polar_histogram_update(histogram * hist, grid * map);

/* candidate_valley: Identifies the the candidate valleys that the robot and drive through */
int * candidate_valley(histogram * smoothed_histogram);

/*===========================================================================*/

// VFH

#define VFH_H

/* Control signal created by the algorithm. */
typedef struct {
  int direction; /* [degrees] */
} control_signal_t;

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
double calculate_avoidance_angle(histogram * hist, grid * map, int * candidate_lst, double pos_x, double pos_y, double pos_yaw, double goal_x, double goal_y);

/* velocity control: Returns a velocity value based on the distance between objects. */
double velocity_control(histogram * hist, double direction);

/* calculate_dsiatnce_from_goal: Returns the distance from robot to goal */
double calculate_distance_from_goal(double pos_x, double pos_y, double goal_x, double goal_y);
