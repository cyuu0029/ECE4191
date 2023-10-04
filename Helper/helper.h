/*
** Helper Headers
**
** Host all generic helper function headers and definitions
*/

/*===========================================================================*/

#ifndef N_SENSORS
#define N_SENSORS 6     // Number of Ultrasonic Sensors
#endif

#ifndef M_PI        // Pi, duh!
#define M_PI 3.141592653589793238462643383279502884196      
#endif

#ifndef M_TWOPI     // 2*Pi, duh!
#define M_TWOPI 6.2831853071795862319959        
#endif

#ifndef M_E     // Exponential, duh!
#define M_E 2.71828182845904523536
#endif

#ifndef ARENA
#define ARENA 120
#endif

#ifndef DEG2RAD
#define DEG2RAD M_PI / 180
#endif
#ifndef Sensor
typedef struct {
  int direction[N_SENSORS]; /* [degrees] */
  int distance[N_SENSORS]; /* [cm] */
} Sensor;
#endif

long double calculate_angle_modulo(long double angle);


int angle_clamp(int angle);

// Calculate distance between current position and end goal
double calculate_distance_from_goal(double pos_x, double pos_y, double goal_x, double goal_y);

// Calculate the angle from current position to goal
double calculate_goal_angle(double pos_x, double pos_y, double pos_yaw, double goal_x, double goal_y);

// Returns whether or not the Ultrasonic detects a wall or obstacles
int detect_obstacle(double angle, double distance, double pos_x, double pos_y, double pos_theta, double threshold);