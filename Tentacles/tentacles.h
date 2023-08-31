/*
** Tentacles Definitions
**
** Host all header files for Robot functionality
*/

/*===========================================================================*/
#define N_SENSORS 5     // Number of Ultrasonic Sensors

#ifndef M_PI    // Pi, duh!
#define M_PI 3.141592653589793238462643383279502884196      
#endif

#ifndef M_TWOPI     // 2*Pi, duh!
#define M_TWOPI 6.2831853071795862319959        
#endif

#ifndef M_E     // Exponential, duh!
#define M_E 2.71828182845904523536
#endif
   
typedef struct {
    int n_tentacles;    // Number of tentacles
    double dt;
    double steps;
    double alpha;
    double beta;
    

} Tentacles;

typedef struct {
  int direction[5]; /* [degrees] */
  int distance[5]; /* [cm] */
} Sensor;



double tentacles_cost_function(Tentacles * octopussy, Sensor * sensors, double v, double w, double goal_x, double goal_y, double goal_th, double pos_x, double pos_y, double pos_theta);

double * planner(Tentacles * octopussy, Sensor * sensors, int goal_x, int goal_y, double goal_th, int pos_x, int pos_y, int pos_theta);

double calculate_distance_from_goal(double pos_x, double pos_y, double goal_x, double goal_y);

long double calculate_angle_modulo(long double angle);
