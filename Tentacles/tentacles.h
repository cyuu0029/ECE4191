/*
** Tentacles Definitions
**
** Host all header files for Robot functionality
*/

/*===========================================================================*/
   
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
