/*
** Tentacles Definitions
**
** Host all headers for Tentacles functionality
*/

/*===========================================================================*/

typedef struct {
    int n_tentacles;    // Number of tentacles
    double dt;
    double steps;
    double alpha;
    double beta;
    

} Tentacles;





double tentacles_cost_function(Tentacles * octopussy, Sensor * sensors, double v, double w, double goal_x, double goal_y, double goal_th, double pos_x, double pos_y, double pos_theta);

double * planner(Tentacles * octopussy, Sensor * sensors, int goal_x, int goal_y, double goal_th, int pos_x, int pos_y, int pos_theta);

int check_collision(Sensor * sensors, double robot_width)
