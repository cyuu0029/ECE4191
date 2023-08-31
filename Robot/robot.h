/*
** Robot Definitions
**
** Host all header files for Robot functionality
*/

/*===========================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

typedef struct {
    long double duty_cycle;
    long double int_error;  // integrated error
    long double w; // omega, [rad per sec]
    long double tangent_v; //tangential velocity, [cm per sec]
    long double desired_w;
    long double Ki;
    long double Kp;
    double wheel_radius; // wheel radius in cm
    int32 enc_count;

} Motor;

typedef struct {
    // Physical measurement
    long double axle_width; // in cm

    // Position
    long double theta;  // in RADIANS
    long double x;   // in cm
    long double y;   // in cm

    // Velocities    
    long double v;   // in cm/s
    long double w;   // in rad/s
    
    // Desired velocity values
    long double desired_v;
    long double desired_theta;
    
    // PI controller 
    long double Ki;
    long double Kp;
    long double int_error; // Integrated error for PI control
    
    // Goal tracker
    long double goal_x;
    long double goal_y;
    long double goal_min_dist; // specifies a threshold of minimum distance to goal at which robot will stop

} Robot;

void motor_create(Motor * motorola, long double wheel_r_scale, long double K_i, long double K_p);
void Drive_Left_Motor(long double duty_cycle);
void Drive_Right_Motor(long double duty_cycle);
long double angle_modulo(long double angle);

void robot_create(Robot * bender, long double robot_axle_width, long double K_i, long double K_p, long double minimum, long double x, long double y);