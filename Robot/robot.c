#include "robot.h"

void *motor_create(Motor * motorola, long double wheel_r_scale, long double K_i, long double K_p) {
    motorola->duty_cycle = 0;
    motorola->int_error = 0;
    motorola->desired_w = 0;
    motorola->wheel_radius = wheel_r_scale * 2.75;
    motorola->enc_count = 0;
    motorola->Ki = K_i;  // TODO: determine good PI params
    motorola->Kp = K_p;
    
    return (motorola);
}

void *robot_create(Robot * bender, long double robot_axle_width, long double K_i, long double K_p, long double minimum, long double x, long double y, long double theta) {
    bender->axle_width = robot_axle_width; 
    bender->int_error = 0;
    bender->Ki = K_i;    // TODO: determine good PI values
    bender->Kp = K_p;
    bender->desired_v = 0;
    bender->desired_theta = 0;
    bender->theta = theta;
    bender->x = x;
    bender->y = y;
    bender->goal_min_dist = minimum;
    bender->goal_x = 0;
    bender->goal_y = 0;
    
    return (bender);
}