/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "..\VFH\include\vfh.h"


struct Motor {
    long double duty_cycle;
    long double int_error;  // integrated error
    long double w; // omega, [rad per sec]
    long double tangent_v; //tangential velocity, [cm per sec]
    long double desired_w;
    long double Ki;
    long double Kp;
    long double wheel_radius; // wheel radius in cm
    int32 enc_count;
};

struct Robot {
    long double theta;  // in RADIANS
    long double x;   // in cm
    long double y;   // in cm
    long double axle_width; // in cm
    
    long double V;   // in cm/s
    long double w;   // in rad/s
    
    long double desired_V;
    long double desired_theta;
    
    long double Ki;
    long double Kp;
    long double int_error; // integrated error for PI control
    
    long double goal_x;
    long double goal_y;
    long double goal_min_dist; // specifies a threshold of minimum distance to goal at which robot will stop
};

const long double PULSES_PER_REV = 3591.92;
const long double POSE_UPDATE_PERIOD = 1.0/50.0; // seconds

void Drive_Left_Motor(long double duty_cycle);
void Drive_Right_Motor(long double duty_cycle);
long double angle_modulo(long double angle);

uint8_t echo_flag = 0;
uint16_t max_count = 65535;
uint16_t echo_distance;
uint8_t mux_select = 0;
int32 left_wheel_count = 0;
int32 right_wheel_count = 0;
char serial_output[150];

struct Motor left_motor;
struct Motor right_motor;
struct Robot robot;

/* Declaration of the needed data structures. */
grid * certainty_grid;
sensor_data sensors;
histogram * polar_histogram;
control_signal_t control_signal;

void print_grid(grid * map);
void print_polar_histogram(histogram * polar_histogram);

  
CY_ISR( Timer_Int_Handler ) {
    echo_distance = 65535 - Timer_Echo_ReadCapture();  // in cm
    sensors.distance[mux_select] = echo_distance;
    
    
    //Timer_Echo_Stop();
    //CyDelayUs(1); // TODO: Should be able to make this shorter, one or two bus clock cycles
    mux_select++;
    if ( mux_select == N_SENSORS) {
        grid_update(certainty_grid, robot.x, robot.y, 180*robot.theta/M_PI, sensors);
        mux_select = 0;
    }
    Control_Reg_US_Write(mux_select);    
    //Timer_Echo_Enable();
    
    PWM_Trigger_WriteCounter(255);    
}

CY_ISR( Pose_Update_Int_Handler ) {
    int32 new = QuadDec_R_GetCounter();
    int32 diff = new - right_motor.enc_count;
    right_motor.enc_count = new;
    right_motor.w = M_TWOPI*diff/POSE_UPDATE_PERIOD/PULSES_PER_REV;
    
    new = QuadDec_L_GetCounter();
    diff = new - left_motor.enc_count;
    left_motor.enc_count = new;
    left_motor.w = M_TWOPI*diff/POSE_UPDATE_PERIOD/PULSES_PER_REV;
    
    
    
    //Calculate and update tangential velocity of wheels
    left_motor.tangent_v = left_motor.w*left_motor.wheel_radius;
    right_motor.tangent_v = right_motor.w*right_motor.wheel_radius;

    //temporary values
    robot.w = (right_motor.tangent_v - left_motor.tangent_v)/robot.axle_width; //instantaneous turning velocity
    robot.V =  (right_motor.tangent_v + left_motor.tangent_v)/2; //instantaneous tangential velocity of robot centre

    // update pose variables
    robot.theta = angle_modulo( robot.theta + robot.w * POSE_UPDATE_PERIOD );
    robot.x = robot.x + POSE_UPDATE_PERIOD * robot.V * cos(robot.theta);
    robot.y = robot.y + POSE_UPDATE_PERIOD * robot.V * sin(robot.theta);
    
    
    // do robot PI control
    long double error = robot.desired_theta - robot.theta;   
    if( error > M_PI ) {     // TODO: give this more thought. Want the robot to choose direction of rotation efficiently, but this might work
        error = error - M_TWOPI;
    }
    if( error < -M_PI) {
        error = error + M_TWOPI;
    }
    
    double scaled_V = robot.desired_V *( 1 - logl( (M_E-1) * fabsl(error) / M_PI + 1 )); // scales velocity depending on how much we have to rotate (makes robot turn on spot more)
    
    robot.int_error = robot.int_error + error;
    long double new_omega = robot.Kp * error + robot.Ki * robot.int_error;
    right_motor.desired_w = (scaled_V + new_omega * robot.axle_width / 2) / right_motor.wheel_radius;
    left_motor.desired_w = (scaled_V - new_omega * robot.axle_width / 2) / left_motor.wheel_radius;   
    
}

CY_ISR( Motor_PI_Int_Handler ) {
    long double error = left_motor.desired_w - left_motor.w;
    left_motor.int_error  = left_motor.int_error + error;
    left_motor.duty_cycle = left_motor.duty_cycle + left_motor.Kp*error + left_motor.Ki*left_motor.int_error;
    Drive_Left_Motor(left_motor.duty_cycle);
    
    error = right_motor.desired_w - right_motor.w;
    right_motor.int_error  = right_motor.int_error + error;
    right_motor.duty_cycle = right_motor.duty_cycle + right_motor.Kp*error + right_motor.Ki*right_motor.int_error;
    Drive_Right_Motor(right_motor.duty_cycle);
}

CY_ISR( Navigation_Test_Int_Handler ) {
    long double angle;
    angle = robot.desired_theta + M_PI/8;
    if( angle >= M_TWOPI ) {
        angle = angle - M_TWOPI;
    }
    robot.desired_theta = angle;
}




int main(void)
{
    long double wheel_r_scale = 0.9578;   
    left_motor.duty_cycle = 0;
    left_motor.int_error = 0;
    left_motor.desired_w = 0;
    left_motor.wheel_radius = wheel_r_scale * 2.75;
    left_motor.enc_count = 0;
    left_motor.Ki = 3e-6;  // TODO: determine good PI params
    left_motor.Kp = 0.0025;
    
    right_motor.duty_cycle = 0;
    right_motor.int_error = 0;
    right_motor.desired_w = 0;
    right_motor.wheel_radius = wheel_r_scale * 1.001*2.75;
    right_motor.enc_count = 0;
    right_motor.Ki = 3e-6;  // TODO: determine good PI params
    right_motor.Kp = 0.0025;
    
    robot.axle_width = 0.967*22.5; // TODO: get accurate measurement
    robot.int_error = 0;
    robot.Ki = 3e-5;    // TODO: determine good PI values
    robot.Kp = 0.75;
    robot.desired_V = 0;
    robot.desired_theta = 0;
    robot.theta = 0;
    robot.x = 30;
    robot.y = 30;
    robot.goal_min_dist = 1;
    
    
    sensors.direction[0] = 0;
    sensors.direction[1] = 45;
    sensors.direction[2] = 90;
    sensors.direction[3] = 270;
    sensors.direction[4] = 315;
    
    CyGlobalIntEnable;
    
    // Registration of Timer ISR
    Timer_Echo_Int_StartEx( Timer_Int_Handler );
    Pose_Update_Int_StartEx( Pose_Update_Int_Handler );
    Motor_PI_Int_StartEx( Motor_PI_Int_Handler );
    Testing_Int_StartEx( Navigation_Test_Int_Handler );
    
    // Start up code - enable UART, PWM and Timer used for ultrasonic module
    UART_Start();
    Timer_Echo_Start();
    PWM_Trigger_Start();
    QuadDec_L_Start();
    PWM_Motor_L_Start();
    QuadDec_R_Start();
    PWM_Motor_R_Start();
    Timer_Avoidance_Start();
    Timer_Avoidance_WriteCounter(1000); // Cause robot to start moving immediately
        
    // Creation of VFH grid environment
    sensors.direction[0] = 0;
    sensors.direction[1] = 30;
    sensors.direction[2] = 90;
    sensors.direction[3] = 270;
    sensors.direction[4] = 300;
    certainty_grid = grid_create(65, 65, 2);
    polar_histogram = polar_histogram_create(5, 20, 10, 5);

    // Check initialisation
    if (certainty_grid == NULL) return -1;
    if (certainty_grid->cells == NULL) return -1;
    if (polar_histogram == NULL) return -1;
    if (polar_histogram->densities == NULL) return -1;

    // Take in initial sensor measurements

    // Update Grid
    int UPDATE_FLAG = grid_update(certainty_grid, robot.x, robot.y, robot.theta, sensors);
    if (UPDATE_FLAG != 1) return -1;

    // Print the current grid
    print_grid(certainty_grid);

    // Define Goal Coordinates
    int N_GOALS = 2;
    int goals[4] = {90, 90, 30, 90};
    robot.goal_x = goals[0];
    robot.goal_y = goals[1]; 
    int goal_counter = 0;
    
    for(;;) {
      
        int dist_to_goal = calculate_distance_from_goal(robot.x, robot.y, robot.goal_x, robot.goal_y);
        sprintf(serial_output, "Distance to goal: %d\n", dist_to_goal);
        UART_PutString(serial_output);
        if( dist_to_goal <= robot.goal_min_dist ) { 
            robot.desired_V = 0;
            if (goal_counter < N_GOALS) {
                robot.goal_x = goals[goal_counter + 2];
                robot.goal_y = goals[goal_counter + 2];
                goal_counter += 2;
                
                // Adjust robot angle to face new goal
                //robot.desired_theta = calculate_goal_angle(robot.x, robot.y, robot.theta, robot.goal_x, robot.goal_y); 
            }
        } else {
            CyDelay(5000);
            //robot.desired_V = 7;

            // Testing VFH Algorithm
            grid * active = active_window(certainty_grid, robot.x, robot.y, 20);

            // Print the current active window
            print_grid(active);

            polar_histogram_update(polar_histogram, active);
                        
            // print_polar_histogram(polar_histogram);

            // Identify candidate valleys
            int * candidates = candidate_valley(polar_histogram);

            // Find angle of drive
            double new_theta = calculate_avoidance_angle(polar_histogram, certainty_grid, candidates, robot.x, robot.y, robot.theta, robot.goal_x, robot.goal_y);
            if (abs(new_theta - robot.desired_theta) > 10) {
                //robot.desired_theta = new_theta;
            }
                
            sprintf(serial_output, "%Lf\n", robot.desired_theta);
            UART_PutString(serial_output);

            // Adjust velocity
            //double adjusted_V = velocity_control(polar_histogram, robot.desired_theta);

            //robot.desired_V = adjusted_V;

            CyDelay(5000);
        }    
    }
}
  


void Drive_Left_Motor(long double duty_cycle) {
    if (duty_cycle < -1) {
        duty_cycle = -1;
    } else if (duty_cycle > 1) {
        duty_cycle = 1;
    }
    
    if (duty_cycle < 0) {
        duty_cycle = -duty_cycle;
        PWM_Motor_L_WriteCompare1(0);
        PWM_Motor_L_WriteCompare2(duty_cycle*10000);
    } else {
        PWM_Motor_L_WriteCompare1(duty_cycle*10000);
        PWM_Motor_L_WriteCompare2(0);
    }
}

void Drive_Right_Motor(long double duty_cycle) {
    if (duty_cycle < -1) {
        duty_cycle = -1;
    } else if (duty_cycle > 1) {
        duty_cycle = 1;
    }
    
    if (duty_cycle < 0) {
        duty_cycle = -duty_cycle;
        PWM_Motor_R_WriteCompare1(0);
        PWM_Motor_R_WriteCompare2(duty_cycle*5000);
    } else {
        PWM_Motor_R_WriteCompare1(duty_cycle*5000);
        PWM_Motor_R_WriteCompare2(0);
    }
}

long double angle_modulo(long double angle) {
    return angle - M_TWOPI * floor(angle / M_TWOPI);  
}

void print_polar_histogram(histogram * polar_histogram) {
  for (int i=0; i < polar_histogram->sectors; i++) {
    sprintf(serial_output, "%d\n", polar_histogram->densities[i]);
    UART_PutString(serial_output);
  }
  UART_PutString("\n\n\n\n\n");
}

void print_grid(grid * map) {
  // Print map
  int WIDTH = map->width;
  int HEIGHT = map->height;

  for (int i = 0; i < WIDTH; ++i) {
    for (int j = 0; j < HEIGHT; ++j) {
      sprintf(serial_output, "%d", map->cells[i * WIDTH + j]);
      UART_PutString(serial_output);
    }
    sprintf(serial_output, "\n");
    UART_PutString(serial_output);
  }

  UART_PutString("\n\n\n\n\n");
}

/* [] END OF FILE */
