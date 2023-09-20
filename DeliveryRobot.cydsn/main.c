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

/* Import all libraries and header files. */
#include "project.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

/* Include all our own written headers */
//#include "..\Tentacles\tentacles.h"
#include "vfh.h"

/* Define all global variables. */ 
const double PULSES_PER_REV = 3591.92;
const double POSE_UPDATE_PERIOD = 1.0/50.0; // seconds

uint8_t echo_flag = 0;          // Ultrasonic flag
uint16_t max_count = 8500;     // Ultrasonic time count
uint16_t echo_distance;         // Ultrasonic distance
uint8_t mux_select = 0;         // For selecting specific ultrasonic sensor

int32 left_wheel_count = 0;
int32 right_wheel_count = 0;
char serial_output[150];        // For UART print output

/* Defining/Creating all data structures */
Motor left_motor;     // Left Motor, duh!
Motor right_motor;    // Right Motor, duh!
Robot robot;          // Robot values, duh!
Sensor sensors;       // Ultrasonics
grid map;             // Grid of area
//histogram polar;      // Polar Histogram
grid active;          // Active window of robot
POD smoothed_POD;


void Drive_Left_Motor(long double duty_cycle);
void Drive_Right_Motor(long double duty_cycle);

/* Interrupt to obtain Ultrasonic measurement value. */  
CY_ISR( Timer_Int_Handler ) {
    // Collect measurement 
    echo_distance = max_count - Timer_Echo_ReadCapture();   // in cm

    sensors.distance[mux_select] = echo_distance;      // Store measured value
    //sprintf(serial_output, "Ultrasonic sensor %d: %d\n",   mux_select, sensors.distance[mux_select]);
    //UART_PutString(serial_output);
        
    mux_select++;   // Iterate the global ultrasonic tracker
    
    // Reset the global ultrasonic tracker when all measurements have been updated
    if( mux_select == N_SENSORS ) { 
        // Update grid with new distance readings
        grid_update(&map, &sensors, &robot);
        mux_select = 0; 
    }
    
    Control_Reg_US_Write(mux_select);
    Timer_Echo_ReadControlRegister();
    PWM_Trigger_WriteCounter(2950);
}

/* Interrupt for Robot pose and desired drive update. */
CY_ISR( Pose_Update_Int_Handler ) {
    // Update encoder values for both left and right motors
    int32 new, diff;
    new = QuadDec_R_GetCounter();
    diff = new - right_motor.enc_count;
    right_motor.enc_count = new;
    right_motor.w = M_TWOPI * diff / POSE_UPDATE_PERIOD / PULSES_PER_REV;
    
    new = QuadDec_L_GetCounter();
    diff = new - left_motor.enc_count;
    left_motor.enc_count = new;
    left_motor.w = M_TWOPI * diff / POSE_UPDATE_PERIOD / PULSES_PER_REV;
    
    // Calculate and update tangential velocity of wheels
    left_motor.tangent_v = left_motor.w * left_motor.wheel_radius;
    right_motor.tangent_v = right_motor.w * right_motor.wheel_radius;

    // Calculate and update Robot velocity and angular velocity
    robot.w = (right_motor.tangent_v - left_motor.tangent_v) / robot.axle_width; //instantaneous turning velocity
    robot.v =  (right_motor.tangent_v + left_motor.tangent_v) / 2; //instantaneous tangential velocity of robot centre

    // Update Robot pose
    robot.theta = calculate_angle_modulo( robot.theta + robot.w * POSE_UPDATE_PERIOD );
    robot.x = robot.x + POSE_UPDATE_PERIOD * robot.v * cos(robot.theta);
    robot.y = robot.y + POSE_UPDATE_PERIOD * robot.v * sin(robot.theta);
    
    
    // Perform robot PI control
    long double error = robot.desired_theta - robot.theta;  
    if( error > M_PI ) {     // TODO: give this more thought. Want the robot to choose direction of rotation efficiently, but this might work
        error = error - M_TWOPI;
    }
    if( error < -M_PI) {
        error = error + M_TWOPI;
    }
    
    // Calculate scaled velocity depending on how much we have to rotate (makes robot turn on spot more)
    double scaled_V = robot.desired_v *( 1 - logl( (M_E - 1) * fabsl(error) / M_PI + 1 ));
    
    // Update motor tangential velocity based on error
    robot.int_error = robot.int_error + error;
    long double new_w = robot.Kp * error + robot.Ki * robot.int_error;
    right_motor.desired_w = (scaled_V + new_w * robot.axle_width / 2) / right_motor.wheel_radius;
    left_motor.desired_w = (scaled_V - new_w * robot.axle_width / 2) / left_motor.wheel_radius;   
    
}

/* Interrupt for motor PI error handler and drive. */
CY_ISR( Motor_PI_Int_Handler ) {
    long double error = left_motor.desired_w - left_motor.w;
    left_motor.int_error  = left_motor.int_error + error;
    left_motor.duty_cycle = left_motor.duty_cycle + left_motor.Kp * error + left_motor.Ki * left_motor.int_error;
    Drive_Left_Motor(left_motor.duty_cycle);
    
    error = right_motor.desired_w - right_motor.w;
    right_motor.int_error  = right_motor.int_error + error;
    right_motor.duty_cycle = right_motor.duty_cycle + right_motor.Kp*error + right_motor.Ki * right_motor.int_error;
    Drive_Right_Motor(right_motor.duty_cycle);

}

/* Interrupt for test handler? */
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
    // Enable all interrupts
    CyGlobalIntEnable;
    
    // Registration of Timer ISR
    Timer_Echo_Int_StartEx( Timer_Int_Handler );
    Pose_Update_Int_StartEx( Pose_Update_Int_Handler );
    Motor_PI_Int_StartEx( Motor_PI_Int_Handler );
    //Testing_Int_StartEx( Navigation_Test_Int_Handler );
    
    // Start up code - enable UART, PWM and Timer used for ultrasonic module
    UART_Start();
    Timer_Echo_Start();
    PWM_Trigger_Start();
    QuadDec_L_Start();
    PWM_Motor_L_Start();
    QuadDec_R_Start();
    PWM_Motor_R_Start();
    Timer_Avoidance_Start();
    Timer_Avoidance_WriteCounter(60000); // Cause robot to start moving immediately
    
    // Define and initialise motors
    long double wheel_r_scale = 0.9378;
    long double motor_Ki = 3e-6;     // TODO: Determine good value
    long double motor_Kp = 0.0025;   // TODO: Determine good value
    
    motor_create(&left_motor, wheel_r_scale, motor_Ki, motor_Kp);
    motor_create(&right_motor, wheel_r_scale, motor_Ki, motor_Kp);
    
    // Define and initialise robot 
    long double robot_axle_width = 0.936*22.5;  // TODO: get accurate measurement
    long double robot_Ki = 3e-5;    // TODO: Determine good value
    long double robot_Kp = 0.5;     // was previously 0.75 before changing for MS1
    long double min_distance = 10;   // Minimum distance between robot position and goal


    /*======================= ROBOT STARTING POSITION =======================*/
    long double start_x = 35;    // Starting x, duh!
    long double start_y = 35;    // Starting y, duh!

    robot_create(&robot, robot_axle_width, robot_Ki, robot_Kp, min_distance, start_x, start_y);
    
    /*=======================================================================*/

    // Define sensor directions (start from front sensor, then move clockwise)
    sensors.direction[0] = 0;
    sensors.direction[1] = 30;
    sensors.direction[2] = 90;
    sensors.direction[3] = 270;
    sensors.direction[4] = 330;

    /*========================= M1: Goal Definition =========================*/
    double n_goals = 6;    // Number of goals, duh!
    double goals[8] = {95, 95, 95, 95, 35, 95, 35, 35};    // Coordinates of goals [x1, y1, x2, y2, ..., xn, yn]
    robot.goal_x = goals[0];   // Update robot x goal
    robot.goal_y = goals[1];   // Update robot y goal
    int goals_reached = 0;  // Counter for number of goas reached, duh!

    /*=======================================================================*/    
    

    /*======================== M1: VFH initialisation =======================*/
    // Defining algorithm parameters taken from https://github.com/rzninvo/robotics_final_project/blob/main/launch/vfh_planning.launch
    map = *(grid_create(65, 65, 2));
    if( map.cells == NULL ) {
        UART_PutChar('N');
        CyDelay(10000000);
    }
    // Active Window
    double alpha = 5;       // Degrees
    double coeff_l = 3;     // Smoothing factor
    int window_size = 25;
    double coeff_a = 5;     // a - bd_max = 0 
    double coeff_b = coeff_a / (sqrt(2) * ((window_size - 1) / 2));  // d_max = sqrt(2) * (ws - 1) / 2
    
    
    active = *grid_create(window_size, window_size, 2);
    
    // Polar Histogram and Candidate Valley
    smoothed_POD = *pod_create(alpha);

    double valley_threshold_lower = 10;
    double valley_threshold_upper = 25;
    double s_max = 40;
    double h_m = 10;

    double ideal_angle, ideal_velocity;
    
    // Collect candidate valleys
    int candidate_idx[72];
            
    
    /*========================================================================*/           
    
    // Spoof ultrasonics
    
    for( int i = 0; i<100; i++ ) {
        for( int j = 0; j<N_SENSORS; j++ ) {
            sensors.direction[j] = 60.0*rand()/RAND_MAX;
            sensors.distance[j] = 20;
        }
        grid_update(&map, &sensors, &robot);
    }
    
    for( int i = 0; i<100; i++ ) {
        for( int j = 0; j<N_SENSORS; j++ ) {
            sensors.direction[j] = 120.0+ 60.0*rand()/RAND_MAX;
            sensors.distance[j] = 20;
        }
        grid_update(&map, &sensors, &robot);
    }
    
    for( int i = 0; i<100; i++ ) {
        for( int j = 0; j<N_SENSORS; j++ ) {
            sensors.direction[j] = 65.0+ 60.0*rand()/RAND_MAX;
            sensors.distance[j] = 45.0;
        }
        grid_update(&map, &sensors, &robot);
    }
    
    for( int i = 0; i<100; i++ ) {
        for( int j = 0; j<N_SENSORS; j++ ) {
            sensors.direction[j] = 45.0+ 30.0*rand()/RAND_MAX;
            sensors.distance[j] = 70.0;
        }
        grid_update(&map, &sensors, &robot);
    }
    
    int j = 3;
    for( int i = 0; i<map.width; i++) { map.cells[i*map.width+j] += 100; }
    for( int i = 0; i<map.width; i++) { map.cells[j*map.width+i] += 100; }
    
    j = map.height-3;
    for( int i = 0; i<map.width; i++) { map.cells[i*map.width+j] += 100; }
    for( int i = 0; i<map.width; i++) { map.cells[j*map.width+i] += 100; }
    
    /*
    
    // Print the grid
    for (int i=0; i<map.width; i++) {
        for (int j=0; j<map.height; j++) {
            if (map.cells[i * map.width + j] > 0) {
                sprintf(serial_output, "X");
                UART_PutString(serial_output);
            } else {
                sprintf(serial_output, "-");
                UART_PutString(serial_output);
            }
        }
        sprintf(serial_output, "\n");
        UART_PutString(serial_output);
    }
    */ 
    int print_delay = 3;
    int print_cnt = 1;
    for(;;) {  
            
        // Calculate distance to the goal
        double dist_to_goal = calculate_distance_from_goal(robot.x, robot.y, robot.goal_x, robot.goal_y);
        robot.desired_v = 5;
        // Check if goal is reached, update, otherwise, drive
        if( dist_to_goal <= robot.goal_min_dist ) { 
            //robot.desired_v = 0;       // Stop the robot
            //robot.desired_theta = 0;
            
            // Iterate to next goal, otherwise, quit
            if (goals_reached < n_goals) {
                robot.goal_x = goals[goals_reached + 2];
                robot.goal_y = goals[goals_reached + 3];
                goals_reached += 2;
                //sprintf(serial_output, "Goal:%Lf, %Lf", robot.goal_x, robot.goal_y);
                //UART_PutString(serial_output);
                //sprintf(serial_output, "Position:%Lf, %Lf, %Lf", robot.goal_x, robot.goal_y, robot.theta);
                //UART_PutString(serial_output);
     
                
                // Get robot to point to next destination to begin
                //double angle_to_goal = calculate_goal_angle(robot.x, robot.y, robot.theta, robot.goal_x, robot.goal_y);
                //robot.desired_theta = angle_to_goal;
            } else {
                sprintf(serial_output, "FINISHED! Did I succeed?");
                UART_PutString(serial_output);
                CyDelay(200000);
            }

        } else {
            
            
            
            if (print_cnt >= print_delay) {
                
                for (int j= map.height - 1; j >= 0; j--) {
                    for (int i=0; i<map.width; i++) {
                        if ( abs(floor(robot.x/map.resolution)-i)<=1 && abs(floor(robot.y/map.resolution)-j)<=1 ) {
                            serial_output[0] = 'X';
                            serial_output[1] = ' ';
                        } else if (abs(floor(robot.goal_x/map.resolution)-i)<=1 && abs(floor(robot.goal_y/map.resolution)-j)<=1 ) {
                            serial_output[0] = 'G';
                            serial_output[1] = ' ';
                        } else {
                            sprintf(serial_output, "%d ", map.cells[i * map.width + j]);
                        }
                        UART_PutString(serial_output);
                    }
                    sprintf(serial_output, "\n");
                    UART_PutString(serial_output);
                }
                UART_PutString("\n\n");    
            }
            
          
            
            
            // Update active window
            active_window(&map, &active, &robot);
            smoothed_POD_histogram(&smoothed_POD, &active, alpha, coeff_l, coeff_a, coeff_b);

            // Loop through densities and select candidate positions
            for (int i = 0; i < smoothed_POD.nsectors; i++) {
                double val = smoothed_POD.density[i];
                if (val < valley_threshold_lower) {
                  candidate_idx[i] = 0;
                } else if (val > valley_threshold_upper) {
                  candidate_idx[i] = 1;
                }
                //sprintf(serial_output, "%.2f ", val);
                //UART_PutString(serial_output);
            }
            //UART_PutString("\n\n\n\n");
            
            // Calculate angle of drive - Output is in degrees, not rad
            ideal_angle = calculate_avoidance_angle2(&smoothed_POD, &robot, candidate_idx, alpha, s_max);
            //sprintf(serial_output, "Trasjectory of travel %f \n", ideal_angle);
            //UART_PutString(serial_output);

            // Update Robot commands and free memory
            ideal_angle = ideal_angle * DEG2RAD;
            ideal_velocity = velocity_control(&smoothed_POD, ideal_angle, alpha, h_m);
            robot.x += 0.25*10*cos(ideal_angle);
            robot.y += 0.25*10*sin(ideal_angle);
            
            if (print_cnt >= print_delay) {
                //sprintf(serial_output, "Angle: %f, Speed: %f\n", ideal_angle*180/M_PI, ideal_velocity);
                //UART_PutString(serial_output);
                print_cnt=0;
            }
            print_cnt++;
         
            robot.desired_theta = ideal_angle;
            //robot.desired_v = ideal_velocity; 
            
            /*
            if (goals_reached == 0 ) {
                robot.x = 60;
                robot.y = 60;
            } else if (goals_reached == 2 ) {
                robot.x = 0;
                robot.y = 60;
            }
            */
        }

    } 
            
}

// Drive functions
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

/* [] END OF FILE */
