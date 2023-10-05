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
#include "..\Helper\helper.h"
#include "..\Robot\robot.h"

/* Define all global variables. */
#define N_SENSORS 6     // Number of Ultrasonic Sensors

#ifndef M_PI    // Pi, duh!
#define M_PI 3.141592653589793238462643383279502884196      
#endif

#ifndef M_TWOPI     // 2*Pi, duh!
#define M_TWOPI 6.2831853071795862319959        
#endif

#ifndef M_E     // Exponential, duh!
#define M_E 2.71828182845904523536
#endif

const double PULSES_PER_REV = 3591.92;
const double POSE_UPDATE_PERIOD = 1.0/50.0; // seconds

uint8_t echo_flag = 0;          // Ultrasonic flag
uint16_t max_count = 65535;     // Ultrasonic time count
uint16_t echo_distance;         // Ultrasonic distance
uint8_t mux_select = 0;         // For selecting specific ultrasonic sensor

int32 left_wheel_count = 0;
int32 right_wheel_count = 0;
char serial_output[150];        // For UART print output
int wall_following_flag = 0;

/* Defining/Creating all data structures*/
Motor left_motor;     // Left Motor, duh!
Motor right_motor;    // Right Motor, duh!
Robot robot;          // Robot values, duh!
Sensor sensors;       // Ultrasonics

void Drive_Left_Motor(long double duty_cycle);
void Drive_Right_Motor(long double duty_cycle);
void Turn_Delay(long double angle);
void move_servo(int servo_nums);

/* Interrupt to obtain Ultrasonic measurement value. */  
CY_ISR( Timer_Int_Handler ) {
    // Collect measurement 
    echo_distance = max_count - Timer_Echo_ReadCapture();   // in cm
    sensors.distance[mux_select] = echo_distance;      // Store measured value
    mux_select++;   // Iterate the global ultrasonic tracker

    // Reset the global ultrasonic tracker when all measurements have been updated
    if( mux_select == N_SENSORS ) { 
        mux_select = 0; 
        wall_following_flag = 1;
    }

    Control_Reg_US_Write(mux_select);
    PWM_Trigger_WriteCounter(255);    
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
    PWM_ServoDir_Start();
    PWM_1_Start();
    
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
    long double robot_Kp = 0.75;     // was previously 0.75 before changing for MS1
    long double min_distance = 5;   // Minimum distance between robot position and goal


    /*======================= ROBOT STARTING POSITION =======================*/
    long double start_x = 0;    // Starting x, duh!
    long double start_y = 0;    // Starting y, duh!
    long double start_th = M_PI/2;
    /*=======================================================================*/


    robot_create(&robot, robot_axle_width, robot_Ki, robot_Kp, min_distance, start_x, start_y, start_th);

    // Define sensor directions (start from front sensor, then move clockwise)
    sensors.direction[0] = 0;
    sensors.direction[1] = 30;
    sensors.direction[2] = 90;
    sensors.direction[3] = 270;
    sensors.direction[4] = 330;

    /*========================= M1: Goal Definition =========================*/
    // Goals should be defined where the bin is
    int n_goals = 2;    // Number of goals, duh!
    int goals[4] = {0, 90, 90, 90};    // Coordinates of goals [x1, y1, x2, y2, ..., xn, yn]
    robot.goal_x = goals[0];   // Update robot x goal
    robot.goal_y = goals[1];   // Update robot y goal
    int goals_reached = 0;  // Counter for number of goas reached, duh!
    /*=======================================================================*/    

    

    /*======================= M1: Wall Following Code =======================*/
    // Point to goal at the beginning
    float ref_direction = M_PI/2;
    int ref_direction_deg = 90;
    robot.desired_theta = M_PI/2;
    
    // KP Controls
    float theta_correction = 0;
    float wall_Kp = 0.01;
   
    // Thresholds
    float front_dist_th = 50;
    float dist_ref = 50;
    int front_count = 0;
    
    // Flags
    int return_flag = 0;
    int B_flag = 0;
    
    // Settings
    int velocity = 15;
    
    /*=======================================================================*/  
    // starts at front left and goes clockwise
    //move_servo(3, 1); 
    //move_servo(0, 0);
    //move_servo(1, 1);
    //move_servo(2, 0);
    
    
    for(;;) {  
        // Wall follow only after sensor is updated
        // Read 3 times
        if ( wall_following_flag ) {
            if (sensors.distance[0] < front_dist_th && sensors.distance[5] < front_dist_th) {                
                switch (ref_direction_deg) {
                    // Travelling towards box A
                    case (90):
         
                        // Stop moving
                        robot.desired_v = 0;

                        // TODO: Unload Package Code
                        move_servo(0b0101); // move servos 3 and 1 simultaneously
                        //
                        
                        // Turn towards box B
                        ref_direction = calculate_angle_modulo(robot.theta - M_PI/2);
                        Turn_Delay(ref_direction);
                        
                        
                        ref_direction_deg = angle_clamp(ref_direction_deg - 90);
                        robot.desired_v = velocity;
                        
                        // Update Flags
                        wall_following_flag = 0;
                        front_dist_th = 450;
                        
                        // Spoof
                        sensors.distance[1] = dist_ref;
                        sensors.distance[2] = dist_ref;
                        sensors.distance[3] = dist_ref;
                        sensors.distance[4] = dist_ref;
                        sensors.distance[0] = 10000;
                        sensors.distance[5] = 10000;
                        
                        break;
                        
                    
                    // Travelling towards box B
                    case (0):
                        // If we haven't stopped at B yet
                        if (!B_flag) {
                            // Stop moving
                            robot.desired_v = 0;
                            
                            // Rotate -90 deg to deliver packages
                            ref_direction = calculate_angle_modulo(robot.theta - M_PI/2);
                            Turn_Delay(ref_direction);

                            // TODO: Unload Package Code
                            move_servo(0b0001);                                      
                            
                            //
                            
                            // Rotate back to go to C
                            ref_direction = calculate_angle_modulo(robot.theta + M_PI/2);
                            Turn_Delay(ref_direction);
                            front_dist_th = 50;
                            B_flag = 1;
                            robot.desired_v = velocity;
                            
                        } else {
                            // Stop moving
                            robot.desired_v = 0;
                            
                            // Rotate -90 deg to deliver packages
                            ref_direction = calculate_angle_modulo(robot.theta - M_PI/2);
                            Turn_Delay(ref_direction);

                            // TODO: Unload Package Code
                            move_servo(0b0010);
                            //
                            
                            // Go back to A
                            ref_direction = calculate_angle_modulo(robot.theta - M_PI/2);
                            Turn_Delay(ref_direction);
                            ref_direction_deg = angle_clamp(ref_direction_deg - 180);
                            robot.desired_v = velocity;
                            
                            // Update Flags
                            wall_following_flag = 0;
                            return_flag = 1;
                            B_flag = 0;
                            
                        }
                        
                        // Spoof
                        sensors.distance[1] = dist_ref;
                        sensors.distance[2] = dist_ref;
                        sensors.distance[3] = dist_ref;
                        sensors.distance[4] = dist_ref;
                        sensors.distance[0] = 10000;
                        sensors.distance[5] = 10000;
                        break;
                    
                    // Travelling Back to A *Can include a flag for safety measures
                    case (180):

                        // Stop moving and turn towards loading bay
                        robot.desired_v = 0;

                        // Go towards beginning
                        ref_direction = calculate_angle_modulo(robot.theta + M_PI/2);
                        Turn_Delay(ref_direction);
                        ref_direction_deg = angle_clamp(ref_direction_deg + 90);
                        robot.desired_v = velocity;
                        wall_following_flag = 0;
                    
                        // Spoof
                        sensors.distance[1] = dist_ref;
                        sensors.distance[2] = dist_ref;
                        sensors.distance[3] = dist_ref;
                        sensors.distance[4] = dist_ref;
                        sensors.distance[0] = 10000;
                        sensors.distance[5] = 10000;
                        break;

                    // Travelling Back to Start
                    case (270):
                         
                        // Stop moving and point back to A
                        robot.desired_v = 0;

                        // Update Position
                        ref_direction = calculate_angle_modulo(robot.theta - M_PI);
                        Turn_Delay(ref_direction);
                        ref_direction_deg = angle_clamp(ref_direction_deg - 180);
                        robot.desired_v = velocity;
                        
                        // Update Flags
                        wall_following_flag = 0;
                        return_flag = 0;
                            
                        // Spoof
                        sensors.distance[1] = dist_ref;
                        sensors.distance[2] = dist_ref;
                        sensors.distance[3] = dist_ref;
                        sensors.distance[4] = dist_ref;
                        sensors.distance[0] = 10000;
                        sensors.distance[5] = 10000;
                        
                        break;

                    default:
                        
                        // Wall follow if broken
                        robot.desired_v = 0;
                        Turn_Delay(M_PI/2);
                        ref_direction = calculate_angle_modulo(ref_direction - M_PI/2);
                        
                        robot.desired_v = velocity;
                        wall_following_flag = 0;

                        // Spoof sensor to avoid sensor updates during turn
                        sensors.distance[1] = dist_ref;
                        sensors.distance[2] = dist_ref;
                        sensors.distance[3] = dist_ref;
                        sensors.distance[4] = dist_ref;
                        sensors.distance[0] = 10000;
                        sensors.distance[5] = 10000;
                }
            }

            // Wall Following
            float error = 0;
            int terminal_phase = sensors.distance[0] < 100 && sensors.distance[5] < 150;
            switch ( return_flag ){
                case (0):
                    // Follow Left Wall
                    robot.desired_v = terminal_phase ? 5: velocity;
                    error = (sensors.distance[1] < sensors.distance[2]) ? dist_ref - sensors.distance[1] : dist_ref - sensors.distance[2];
                    //error = dist_ref - (sensors.distance[1] + sensors.distance[2] / 2);
                    theta_correction = wall_Kp * -(error);
                    robot.desired_theta = ref_direction + theta_correction;
                    wall_following_flag = 0;
                    
                    break;    
                
                case (1):
                    // Follow right wall
                    robot.desired_v = terminal_phase ? 5: velocity;
                    
                    error = (sensors.distance[3] < sensors.distance[4]) ? dist_ref - sensors.distance[3] : dist_ref - sensors.distance[4];
                    
                    theta_correction = wall_Kp * -(error);
                    robot.desired_theta = ref_direction - theta_correction;
                    wall_following_flag = 0;
                    
                    break;  
                    
                default:
                    robot.desired_v = 0;
                    robot.desired_theta = 190000;
                 
            }
            
            
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

void Turn_Delay(long double angle) {
    // Set desired turn in radians
    robot.desired_theta = angle;
    
    // Idle loop to wait until turn is complete
    while( !( fabs(robot.theta-robot.desired_theta) < 0.2 ) ) {}; 
}

int velocity_control(int max_velocity) {
    // Adjust the velocity the closer we approach the walls
    int avg_dist = (sensors.distance[0] + sensors.distance[1]) / 2;
    int thresh = 1;
    int velocity = floor(max_velocity * (1200 - avg_dist) / 400);
}

void move_servo(int servo_nums) {
    CyDelay(100);
    PWM_ServoDir_WriteCompare1(2000);
    PWM_ServoDir_WriteCompare2(4000);
    Control_Reg_ServoSelect_Write(servo_nums);
    Control_Reg_ServoTrigger_Write(1);
    
    CyDelay(2550);
    
    PWM_ServoDir_WriteCompare1(4000);
    PWM_ServoDir_WriteCompare2(2000);
    Control_Reg_ServoTrigger_Write(0);
}
/* [] END OF FILE */
