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

#include "vfh.h"
//#include "polar_histogram.h"
//#include "histogram_grid.h"

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

  
CY_ISR( Timer_Int_Handler ) {
    echo_flag = 1;
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
    
    robot.V = robot.desired_V *( 1 - logl( (M_E-1) * fabsl(error) / M_PI + 1 )); // scales velocity depending on how much we have to rotate (makes robot turn on spot more)
    
    robot.int_error = robot.int_error + error;
    long double new_omega = robot.Kp * error + robot.Ki * robot.int_error;
    right_motor.desired_w = (robot.V + new_omega * robot.axle_width / 2) / right_motor.wheel_radius;
    left_motor.desired_w = (robot.V - new_omega * robot.axle_width / 2) / left_motor.wheel_radius;   
    
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
    angle = robot.desired_theta + M_PI/2;
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
    
    robot.goal_min_dist = 1;
    robot.goal_x = 100;
    robot.goal_y = -100;
    
    CyGlobalIntEnable;
    
    // Registration of Timer ISR
    //Timer_Echo_Int_StartEx( Timer_Int_Handler );
    //Pose_Update_Int_StartEx( Pose_Update_Int_Handler );
    //Motor_PI_Int_StartEx( Motor_PI_Int_Handler );
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
    
    /* Declaration of the needed data structures. */
	grid * certainty_grid;
	sensor_data sensors;
	histogram * polar_histogram;
	control_signal_t control_signal;
        
    certainty_grid = initial_grid(65, 65, 2);
    polar_histogram = initial_histogram(5, 20, 10, 5);
        
    /* Are the initializations ok? */
	if (certainty_grid == NULL) return -1;
	if (certainty_grid->cells == NULL) return -1;
	if (polar_histogram == NULL) return -1;
	if (polar_histogram->densities == NULL) return -1;
    
    /*
	** Fake measures.
	*/
    for (int i = 0; i < 1000; ++i) {
        for( int j = 0; j<4; j++) {
            sensors.direction[j] = (int) ((40.0 * rand()) / RAND_MAX + 90.0); /* [degrees] */
        	sensors.distance[j] = (u_long) (((65.0 * rand()) / RAND_MAX)+5.0); /* [cm] */
        }
        for( int j = 4; j<8; j++) {
        	sensors.direction[j] = (int) ((40.0 * rand()) / RAND_MAX); /* [degrees] */
        	sensors.distance[j] = (u_long) (((65.0 * rand()) / RAND_MAX)+5.0); /* [cm] */
        }
        update_grid(certainty_grid, 65, 65, 0, sensors);
    }    
    
    for(;;) {
        
        for( int y=certainty_grid->height-1; y >= 0; --y ) {
            for( int x=0; x < certainty_grid->width; ++x ) {
                sprintf(serial_output, "%lu ", certainty_grid->cells[x*certainty_grid->width+y] );
                UART_PutString(serial_output);
            }
            UART_PutString("\n");
        }
        UART_PutString("\n\n\n\n\n");
        
        
        grid * active = active_window(certainty_grid,0,0,5);
        
        
        for( int y=active->height-1; y >= 0; --y ) {
            for( int x=0; x < active->width; ++x ) {
                sprintf(serial_output, "%lu ", active->cells[x*active->width+y] );
                UART_PutString(serial_output);
            }
            UART_PutString("\n");
        }
        UART_PutString("\n\n\n\n\n");
        
        
        hist_update(polar_histogram, certainty_grid);
        
        
        for( int i=0; i<polar_histogram->sectors; i++ ) {
            sprintf(serial_output, "%lf\n", polar_histogram->densities[i]);
            UART_PutString(serial_output);
        }
        UART_PutString("\n\n\n\n\n");
        
        
        double dir = calculate_direction2(polar_histogram, 45);
        
        sprintf(serial_output, "best_dir: %lf\n", dir);
        UART_PutString(serial_output);
        
        
        /*
        long double dy = robot.goal_y - robot.y;
        long double dx = robot.goal_x - robot.x;
        long double dist_to_goal = sqrtl( dy*dy + dx*dx );
        long double theta_to_goal = atan2l( dy, dx );
        
        if( dist_to_goal <= robot.goal_min_dist ) {
            robot.desired_V = 0;
        } else if( echo_flag && echo_distance < 40 && echo_distance <= dist_to_goal) {
            robot.desired_theta = angle_modulo( robot.theta - M_PI/5 );
            Timer_Avoidance_WriteCounter(65535);
        } else if( Timer_Avoidance_ReadCounter() < 65532 ){
            robot.desired_theta = theta_to_goal;
            if( dist_to_goal < 10 ) {
                robot.desired_V = 2;
            } else {
                robot.desired_V = 10;
            }
        }
        */
        
        // if a distance was measured, print the distance and clear the flag
        if ( echo_flag == 1 ) {
            echo_distance = 65535 - Timer_Echo_ReadCapture();  // in cm
            
            /*
            Timer_Echo_Stop();
            //CyDelayUs(1); // TODO: Should be able to make this shorter, one or two bus clock cycles
            if (mux_select == 0) {
                mux_select = 1;
            } else {
                mux_select = 0;
            }
            mux_select = 0;
            Control_Reg_US_Write(mux_select);
            Timer_Echo_Enable();
            */
            PWM_Trigger_WriteCounter(255);    

            CyGlobalIntDisable; // Disable global interrupts, so the flag gets cleared. 
            echo_flag = 0;
            CyGlobalIntEnable; // Enable global interrupts after the flag is cleared. 
        }
        
        //sprintf(serial_output, "dx: %Lf, dy: %Lf, dtg: %Lf, ttg: %Lf, dist: %i, tmr: %i\n", dx, dy, dist_to_goal, theta_to_goal, echo_distance, Timer_Avoidance_ReadCounter());
        //sprintf(serial_output, "desired: %lf, actual: %lf, dc:%lf, enc: %li\n", right_motor.desired_w,right_motor.w, right_motor.duty_cycle, QuadDec_R_GetCounter());
        //sprintf(serial_output, "x: %lf, y: %lf, theta: %lf\n", robot.x, robot.y, robot.theta);
        //UART_PutString(serial_output);
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

/* [] END OF FILE */
