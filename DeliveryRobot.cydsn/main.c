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

struct Motor {
    float duty_cycle;
    double int_error;  // integrated error
    double w; // omega, [rad per sec]
    double desired_w;
    double Ki;
    double Kp;
    double wheel_radius; // wheel radius in cm
    int32 enc_count;
};

struct Robot {
    float theta;
    float x;
    float y;
    float axle_width; // in cm
};

const float PULSES_PER_REV = 3591.92;
const float MOTOR_CONTROL_PERIOD = 0.02; // seconds
const double TWO_PI = 2*3.14159265358979;

void Drive_Left_Motor(double duty_cycle);
void Drive_Right_Motor(double duty_cycle);

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
    // read centimeters
    echo_distance = Timer_Echo_ReadCapture();
    echo_flag = 1;
    Timer_Echo_WriteCounter(max_count); // TODO: check if this is needed next lab
    // clear the interrupt
    Timer_Echo_ReadStatusRegister();
}

CY_ISR( Wheel_Vel_Int_Handler ) {
    int32 new = QuadDec_L_GetCounter();
    int32 diff = new - left_motor.enc_count;
    left_motor.enc_count = new;
    left_motor.w = TWO_PI*diff/MOTOR_CONTROL_PERIOD/PULSES_PER_REV;
    
    new = QuadDec_R_GetCounter();
    diff = new - right_motor.enc_count;
    right_motor.enc_count = new;
    right_motor.w = TWO_PI*diff/MOTOR_CONTROL_PERIOD/PULSES_PER_REV;
    
    // TODO: Add localisation update
}

CY_ISR( Motor_PI_Int_Handler ) {
    double error = left_motor.desired_w - left_motor.w;
    left_motor.int_error  = left_motor.int_error + error;
    left_motor.duty_cycle = left_motor.duty_cycle + left_motor.Kp*error + left_motor.Ki*left_motor.int_error;
    Drive_Left_Motor(left_motor.duty_cycle);
    
    error = right_motor.desired_w - right_motor.w;
    right_motor.int_error  = right_motor.int_error + error;
    right_motor.duty_cycle = right_motor.duty_cycle + right_motor.Kp*error + right_motor.Ki*right_motor.int_error;
    Drive_Right_Motor(right_motor.duty_cycle);
}



int main(void)
{
    left_motor.duty_cycle = 0;
    left_motor.int_error = 0;
    left_motor.desired_w = 0.3;
    left_motor.wheel_radius = 2.5;
    left_motor.enc_count = 0;
    left_motor.Ki = 3e-7;  // TODO: determine good PI params
    left_motor.Kp = 0.005;
    
    right_motor.duty_cycle = 0;
    right_motor.int_error = 0;
    right_motor.desired_w = 0.3;
    right_motor.wheel_radius = 2.5;
    right_motor.enc_count = 0;
    right_motor.Ki = 3e-7;  // TODO: determine good PI params
    right_motor.Kp = 0.005;
    
    CyGlobalIntEnable; /* Enable global interrupts. */

    // Start up code - enable UART, PWM and Timer used for ultrasonic module
    UART_Start();
    Timer_Echo_Start();
    PWM_Trigger_Start();
    QuadDec_L_Start();
    PWM_Motor_L_Start();
    QuadDec_R_Start();
    PWM_Motor_R_Start();
    
    // Registration of Timer ISR
    Timer_Echo_Int_StartEx( Timer_Int_Handler );
    Wheel_Vel_Int_StartEx( Wheel_Vel_Int_Handler );
    Motor_PI_Int_StartEx( Motor_PI_Int_Handler );
    


    for(;;) {
        sprintf(serial_output, "desired: %lf, actual: %lf, dc:%lf\n", left_motor.desired_w,left_motor.w, left_motor.duty_cycle);
        UART_PutString(serial_output);
        // if a distance was measured, print the distance and clear the flag
        while ( echo_flag == 1 ) {
            //sprintf(serial_output, "%d cm", 65535-echo_distance);
            //UART_PutString(serial_output);
            //UART_PutCRLF(0x0D);
            
            Timer_Echo_Stop();
            CyDelayUs(1); // TODO: Should be able to make this shorter, one or two bus clock cycles
            if (mux_select == 0) {
                mux_select = 1;
            } else {
                mux_select = 0;
            }
            Control_Reg_US_Write(mux_select);
            Timer_Echo_Enable();
            PWM_Trigger_WriteCounter(1);


            CyGlobalIntDisable; // Disable global interrupts, so the flag gets cleared. 
            echo_flag = 0;
            CyGlobalIntEnable; // Enable global interrupts after the flag is cleared. 
    }
 }
  
}

void Drive_Left_Motor(double duty_cycle) {
    if (duty_cycle < -1) {
        duty_cycle = -1;
    } else if (duty_cycle > 1) {
        duty_cycle = 1;
    }
    
    if (duty_cycle < 0) {
        duty_cycle = -duty_cycle;
        PWM_Motor_L_WriteCompare1(0);
        PWM_Motor_L_WriteCompare2(duty_cycle*65535);
    } else {
        PWM_Motor_L_WriteCompare1(duty_cycle*65535);
        PWM_Motor_L_WriteCompare2(0);
    }
}

void Drive_Right_Motor(double duty_cycle) {
    if (duty_cycle < -1) {
        duty_cycle = -1;
    } else if (duty_cycle > 1) {
        duty_cycle = 1;
    }
    
    if (duty_cycle < 0) {
        duty_cycle = -duty_cycle;
        PWM_Motor_R_WriteCompare1(0);
        PWM_Motor_R_WriteCompare2(duty_cycle*65535);
    } else {
        PWM_Motor_R_WriteCompare1(duty_cycle*65535);
        PWM_Motor_R_WriteCompare2(0);
    }
}

/* [] END OF FILE */
