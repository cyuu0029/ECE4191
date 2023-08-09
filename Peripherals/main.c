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
    float error;
    float rps;
    int wheel_radius; // wheel radius in cm
};

void Drive_Left_Motor(float duty_cycle);
  
uint8_t echo_flag = 0;
uint16_t max_count = 65535;
uint16_t echo_distance;
uint8_t mux_select = 0;
int32 left_wheel_count = 0;
int32 right_wheel_count = 0;
char serial_output[150];
float duty_cyle= -1;
struct Motor left_motor;
struct Motor right_motor;


  
CY_ISR( Timer_Int_Handler ) {
    // read centimeters
    echo_distance = Timer_Echo_ReadCapture();
    echo_flag = 1;
    Timer_Echo_WriteCounter(max_count);
    // clear the interrupt
    Timer_Echo_ReadStatusRegister();
}

CY_ISR( Wheel_Vel_Int_Handler ) {
    int32 left_new = QuadDec_L_GetCounter();
    int32 left_diff = left_new - left_wheel_count;
    left_wheel_count = left_new;
    left_motor.rps = (left_diff/0.02)/3591.92;
    sprintf(serial_output, "Test: %f", left_motor.rps);
    UART_PutString(serial_output);
    UART_PutCRLF(0x0D);
}



int main(void)
{
    left_motor.duty_cycle = 0;
    left_motor.error = 0;
    left_motor.rps = 0;
    left_motor.wheel_radius = 5;
    
    
    CyGlobalIntEnable; /* Enable global interrupts. */

    // Start up code - enable UART, PWM and Timer used for ultrasonic module
    UART_Start();
    Timer_Echo_Start();
    PWM_Trigger_Start();
    QuadDec_L_Start();
    PWM_Motor_L_Start();

    // Registration of Timer ISR
    Timer_Echo_Int_StartEx( Timer_Int_Handler );
    Wheel_Vel_Int_StartEx( Wheel_Vel_Int_Handler );

    


    for(;;) {
        duty_cyle = duty_cyle + 0.1;
        Drive_Left_Motor(duty_cyle);
        CyDelay(100);
        
        // if a distance was measured, print the distance and clear the flag
        while ( echo_flag == 1 ) {
            //sprintf(serial_output, "%d cm", 65535-echo_distance);
            //UART_PutString(serial_output);
            //UART_PutCRLF(0x0D);
            
            Timer_Echo_Stop();
            CyDelayUs(1);
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

void Drive_Left_Motor(float duty_cycle) {
    if (duty_cycle < 0) {
        duty_cycle = -duty_cycle;
        PWM_Motor_L_WriteCompare1(0);
        PWM_Motor_L_WriteCompare2(duty_cycle*10000);
    } else {
        PWM_Motor_L_WriteCompare1(duty_cycle*10000);
        PWM_Motor_L_WriteCompare2(0);
    }
}

/* [] END OF FILE */
