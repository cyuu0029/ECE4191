/*******************************************************************************
* File Name: PWM_Motor_L_PM.c
* Version 3.30
*
* Description:
*  This file provides the power management source code to API for the
*  PWM.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "PWM_Motor_L.h"

static PWM_Motor_L_backupStruct PWM_Motor_L_backup;


/*******************************************************************************
* Function Name: PWM_Motor_L_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration of the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  PWM_Motor_L_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void PWM_Motor_L_SaveConfig(void) 
{

    #if(!PWM_Motor_L_UsingFixedFunction)
        #if(!PWM_Motor_L_PWMModeIsCenterAligned)
            PWM_Motor_L_backup.PWMPeriod = PWM_Motor_L_ReadPeriod();
        #endif /* (!PWM_Motor_L_PWMModeIsCenterAligned) */
        PWM_Motor_L_backup.PWMUdb = PWM_Motor_L_ReadCounter();
        #if (PWM_Motor_L_UseStatus)
            PWM_Motor_L_backup.InterruptMaskValue = PWM_Motor_L_STATUS_MASK;
        #endif /* (PWM_Motor_L_UseStatus) */

        #if(PWM_Motor_L_DeadBandMode == PWM_Motor_L__B_PWM__DBM_256_CLOCKS || \
            PWM_Motor_L_DeadBandMode == PWM_Motor_L__B_PWM__DBM_2_4_CLOCKS)
            PWM_Motor_L_backup.PWMdeadBandValue = PWM_Motor_L_ReadDeadTime();
        #endif /*  deadband count is either 2-4 clocks or 256 clocks */

        #if(PWM_Motor_L_KillModeMinTime)
             PWM_Motor_L_backup.PWMKillCounterPeriod = PWM_Motor_L_ReadKillTime();
        #endif /* (PWM_Motor_L_KillModeMinTime) */

        #if(PWM_Motor_L_UseControl)
            PWM_Motor_L_backup.PWMControlRegister = PWM_Motor_L_ReadControlRegister();
        #endif /* (PWM_Motor_L_UseControl) */
    #endif  /* (!PWM_Motor_L_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: PWM_Motor_L_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration of the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  PWM_Motor_L_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void PWM_Motor_L_RestoreConfig(void) 
{
        #if(!PWM_Motor_L_UsingFixedFunction)
            #if(!PWM_Motor_L_PWMModeIsCenterAligned)
                PWM_Motor_L_WritePeriod(PWM_Motor_L_backup.PWMPeriod);
            #endif /* (!PWM_Motor_L_PWMModeIsCenterAligned) */

            PWM_Motor_L_WriteCounter(PWM_Motor_L_backup.PWMUdb);

            #if (PWM_Motor_L_UseStatus)
                PWM_Motor_L_STATUS_MASK = PWM_Motor_L_backup.InterruptMaskValue;
            #endif /* (PWM_Motor_L_UseStatus) */

            #if(PWM_Motor_L_DeadBandMode == PWM_Motor_L__B_PWM__DBM_256_CLOCKS || \
                PWM_Motor_L_DeadBandMode == PWM_Motor_L__B_PWM__DBM_2_4_CLOCKS)
                PWM_Motor_L_WriteDeadTime(PWM_Motor_L_backup.PWMdeadBandValue);
            #endif /* deadband count is either 2-4 clocks or 256 clocks */

            #if(PWM_Motor_L_KillModeMinTime)
                PWM_Motor_L_WriteKillTime(PWM_Motor_L_backup.PWMKillCounterPeriod);
            #endif /* (PWM_Motor_L_KillModeMinTime) */

            #if(PWM_Motor_L_UseControl)
                PWM_Motor_L_WriteControlRegister(PWM_Motor_L_backup.PWMControlRegister);
            #endif /* (PWM_Motor_L_UseControl) */
        #endif  /* (!PWM_Motor_L_UsingFixedFunction) */
    }


/*******************************************************************************
* Function Name: PWM_Motor_L_Sleep
********************************************************************************
*
* Summary:
*  Disables block's operation and saves the user configuration. Should be called
*  just prior to entering sleep.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  PWM_Motor_L_backup.PWMEnableState:  Is modified depending on the enable
*  state of the block before entering sleep mode.
*
*******************************************************************************/
void PWM_Motor_L_Sleep(void) 
{
    #if(PWM_Motor_L_UseControl)
        if(PWM_Motor_L_CTRL_ENABLE == (PWM_Motor_L_CONTROL & PWM_Motor_L_CTRL_ENABLE))
        {
            /*Component is enabled */
            PWM_Motor_L_backup.PWMEnableState = 1u;
        }
        else
        {
            /* Component is disabled */
            PWM_Motor_L_backup.PWMEnableState = 0u;
        }
    #endif /* (PWM_Motor_L_UseControl) */

    /* Stop component */
    PWM_Motor_L_Stop();

    /* Save registers configuration */
    PWM_Motor_L_SaveConfig();
}


/*******************************************************************************
* Function Name: PWM_Motor_L_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration. Should be called just after
*  awaking from sleep.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  PWM_Motor_L_backup.pwmEnable:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void PWM_Motor_L_Wakeup(void) 
{
     /* Restore registers values */
    PWM_Motor_L_RestoreConfig();

    if(PWM_Motor_L_backup.PWMEnableState != 0u)
    {
        /* Enable component's operation */
        PWM_Motor_L_Enable();
    } /* Do nothing if component's block was disabled before */

}


/* [] END OF FILE */
