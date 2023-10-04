/*******************************************************************************
* File Name: PWM_ServoDir_PM.c
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

#include "PWM_ServoDir.h"

static PWM_ServoDir_backupStruct PWM_ServoDir_backup;


/*******************************************************************************
* Function Name: PWM_ServoDir_SaveConfig
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
*  PWM_ServoDir_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void PWM_ServoDir_SaveConfig(void) 
{

    #if(!PWM_ServoDir_UsingFixedFunction)
        #if(!PWM_ServoDir_PWMModeIsCenterAligned)
            PWM_ServoDir_backup.PWMPeriod = PWM_ServoDir_ReadPeriod();
        #endif /* (!PWM_ServoDir_PWMModeIsCenterAligned) */
        PWM_ServoDir_backup.PWMUdb = PWM_ServoDir_ReadCounter();
        #if (PWM_ServoDir_UseStatus)
            PWM_ServoDir_backup.InterruptMaskValue = PWM_ServoDir_STATUS_MASK;
        #endif /* (PWM_ServoDir_UseStatus) */

        #if(PWM_ServoDir_DeadBandMode == PWM_ServoDir__B_PWM__DBM_256_CLOCKS || \
            PWM_ServoDir_DeadBandMode == PWM_ServoDir__B_PWM__DBM_2_4_CLOCKS)
            PWM_ServoDir_backup.PWMdeadBandValue = PWM_ServoDir_ReadDeadTime();
        #endif /*  deadband count is either 2-4 clocks or 256 clocks */

        #if(PWM_ServoDir_KillModeMinTime)
             PWM_ServoDir_backup.PWMKillCounterPeriod = PWM_ServoDir_ReadKillTime();
        #endif /* (PWM_ServoDir_KillModeMinTime) */

        #if(PWM_ServoDir_UseControl)
            PWM_ServoDir_backup.PWMControlRegister = PWM_ServoDir_ReadControlRegister();
        #endif /* (PWM_ServoDir_UseControl) */
    #endif  /* (!PWM_ServoDir_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: PWM_ServoDir_RestoreConfig
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
*  PWM_ServoDir_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void PWM_ServoDir_RestoreConfig(void) 
{
        #if(!PWM_ServoDir_UsingFixedFunction)
            #if(!PWM_ServoDir_PWMModeIsCenterAligned)
                PWM_ServoDir_WritePeriod(PWM_ServoDir_backup.PWMPeriod);
            #endif /* (!PWM_ServoDir_PWMModeIsCenterAligned) */

            PWM_ServoDir_WriteCounter(PWM_ServoDir_backup.PWMUdb);

            #if (PWM_ServoDir_UseStatus)
                PWM_ServoDir_STATUS_MASK = PWM_ServoDir_backup.InterruptMaskValue;
            #endif /* (PWM_ServoDir_UseStatus) */

            #if(PWM_ServoDir_DeadBandMode == PWM_ServoDir__B_PWM__DBM_256_CLOCKS || \
                PWM_ServoDir_DeadBandMode == PWM_ServoDir__B_PWM__DBM_2_4_CLOCKS)
                PWM_ServoDir_WriteDeadTime(PWM_ServoDir_backup.PWMdeadBandValue);
            #endif /* deadband count is either 2-4 clocks or 256 clocks */

            #if(PWM_ServoDir_KillModeMinTime)
                PWM_ServoDir_WriteKillTime(PWM_ServoDir_backup.PWMKillCounterPeriod);
            #endif /* (PWM_ServoDir_KillModeMinTime) */

            #if(PWM_ServoDir_UseControl)
                PWM_ServoDir_WriteControlRegister(PWM_ServoDir_backup.PWMControlRegister);
            #endif /* (PWM_ServoDir_UseControl) */
        #endif  /* (!PWM_ServoDir_UsingFixedFunction) */
    }


/*******************************************************************************
* Function Name: PWM_ServoDir_Sleep
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
*  PWM_ServoDir_backup.PWMEnableState:  Is modified depending on the enable
*  state of the block before entering sleep mode.
*
*******************************************************************************/
void PWM_ServoDir_Sleep(void) 
{
    #if(PWM_ServoDir_UseControl)
        if(PWM_ServoDir_CTRL_ENABLE == (PWM_ServoDir_CONTROL & PWM_ServoDir_CTRL_ENABLE))
        {
            /*Component is enabled */
            PWM_ServoDir_backup.PWMEnableState = 1u;
        }
        else
        {
            /* Component is disabled */
            PWM_ServoDir_backup.PWMEnableState = 0u;
        }
    #endif /* (PWM_ServoDir_UseControl) */

    /* Stop component */
    PWM_ServoDir_Stop();

    /* Save registers configuration */
    PWM_ServoDir_SaveConfig();
}


/*******************************************************************************
* Function Name: PWM_ServoDir_Wakeup
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
*  PWM_ServoDir_backup.pwmEnable:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void PWM_ServoDir_Wakeup(void) 
{
     /* Restore registers values */
    PWM_ServoDir_RestoreConfig();

    if(PWM_ServoDir_backup.PWMEnableState != 0u)
    {
        /* Enable component's operation */
        PWM_ServoDir_Enable();
    } /* Do nothing if component's block was disabled before */

}


/* [] END OF FILE */
