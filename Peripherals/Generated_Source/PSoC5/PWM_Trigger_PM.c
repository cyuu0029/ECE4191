/*******************************************************************************
* File Name: PWM_Trigger_PM.c
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

#include "PWM_Trigger.h"

static PWM_Trigger_backupStruct PWM_Trigger_backup;


/*******************************************************************************
* Function Name: PWM_Trigger_SaveConfig
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
*  PWM_Trigger_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void PWM_Trigger_SaveConfig(void) 
{

    #if(!PWM_Trigger_UsingFixedFunction)
        #if(!PWM_Trigger_PWMModeIsCenterAligned)
            PWM_Trigger_backup.PWMPeriod = PWM_Trigger_ReadPeriod();
        #endif /* (!PWM_Trigger_PWMModeIsCenterAligned) */
        PWM_Trigger_backup.PWMUdb = PWM_Trigger_ReadCounter();
        #if (PWM_Trigger_UseStatus)
            PWM_Trigger_backup.InterruptMaskValue = PWM_Trigger_STATUS_MASK;
        #endif /* (PWM_Trigger_UseStatus) */

        #if(PWM_Trigger_DeadBandMode == PWM_Trigger__B_PWM__DBM_256_CLOCKS || \
            PWM_Trigger_DeadBandMode == PWM_Trigger__B_PWM__DBM_2_4_CLOCKS)
            PWM_Trigger_backup.PWMdeadBandValue = PWM_Trigger_ReadDeadTime();
        #endif /*  deadband count is either 2-4 clocks or 256 clocks */

        #if(PWM_Trigger_KillModeMinTime)
             PWM_Trigger_backup.PWMKillCounterPeriod = PWM_Trigger_ReadKillTime();
        #endif /* (PWM_Trigger_KillModeMinTime) */

        #if(PWM_Trigger_UseControl)
            PWM_Trigger_backup.PWMControlRegister = PWM_Trigger_ReadControlRegister();
        #endif /* (PWM_Trigger_UseControl) */
    #endif  /* (!PWM_Trigger_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: PWM_Trigger_RestoreConfig
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
*  PWM_Trigger_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void PWM_Trigger_RestoreConfig(void) 
{
        #if(!PWM_Trigger_UsingFixedFunction)
            #if(!PWM_Trigger_PWMModeIsCenterAligned)
                PWM_Trigger_WritePeriod(PWM_Trigger_backup.PWMPeriod);
            #endif /* (!PWM_Trigger_PWMModeIsCenterAligned) */

            PWM_Trigger_WriteCounter(PWM_Trigger_backup.PWMUdb);

            #if (PWM_Trigger_UseStatus)
                PWM_Trigger_STATUS_MASK = PWM_Trigger_backup.InterruptMaskValue;
            #endif /* (PWM_Trigger_UseStatus) */

            #if(PWM_Trigger_DeadBandMode == PWM_Trigger__B_PWM__DBM_256_CLOCKS || \
                PWM_Trigger_DeadBandMode == PWM_Trigger__B_PWM__DBM_2_4_CLOCKS)
                PWM_Trigger_WriteDeadTime(PWM_Trigger_backup.PWMdeadBandValue);
            #endif /* deadband count is either 2-4 clocks or 256 clocks */

            #if(PWM_Trigger_KillModeMinTime)
                PWM_Trigger_WriteKillTime(PWM_Trigger_backup.PWMKillCounterPeriod);
            #endif /* (PWM_Trigger_KillModeMinTime) */

            #if(PWM_Trigger_UseControl)
                PWM_Trigger_WriteControlRegister(PWM_Trigger_backup.PWMControlRegister);
            #endif /* (PWM_Trigger_UseControl) */
        #endif  /* (!PWM_Trigger_UsingFixedFunction) */
    }


/*******************************************************************************
* Function Name: PWM_Trigger_Sleep
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
*  PWM_Trigger_backup.PWMEnableState:  Is modified depending on the enable
*  state of the block before entering sleep mode.
*
*******************************************************************************/
void PWM_Trigger_Sleep(void) 
{
    #if(PWM_Trigger_UseControl)
        if(PWM_Trigger_CTRL_ENABLE == (PWM_Trigger_CONTROL & PWM_Trigger_CTRL_ENABLE))
        {
            /*Component is enabled */
            PWM_Trigger_backup.PWMEnableState = 1u;
        }
        else
        {
            /* Component is disabled */
            PWM_Trigger_backup.PWMEnableState = 0u;
        }
    #endif /* (PWM_Trigger_UseControl) */

    /* Stop component */
    PWM_Trigger_Stop();

    /* Save registers configuration */
    PWM_Trigger_SaveConfig();
}


/*******************************************************************************
* Function Name: PWM_Trigger_Wakeup
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
*  PWM_Trigger_backup.pwmEnable:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void PWM_Trigger_Wakeup(void) 
{
     /* Restore registers values */
    PWM_Trigger_RestoreConfig();

    if(PWM_Trigger_backup.PWMEnableState != 0u)
    {
        /* Enable component's operation */
        PWM_Trigger_Enable();
    } /* Do nothing if component's block was disabled before */

}


/* [] END OF FILE */
