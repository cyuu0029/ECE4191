/*******************************************************************************
* File Name: Timer_Echo_PM.c
* Version 2.80
*
*  Description:
*     This file provides the power management source code to API for the
*     Timer.
*
*   Note:
*     None
*
*******************************************************************************
* Copyright 2008-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "Timer_Echo.h"

static Timer_Echo_backupStruct Timer_Echo_backup;


/*******************************************************************************
* Function Name: Timer_Echo_SaveConfig
********************************************************************************
*
* Summary:
*     Save the current user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Timer_Echo_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void Timer_Echo_SaveConfig(void) 
{
    #if (!Timer_Echo_UsingFixedFunction)
        Timer_Echo_backup.TimerUdb = Timer_Echo_ReadCounter();
        Timer_Echo_backup.InterruptMaskValue = Timer_Echo_STATUS_MASK;
        #if (Timer_Echo_UsingHWCaptureCounter)
            Timer_Echo_backup.TimerCaptureCounter = Timer_Echo_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!Timer_Echo_UDB_CONTROL_REG_REMOVED)
            Timer_Echo_backup.TimerControlRegister = Timer_Echo_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: Timer_Echo_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Timer_Echo_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Timer_Echo_RestoreConfig(void) 
{   
    #if (!Timer_Echo_UsingFixedFunction)

        Timer_Echo_WriteCounter(Timer_Echo_backup.TimerUdb);
        Timer_Echo_STATUS_MASK =Timer_Echo_backup.InterruptMaskValue;
        #if (Timer_Echo_UsingHWCaptureCounter)
            Timer_Echo_SetCaptureCount(Timer_Echo_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!Timer_Echo_UDB_CONTROL_REG_REMOVED)
            Timer_Echo_WriteControlRegister(Timer_Echo_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: Timer_Echo_Sleep
********************************************************************************
*
* Summary:
*     Stop and Save the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Timer_Echo_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void Timer_Echo_Sleep(void) 
{
    #if(!Timer_Echo_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(Timer_Echo_CTRL_ENABLE == (Timer_Echo_CONTROL & Timer_Echo_CTRL_ENABLE))
        {
            /* Timer is enabled */
            Timer_Echo_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            Timer_Echo_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    Timer_Echo_Stop();
    Timer_Echo_SaveConfig();
}


/*******************************************************************************
* Function Name: Timer_Echo_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Timer_Echo_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Timer_Echo_Wakeup(void) 
{
    Timer_Echo_RestoreConfig();
    #if(!Timer_Echo_UDB_CONTROL_REG_REMOVED)
        if(Timer_Echo_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                Timer_Echo_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
