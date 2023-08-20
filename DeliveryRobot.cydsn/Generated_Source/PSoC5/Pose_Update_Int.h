/*******************************************************************************
* File Name: Pose_Update_Int.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_Pose_Update_Int_H)
#define CY_ISR_Pose_Update_Int_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void Pose_Update_Int_Start(void);
void Pose_Update_Int_StartEx(cyisraddress address);
void Pose_Update_Int_Stop(void);

CY_ISR_PROTO(Pose_Update_Int_Interrupt);

void Pose_Update_Int_SetVector(cyisraddress address);
cyisraddress Pose_Update_Int_GetVector(void);

void Pose_Update_Int_SetPriority(uint8 priority);
uint8 Pose_Update_Int_GetPriority(void);

void Pose_Update_Int_Enable(void);
uint8 Pose_Update_Int_GetState(void);
void Pose_Update_Int_Disable(void);

void Pose_Update_Int_SetPending(void);
void Pose_Update_Int_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the Pose_Update_Int ISR. */
#define Pose_Update_Int_INTC_VECTOR            ((reg32 *) Pose_Update_Int__INTC_VECT)

/* Address of the Pose_Update_Int ISR priority. */
#define Pose_Update_Int_INTC_PRIOR             ((reg8 *) Pose_Update_Int__INTC_PRIOR_REG)

/* Priority of the Pose_Update_Int interrupt. */
#define Pose_Update_Int_INTC_PRIOR_NUMBER      Pose_Update_Int__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable Pose_Update_Int interrupt. */
#define Pose_Update_Int_INTC_SET_EN            ((reg32 *) Pose_Update_Int__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the Pose_Update_Int interrupt. */
#define Pose_Update_Int_INTC_CLR_EN            ((reg32 *) Pose_Update_Int__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the Pose_Update_Int interrupt state to pending. */
#define Pose_Update_Int_INTC_SET_PD            ((reg32 *) Pose_Update_Int__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the Pose_Update_Int interrupt. */
#define Pose_Update_Int_INTC_CLR_PD            ((reg32 *) Pose_Update_Int__INTC_CLR_PD_REG)


#endif /* CY_ISR_Pose_Update_Int_H */


/* [] END OF FILE */
