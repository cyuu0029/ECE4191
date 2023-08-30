/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: controllerVFH.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

#ifndef CONTROLLERVFH_H
#define CONTROLLERVFH_H

/* Include Files */
#include "rtwtypes.h"
#include "vfhTest_internal_types.h"
#include "vfhTest_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double controllerVFH_stepImpl(controllerVFH *obj,
                              const emxArray_real_T *varargin_1,
                              const emxArray_real_T *varargin_2,
                              double varargin_3);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for controllerVFH.h
 *
 * [EOF]
 */
