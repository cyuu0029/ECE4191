/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: lidarScan.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

#ifndef LIDARSCAN_H
#define LIDARSCAN_H

/* Include Files */
#include "rtwtypes.h"
#include "vfhTest_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void lidarScan_lidarScan(const emxArray_real_T *varargin_1,
                         const emxArray_real_T *varargin_2,
                         emxArray_real_T *obj_InternalRanges,
                         emxArray_real_T *obj_InternalAngles,
                         bool *obj_ContainsOnlyFiniteData);

void lidarScan_removeInvalidData(const emxArray_real_T *obj_InternalRanges,
                                 const emxArray_real_T *obj_InternalAngles,
                                 const double varargin_2[2],
                                 emxArray_real_T *objOut_InternalRanges,
                                 emxArray_real_T *objOut_InternalAngles,
                                 bool *objOut_ContainsOnlyFiniteData);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for lidarScan.h
 *
 * [EOF]
 */
