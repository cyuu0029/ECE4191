/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: VectorFieldHistogramBase.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

#ifndef VECTORFIELDHISTOGRAMBASE_H
#define VECTORFIELDHISTOGRAMBASE_H

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
void c_VectorFieldHistogramBase_buil(
    controllerVFH *obj, const emxArray_real_T *scan_InternalRanges,
    const emxArray_real_T *scan_InternalAngles);

double c_VectorFieldHistogramBase_sele(controllerVFH *obj, double targetDir);

void c_VectorFieldHistogramBase_setu(controllerVFH *obj);

void d_VectorFieldHistogramBase_buil(
    controllerVFH *obj, const emxArray_real_T *scan_InternalRanges,
    const emxArray_real_T *scan_InternalAngles);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for VectorFieldHistogramBase.h
 *
 * [EOF]
 */
