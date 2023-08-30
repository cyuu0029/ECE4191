/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: histc.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

#ifndef HISTC_H
#define HISTC_H

/* Include Files */
#include "rtwtypes.h"
#include "vfhTest_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void histc(const emxArray_real_T *X, const double edges[180], double N[180],
           emxArray_real_T *BIN);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for histc.h
 *
 * [EOF]
 */
