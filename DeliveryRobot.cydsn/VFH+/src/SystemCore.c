/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: SystemCore.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "SystemCore.h"
#include "VectorFieldHistogramBase.h"
#include "rt_nonfinite.h"
#include "vfhTest_internal_types.h"
#include "vfhTest_types.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : controllerVFH *obj
 *                const emxArray_real_T *varargin_1
 *                const emxArray_real_T *varargin_2
 * Return Type  : void
 */
void SystemCore_setupAndReset(controllerVFH *obj,
                              const emxArray_real_T *varargin_1,
                              const emxArray_real_T *varargin_2)
{
  int i;
  obj->isInitialized = 1;
  obj->inputVarSize[0].f1[0] = 1U;
  obj->inputVarSize[0].f1[1] = (unsigned int)varargin_1->size[1];
  for (i = 0; i < 6; i++) {
    obj->inputVarSize[0].f1[i + 2] = 1U;
  }
  obj->inputVarSize[1].f1[0] = 1U;
  obj->inputVarSize[1].f1[1] = (unsigned int)varargin_2->size[1];
  for (i = 0; i < 6; i++) {
    obj->inputVarSize[1].f1[i + 2] = 1U;
  }
  for (i = 0; i < 8; i++) {
    obj->inputVarSize[2].f1[i] = 1U;
  }
  c_VectorFieldHistogramBase_setu(obj);
  memset(&obj->BinaryHistogram[0], 0, 180U * sizeof(bool));
  obj->PreviousDirection *= 0.0;
}

/*
 * File trailer for SystemCore.c
 *
 * [EOF]
 */
