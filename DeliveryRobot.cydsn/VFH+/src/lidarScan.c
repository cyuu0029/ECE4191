/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: lidarScan.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "lidarScan.h"
#include "mod.h"
#include "nullAssignment.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "vfhTest_emxutil.h"
#include "vfhTest_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Type Definitions */
#ifndef typedef_cell_wrap_17
#define typedef_cell_wrap_17
typedef struct {
  double f1[2];
} cell_wrap_17;
#endif /* typedef_cell_wrap_17 */

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *varargin_1
 *                const emxArray_real_T *varargin_2
 *                emxArray_real_T *obj_InternalRanges
 *                emxArray_real_T *obj_InternalAngles
 *                bool *obj_ContainsOnlyFiniteData
 * Return Type  : void
 */
void lidarScan_lidarScan(const emxArray_real_T *varargin_1,
                         const emxArray_real_T *varargin_2,
                         emxArray_real_T *obj_InternalRanges,
                         emxArray_real_T *obj_InternalAngles,
                         bool *obj_ContainsOnlyFiniteData)
{
  static bool x_data[40000];
  emxArray_real_T *thetaWrap;
  emxArray_real_T *y;
  const double *varargin_1_data;
  const double *varargin_2_data;
  double *obj_InternalRanges_data;
  double *thetaWrap_data;
  int i;
  int k;
  int nx;
  bool exitg1;
  bool varargout_1;
  varargin_2_data = varargin_2->data;
  varargin_1_data = varargin_1->data;
  i = obj_InternalRanges->size[0];
  obj_InternalRanges->size[0] = varargin_1->size[0];
  emxEnsureCapacity_real_T(obj_InternalRanges, i);
  obj_InternalRanges_data = obj_InternalRanges->data;
  nx = varargin_1->size[0];
  for (i = 0; i < nx; i++) {
    obj_InternalRanges_data[i] = varargin_1_data[i];
  }
  i = obj_InternalAngles->size[0];
  obj_InternalAngles->size[0] = varargin_2->size[0];
  emxEnsureCapacity_real_T(obj_InternalAngles, i);
  obj_InternalRanges_data = obj_InternalAngles->data;
  nx = varargin_2->size[0];
  for (i = 0; i < nx; i++) {
    obj_InternalRanges_data[i] = varargin_2_data[i];
  }
  emxInit_real_T(&y, 1);
  nx = varargin_2->size[0];
  i = y->size[0];
  y->size[0] = varargin_2->size[0];
  emxEnsureCapacity_real_T(y, i);
  obj_InternalRanges_data = y->data;
  for (k = 0; k < nx; k++) {
    obj_InternalRanges_data[k] = fabs(varargin_2_data[k]);
  }
  k = y->size[0];
  nx = y->size[0];
  for (i = 0; i < nx; i++) {
    x_data[i] = (obj_InternalRanges_data[i] > 3.1415926535897931);
  }
  varargout_1 = false;
  nx = 1;
  exitg1 = false;
  while ((!exitg1) && (nx <= k)) {
    if (x_data[nx - 1]) {
      varargout_1 = true;
      exitg1 = true;
    } else {
      nx++;
    }
  }
  if (varargout_1) {
    i = y->size[0];
    y->size[0] = varargin_2->size[0];
    emxEnsureCapacity_real_T(y, i);
    obj_InternalRanges_data = y->data;
    nx = varargin_2->size[0];
    for (i = 0; i < nx; i++) {
      obj_InternalRanges_data[i] = varargin_2_data[i] + 3.1415926535897931;
    }
    emxInit_real_T(&thetaWrap, 1);
    i = thetaWrap->size[0];
    thetaWrap->size[0] = y->size[0];
    emxEnsureCapacity_real_T(thetaWrap, i);
    thetaWrap_data = thetaWrap->data;
    nx = y->size[0];
    for (k = 0; k < nx; k++) {
      thetaWrap_data[k] = b_mod(obj_InternalRanges_data[k]);
    }
    k = thetaWrap->size[0];
    nx = thetaWrap->size[0];
    for (i = 0; i < nx; i++) {
      x_data[i] = (thetaWrap_data[i] == 0.0);
    }
    for (nx = 0; nx < k; nx++) {
      if (x_data[nx] && (obj_InternalRanges_data[nx] > 0.0)) {
        thetaWrap_data[nx] = 6.2831853071795862;
      }
    }
    i = obj_InternalAngles->size[0];
    obj_InternalAngles->size[0] = thetaWrap->size[0];
    emxEnsureCapacity_real_T(obj_InternalAngles, i);
    obj_InternalRanges_data = obj_InternalAngles->data;
    nx = thetaWrap->size[0];
    for (i = 0; i < nx; i++) {
      obj_InternalRanges_data[i] = thetaWrap_data[i] - 3.1415926535897931;
    }
    emxFree_real_T(&thetaWrap);
  }
  emxFree_real_T(&y);
  *obj_ContainsOnlyFiniteData = false;
}

/*
 * Arguments    : const emxArray_real_T *obj_InternalRanges
 *                const emxArray_real_T *obj_InternalAngles
 *                const double varargin_2[2]
 *                emxArray_real_T *objOut_InternalRanges
 *                emxArray_real_T *objOut_InternalAngles
 *                bool *objOut_ContainsOnlyFiniteData
 * Return Type  : void
 */
void lidarScan_removeInvalidData(const emxArray_real_T *obj_InternalRanges,
                                 const emxArray_real_T *obj_InternalAngles,
                                 const double varargin_2[2],
                                 emxArray_real_T *objOut_InternalRanges,
                                 emxArray_real_T *objOut_InternalAngles,
                                 bool *objOut_ContainsOnlyFiniteData)
{
  static double dv[2] = {0.0, 0.0};
  static bool b_tmp_data[40000];
  static bool c_tmp_data[40000];
  static bool tmp_data[40000];
  static bool validAngleLimitIndices_data[40000];
  static bool validIdx_data[40000];
  static bool validRangeLimitIndices_data[40000];
  cell_wrap_17 parsedResults[2];
  emxArray_real_T *b_objOut_InternalAngles;
  emxArray_real_T *b_objOut_InternalRanges;
  double *b_objOut_InternalRanges_data;
  double *objOut_InternalAngles_data;
  double *objOut_InternalRanges_data;
  int i;
  int loop_ub;
  int validIdx_size;
  bool exitg1;
  bool p;
  dv[1U] = rtGetInf();
  lidarScan_lidarScan(obj_InternalRanges, obj_InternalAngles,
                      objOut_InternalRanges, objOut_InternalAngles,
                      objOut_ContainsOnlyFiniteData);
  objOut_InternalAngles_data = objOut_InternalAngles->data;
  objOut_InternalRanges_data = objOut_InternalRanges->data;
  *objOut_ContainsOnlyFiniteData = true;
  loop_ub = objOut_InternalRanges->size[0];
  for (i = 0; i < loop_ub; i++) {
    validRangeLimitIndices_data[i] = true;
  }
  loop_ub = objOut_InternalRanges->size[0];
  for (i = 0; i < loop_ub; i++) {
    validAngleLimitIndices_data[i] = true;
  }
  parsedResults[1].f1[0] = -3.1415926535897931;
  parsedResults[1].f1[1] = 3.1415926535897931;
  p = true;
  loop_ub = 0;
  exitg1 = false;
  while ((!exitg1) && (loop_ub < 2)) {
    if (!(varargin_2[loop_ub] == dv[loop_ub])) {
      p = false;
      exitg1 = true;
    } else {
      loop_ub++;
    }
  }
  if (!p) {
    loop_ub = objOut_InternalRanges->size[0];
    for (i = 0; i < loop_ub; i++) {
      validRangeLimitIndices_data[i] =
          ((objOut_InternalRanges_data[i] >= varargin_2[0]) &&
           (objOut_InternalRanges_data[i] <= varargin_2[1]));
    }
  }
  p = true;
  loop_ub = 0;
  exitg1 = false;
  while ((!exitg1) && (loop_ub < 2)) {
    if (!(parsedResults[1].f1[loop_ub] ==
          6.2831853071795862 * (double)loop_ub + -3.1415926535897931)) {
      p = false;
      exitg1 = true;
    } else {
      loop_ub++;
    }
  }
  if (!p) {
    sort(parsedResults[1].f1);
    loop_ub = objOut_InternalAngles->size[0];
    for (i = 0; i < loop_ub; i++) {
      validAngleLimitIndices_data[i] =
          ((objOut_InternalAngles_data[i] >= parsedResults[1].f1[0]) &&
           (objOut_InternalAngles_data[i] <= parsedResults[1].f1[1]));
    }
  }
  validIdx_size = objOut_InternalRanges->size[0];
  loop_ub = objOut_InternalRanges->size[0];
  for (i = 0; i < loop_ub; i++) {
    validIdx_data[i] = rtIsInf(objOut_InternalRanges_data[i]);
  }
  loop_ub = objOut_InternalRanges->size[0];
  for (i = 0; i < loop_ub; i++) {
    tmp_data[i] = rtIsNaN(objOut_InternalRanges_data[i]);
  }
  loop_ub = objOut_InternalAngles->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_tmp_data[i] = rtIsInf(objOut_InternalAngles_data[i]);
  }
  loop_ub = objOut_InternalAngles->size[0];
  for (i = 0; i < loop_ub; i++) {
    c_tmp_data[i] = rtIsNaN(objOut_InternalAngles_data[i]);
  }
  for (i = 0; i < validIdx_size; i++) {
    validIdx_data[i] =
        ((!validIdx_data[i]) && (!tmp_data[i]) &&
         ((!b_tmp_data[i]) && (!c_tmp_data[i])) &&
         validRangeLimitIndices_data[i] && validAngleLimitIndices_data[i]);
  }
  p = true;
  loop_ub = 1;
  exitg1 = false;
  while ((!exitg1) && (loop_ub <= validIdx_size)) {
    if (!validIdx_data[loop_ub - 1]) {
      p = false;
      exitg1 = true;
    } else {
      loop_ub++;
    }
  }
  if (!p) {
    for (i = 0; i < validIdx_size; i++) {
      validIdx_data[i] = !validIdx_data[i];
    }
    emxInit_real_T(&b_objOut_InternalRanges, 1);
    nullAssignment(objOut_InternalRanges, validIdx_data, validIdx_size);
    objOut_InternalRanges_data = objOut_InternalRanges->data;
    nullAssignment(objOut_InternalAngles, validIdx_data, validIdx_size);
    objOut_InternalAngles_data = objOut_InternalAngles->data;
    i = b_objOut_InternalRanges->size[0];
    b_objOut_InternalRanges->size[0] = objOut_InternalRanges->size[0];
    emxEnsureCapacity_real_T(b_objOut_InternalRanges, i);
    b_objOut_InternalRanges_data = b_objOut_InternalRanges->data;
    loop_ub = objOut_InternalRanges->size[0] - 1;
    for (i = 0; i <= loop_ub; i++) {
      b_objOut_InternalRanges_data[i] = objOut_InternalRanges_data[i];
    }
    emxInit_real_T(&b_objOut_InternalAngles, 1);
    i = b_objOut_InternalAngles->size[0];
    b_objOut_InternalAngles->size[0] = objOut_InternalAngles->size[0];
    emxEnsureCapacity_real_T(b_objOut_InternalAngles, i);
    objOut_InternalRanges_data = b_objOut_InternalAngles->data;
    loop_ub = objOut_InternalAngles->size[0] - 1;
    for (i = 0; i <= loop_ub; i++) {
      objOut_InternalRanges_data[i] = objOut_InternalAngles_data[i];
    }
    lidarScan_lidarScan(b_objOut_InternalRanges, b_objOut_InternalAngles,
                        objOut_InternalRanges, objOut_InternalAngles,
                        objOut_ContainsOnlyFiniteData);
    emxFree_real_T(&b_objOut_InternalAngles);
    emxFree_real_T(&b_objOut_InternalRanges);
  }
  *objOut_ContainsOnlyFiniteData = true;
}

/*
 * File trailer for lidarScan.c
 *
 * [EOF]
 */
