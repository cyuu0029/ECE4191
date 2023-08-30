/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: controllerVFH.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "controllerVFH.h"
#include "VectorFieldHistogramBase.h"
#include "lidarScan.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "vfhTest_emxutil.h"
#include "vfhTest_internal_types.h"
#include "vfhTest_types.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : controllerVFH *obj
 *                const emxArray_real_T *varargin_1
 *                const emxArray_real_T *varargin_2
 *                double varargin_3
 * Return Type  : double
 */
double controllerVFH_stepImpl(controllerVFH *obj,
                              const emxArray_real_T *varargin_1,
                              const emxArray_real_T *varargin_2,
                              double varargin_3)
{
  emxArray_real_T c_varargin_1;
  emxArray_real_T c_varargin_2;
  emxArray_real_T *scan_InternalAngles;
  emxArray_real_T *scan_InternalRanges;
  double b_obj;
  double steeringDir;
  double target;
  int b_varargin_1;
  int b_varargin_2;
  int d_varargin_1;
  int d_varargin_2;
  int i;
  unsigned char b_tmp_data[180];
  unsigned char tmp_data[180];
  bool expl_temp;
  emxInit_real_T(&scan_InternalRanges, 1);
  emxInit_real_T(&scan_InternalAngles, 1);
  b_varargin_1 = varargin_1->size[1];
  b_varargin_2 = varargin_2->size[1];
  c_varargin_1 = *varargin_1;
  d_varargin_1 = b_varargin_1;
  c_varargin_1.size = &d_varargin_1;
  c_varargin_1.numDimensions = 1;
  c_varargin_2 = *varargin_2;
  d_varargin_2 = b_varargin_2;
  c_varargin_2.size = &d_varargin_2;
  c_varargin_2.numDimensions = 1;
  lidarScan_lidarScan(&c_varargin_1, &c_varargin_2, scan_InternalRanges,
                      scan_InternalAngles, &expl_temp);
  target = varargin_3;
  if (fabs(varargin_3) > 3.1415926535897931) {
    target = b_mod(varargin_3 + 3.1415926535897931);
    if ((target == 0.0) && (varargin_3 + 3.1415926535897931 > 0.0)) {
      target = 6.2831853071795862;
    }
    target -= 3.1415926535897931;
  }
  c_VectorFieldHistogramBase_buil(obj, scan_InternalRanges,
                                  scan_InternalAngles);
  b_obj = obj->HistogramThresholds[1];
  b_varargin_2 = 0;
  b_varargin_1 = 0;
  for (i = 0; i < 180; i++) {
    expl_temp = (obj->PolarObstacleDensity[i] > b_obj);
    if (expl_temp) {
      b_varargin_2++;
      tmp_data[b_varargin_1] = (unsigned char)(i + 1);
      b_varargin_1++;
    }
  }
  for (b_varargin_1 = 0; b_varargin_1 < b_varargin_2; b_varargin_1++) {
    obj->BinaryHistogram[tmp_data[b_varargin_1] - 1] = true;
  }
  b_obj = obj->HistogramThresholds[0];
  b_varargin_2 = 0;
  b_varargin_1 = 0;
  for (i = 0; i < 180; i++) {
    expl_temp = (obj->PolarObstacleDensity[i] < b_obj);
    if (expl_temp) {
      b_varargin_2++;
      b_tmp_data[b_varargin_1] = (unsigned char)(i + 1);
      b_varargin_1++;
    }
  }
  for (b_varargin_1 = 0; b_varargin_1 < b_varargin_2; b_varargin_1++) {
    obj->BinaryHistogram[b_tmp_data[b_varargin_1] - 1] = false;
  }
  d_VectorFieldHistogramBase_buil(obj, scan_InternalRanges,
                                  scan_InternalAngles);
  steeringDir = c_VectorFieldHistogramBase_sele(obj, target);
  emxFree_real_T(&scan_InternalAngles);
  emxFree_real_T(&scan_InternalRanges);
  return steeringDir;
}

/*
 * File trailer for controllerVFH.c
 *
 * [EOF]
 */
