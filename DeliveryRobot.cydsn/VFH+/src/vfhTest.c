/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: vfhTest.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "vfhTest.h"
#include "SystemCore.h"
#include "controllerVFH.h"
#include "rt_nonfinite.h"
#include "vfhTest_internal_types.h"
#include "vfhTest_types.h"

/* Function Definitions */
/*
 * function [steeringDir] = vfhTest(ranges,angles,targetDir)
 *
 * UNTITLED Summary of this function goes here
 *    Detailed explanation goes here
 *
 * Arguments    : const emxArray_real_T *ranges
 *                const emxArray_real_T *angles
 *                double targetDir
 * Return Type  : double
 */
double vfhTest(const emxArray_real_T *ranges, const emxArray_real_T *angles,
               double targetDir)
{
  static controllerVFH vfh;
  unsigned int inSize[8];
  int k;
  bool exitg1;
  /* 'vfhTest:4' vfh = controllerVFH; */
  vfh.NarrowOpeningThreshold = 0.8;
  vfh.RobotRadius = 0.1;
  vfh.SafetyDistance = 0.1;
  vfh.MinTurningRadius = 0.1;
  vfh.TargetDirectionWeight = 5.0;
  vfh.CurrentDirectionWeight = 2.0;
  vfh.PreviousDirectionWeight = 2.0;
  vfh.DistanceLimits[0] = 0.05;
  vfh.HistogramThresholds[0] = 3.0;
  vfh.AngularLimits[0] = -3.1415926535897931;
  vfh.DistanceLimits[1] = 2.0;
  vfh.HistogramThresholds[1] = 10.0;
  vfh.AngularLimits[1] = 3.1415926535897931;
  vfh.isInitialized = 0;
  /* 'vfhTest:5' steeringDir = vfh(ranges,angles,targetDir); */
  SystemCore_setupAndReset(&vfh, ranges, angles);
  inSize[0] = 1U;
  inSize[1] = (unsigned int)ranges->size[1];
  for (k = 0; k < 6; k++) {
    inSize[k + 2] = 1U;
  }
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 8)) {
    if (vfh.inputVarSize[0].f1[k] != inSize[k]) {
      for (k = 0; k < 8; k++) {
        vfh.inputVarSize[0].f1[k] = inSize[k];
      }
      exitg1 = true;
    } else {
      k++;
    }
  }
  inSize[0] = 1U;
  inSize[1] = (unsigned int)angles->size[1];
  for (k = 0; k < 6; k++) {
    inSize[k + 2] = 1U;
  }
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 8)) {
    if (vfh.inputVarSize[1].f1[k] != inSize[k]) {
      for (k = 0; k < 8; k++) {
        vfh.inputVarSize[1].f1[k] = inSize[k];
      }
      exitg1 = true;
    } else {
      k++;
    }
  }
  return controllerVFH_stepImpl(&vfh, ranges, angles, targetDir);
}

/*
 * File trailer for vfhTest.c
 *
 * [EOF]
 */
