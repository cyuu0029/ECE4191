/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: vfhTest_internal_types.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

#ifndef VFHTEST_INTERNAL_TYPES_H
#define VFHTEST_INTERNAL_TYPES_H

/* Include Files */
#include "rtwtypes.h"
#include "vfhTest_types.h"

/* Type Definitions */
#ifndef typedef_cell_wrap_3
#define typedef_cell_wrap_3
typedef struct {
  unsigned int f1[8];
} cell_wrap_3;
#endif /* typedef_cell_wrap_3 */

#ifndef typedef_controllerVFH
#define typedef_controllerVFH
typedef struct {
  int isInitialized;
  cell_wrap_3 inputVarSize[3];
  double NarrowOpeningThreshold;
  double DistanceLimits[2];
  double RobotRadius;
  double SafetyDistance;
  double MinTurningRadius;
  double TargetDirectionWeight;
  double CurrentDirectionWeight;
  double PreviousDirectionWeight;
  double HistogramThresholds[2];
  double PolarObstacleDensity[180];
  bool BinaryHistogram[180];
  bool MaskedHistogram[180];
  double PreviousDirection;
  double AngularSectorMidPoints[180];
  double AngularDifference;
  double AngularSectorStartPoints[180];
  double AngularLimits[2];
} controllerVFH;
#endif /* typedef_controllerVFH */

#endif
/*
 * File trailer for vfhTest_internal_types.h
 *
 * [EOF]
 */
