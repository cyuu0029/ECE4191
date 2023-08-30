/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: VectorFieldHistogramBase.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "VectorFieldHistogramBase.h"
#include "cross.h"
#include "find.h"
#include "histc.h"
#include "lidarScan.h"
#include "minOrMax.h"
#include "mod.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "vfhTest_emxutil.h"
#include "vfhTest_internal_types.h"
#include "vfhTest_types.h"
#include "wrapToPi.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void c_VectorFieldHistogramBase_comp(
    const controllerVFH *obj, const double c_data[], const int c_size[2],
    double targetDir, double prevDir, double cost_data[], int cost_size[2]);

/* Function Definitions */
/*
 * Arguments    : const controllerVFH *obj
 *                const double c_data[]
 *                const int c_size[2]
 *                double targetDir
 *                double prevDir
 *                double cost_data[]
 *                int cost_size[2]
 * Return Type  : void
 */
static void c_VectorFieldHistogramBase_comp(
    const controllerVFH *obj, const double c_data[], const int c_size[2],
    double targetDir, double prevDir, double cost_data[], int cost_size[2])
{
  static double b_b_data[276];
  static double b_data[276];
  static double delta_data[276];
  double b;
  double cdWeight;
  double pdWeight;
  double tdWeight;
  int delta_size[2];
  int k;
  int nx;
  tdWeight = obj->TargetDirectionWeight;
  cdWeight = obj->CurrentDirectionWeight;
  pdWeight = obj->PreviousDirectionWeight;
  b = (tdWeight + cdWeight) + pdWeight;
  delta_size[0] = 1;
  delta_size[1] = c_size[1];
  nx = c_size[1];
  for (k = 0; k < nx; k++) {
    delta_data[k] = targetDir - c_data[k];
  }
  wrapToPi(delta_data, delta_size);
  nx = delta_size[1];
  cost_size[1] = (short)delta_size[1];
  for (k = 0; k < nx; k++) {
    cost_data[k] = fabs(delta_data[k]);
  }
  delta_size[0] = 1;
  delta_size[1] = c_size[1];
  nx = c_size[1];
  for (k = 0; k < nx; k++) {
    delta_data[k] = 0.0 - c_data[k];
  }
  wrapToPi(delta_data, delta_size);
  nx = delta_size[1];
  for (k = 0; k < nx; k++) {
    b_data[k] = fabs(delta_data[k]);
  }
  delta_size[0] = 1;
  delta_size[1] = c_size[1];
  nx = c_size[1];
  for (k = 0; k < nx; k++) {
    delta_data[k] = prevDir - c_data[k];
  }
  wrapToPi(delta_data, delta_size);
  nx = delta_size[1];
  for (k = 0; k < nx; k++) {
    b_b_data[k] = fabs(delta_data[k]);
  }
  cost_size[0] = 1;
  nx = cost_size[1] - 1;
  for (k = 0; k <= nx; k++) {
    cost_data[k] = ((tdWeight * cost_data[k] + cdWeight * b_data[k]) +
                    pdWeight * b_b_data[k]) /
                   3.0 * b;
  }
}

/*
 * Arguments    : controllerVFH *obj
 *                const emxArray_real_T *scan_InternalRanges
 *                const emxArray_real_T *scan_InternalAngles
 * Return Type  : void
 */
void c_VectorFieldHistogramBase_buil(controllerVFH *obj,
                                     const emxArray_real_T *scan_InternalRanges,
                                     const emxArray_real_T *scan_InternalAngles)
{
  emxArray_boolean_T *validWeights;
  emxArray_real_T *higherAng;
  emxArray_real_T *higherVec;
  emxArray_real_T *kalphaVec;
  emxArray_real_T *kh;
  emxArray_real_T *lh;
  emxArray_real_T *lk;
  emxArray_real_T *lowerVec;
  emxArray_real_T *sinOfEnlargement;
  emxArray_real_T *validScan_InternalAngles;
  emxArray_real_T *varargin_2;
  emxArray_real_T *weightedRanges;
  double b_validWeights[180];
  double kalpha[3];
  double b_obj[2];
  double constA;
  double *higherAng_data;
  double *lh_data;
  double *sinOfEnlargement_data;
  double *validScan_InternalAngles_data;
  double *weightedRanges_data;
  int b_k;
  int i;
  int k;
  int loop_ub;
  int nx;
  bool expl_temp;
  bool *validWeights_data;
  emxInit_real_T(&validScan_InternalAngles, 1);
  emxInit_real_T(&weightedRanges, 1);
  emxInit_real_T(&sinOfEnlargement, 1);
  b_obj[0] = obj->DistanceLimits[0];
  b_obj[1] = obj->DistanceLimits[1];
  lidarScan_removeInvalidData(scan_InternalRanges, scan_InternalAngles, b_obj,
                              sinOfEnlargement, validScan_InternalAngles,
                              &expl_temp);
  validScan_InternalAngles_data = validScan_InternalAngles->data;
  sinOfEnlargement_data = sinOfEnlargement->data;
  constA = obj->DistanceLimits[1];
  k = weightedRanges->size[0];
  weightedRanges->size[0] = sinOfEnlargement->size[0];
  emxEnsureCapacity_real_T(weightedRanges, k);
  weightedRanges_data = weightedRanges->data;
  loop_ub = sinOfEnlargement->size[0];
  for (k = 0; k < loop_ub; k++) {
    weightedRanges_data[k] = constA - sinOfEnlargement_data[k];
  }
  constA = obj->RobotRadius + obj->SafetyDistance;
  if (constA == 0.0) {
    histc(validScan_InternalAngles, obj->AngularSectorMidPoints,
          obj->PolarObstacleDensity, sinOfEnlargement);
    sinOfEnlargement_data = sinOfEnlargement->data;
    memset(&obj->PolarObstacleDensity[0], 0, 180U * sizeof(double));
    k = sinOfEnlargement->size[0];
    for (i = 0; i < k; i++) {
      nx = (int)sinOfEnlargement_data[i] - 1;
      obj->PolarObstacleDensity[nx] += weightedRanges_data[i];
    }
  } else {
    loop_ub = sinOfEnlargement->size[0];
    for (k = 0; k < loop_ub; k++) {
      sinOfEnlargement_data[k] = constA / sinOfEnlargement_data[k];
    }
    nx = sinOfEnlargement->size[0];
    for (i = 0; i < nx; i++) {
      if (sinOfEnlargement_data[i] >= 1.0) {
        sinOfEnlargement_data[i] = 0.99999999999999978;
      }
    }
    nx = sinOfEnlargement->size[0];
    for (b_k = 0; b_k < nx; b_k++) {
      sinOfEnlargement_data[b_k] = asin(sinOfEnlargement_data[b_k]);
    }
    emxInit_real_T(&higherAng, 1);
    k = higherAng->size[0];
    higherAng->size[0] = validScan_InternalAngles->size[0];
    emxEnsureCapacity_real_T(higherAng, k);
    higherAng_data = higherAng->data;
    loop_ub = validScan_InternalAngles->size[0];
    for (k = 0; k < loop_ub; k++) {
      higherAng_data[k] =
          validScan_InternalAngles_data[k] + sinOfEnlargement_data[k];
    }
    k = sinOfEnlargement->size[0];
    sinOfEnlargement->size[0] = validScan_InternalAngles->size[0];
    emxEnsureCapacity_real_T(sinOfEnlargement, k);
    sinOfEnlargement_data = sinOfEnlargement->data;
    loop_ub = validScan_InternalAngles->size[0];
    for (k = 0; k < loop_ub; k++) {
      sinOfEnlargement_data[k] =
          validScan_InternalAngles_data[k] - sinOfEnlargement_data[k];
    }
    k = validScan_InternalAngles->size[0];
    validScan_InternalAngles->size[0] = sinOfEnlargement->size[0];
    emxEnsureCapacity_real_T(validScan_InternalAngles, k);
    validScan_InternalAngles_data = validScan_InternalAngles->data;
    loop_ub = sinOfEnlargement->size[0];
    for (k = 0; k < loop_ub; k++) {
      validScan_InternalAngles_data[k] = sinOfEnlargement_data[k];
    }
    nx = sinOfEnlargement->size[0];
    for (b_k = 0; b_k < nx; b_k++) {
      validScan_InternalAngles_data[b_k] =
          cos(validScan_InternalAngles_data[b_k]);
    }
    emxInit_real_T(&varargin_2, 1);
    k = varargin_2->size[0];
    varargin_2->size[0] = sinOfEnlargement->size[0];
    emxEnsureCapacity_real_T(varargin_2, k);
    lh_data = varargin_2->data;
    loop_ub = sinOfEnlargement->size[0];
    for (k = 0; k < loop_ub; k++) {
      lh_data[k] = sinOfEnlargement_data[k];
    }
    nx = sinOfEnlargement->size[0];
    for (b_k = 0; b_k < nx; b_k++) {
      lh_data[b_k] = sin(lh_data[b_k]);
    }
    emxInit_real_T(&lowerVec, 2);
    k = lowerVec->size[0] * lowerVec->size[1];
    lowerVec->size[0] = validScan_InternalAngles->size[0];
    lowerVec->size[1] = 3;
    emxEnsureCapacity_real_T(lowerVec, k);
    sinOfEnlargement_data = lowerVec->data;
    loop_ub = validScan_InternalAngles->size[0];
    for (k = 0; k < loop_ub; k++) {
      sinOfEnlargement_data[k] = validScan_InternalAngles_data[k];
    }
    loop_ub = varargin_2->size[0];
    for (k = 0; k < loop_ub; k++) {
      sinOfEnlargement_data[k + lowerVec->size[0]] = lh_data[k];
    }
    loop_ub = sinOfEnlargement->size[0];
    for (k = 0; k < loop_ub; k++) {
      sinOfEnlargement_data[k + lowerVec->size[0] * 2] = 0.0;
    }
    k = validScan_InternalAngles->size[0];
    validScan_InternalAngles->size[0] = higherAng->size[0];
    emxEnsureCapacity_real_T(validScan_InternalAngles, k);
    validScan_InternalAngles_data = validScan_InternalAngles->data;
    loop_ub = higherAng->size[0];
    for (k = 0; k < loop_ub; k++) {
      validScan_InternalAngles_data[k] = higherAng_data[k];
    }
    nx = higherAng->size[0];
    for (b_k = 0; b_k < nx; b_k++) {
      validScan_InternalAngles_data[b_k] =
          cos(validScan_InternalAngles_data[b_k]);
    }
    k = varargin_2->size[0];
    varargin_2->size[0] = higherAng->size[0];
    emxEnsureCapacity_real_T(varargin_2, k);
    lh_data = varargin_2->data;
    loop_ub = higherAng->size[0];
    for (k = 0; k < loop_ub; k++) {
      lh_data[k] = higherAng_data[k];
    }
    nx = higherAng->size[0];
    for (b_k = 0; b_k < nx; b_k++) {
      lh_data[b_k] = sin(lh_data[b_k]);
    }
    emxInit_real_T(&higherVec, 2);
    k = higherVec->size[0] * higherVec->size[1];
    higherVec->size[0] = validScan_InternalAngles->size[0];
    higherVec->size[1] = 3;
    emxEnsureCapacity_real_T(higherVec, k);
    sinOfEnlargement_data = higherVec->data;
    loop_ub = validScan_InternalAngles->size[0];
    for (k = 0; k < loop_ub; k++) {
      sinOfEnlargement_data[k] = validScan_InternalAngles_data[k];
    }
    loop_ub = varargin_2->size[0];
    for (k = 0; k < loop_ub; k++) {
      sinOfEnlargement_data[k + higherVec->size[0]] = lh_data[k];
    }
    emxFree_real_T(&varargin_2);
    loop_ub = higherAng->size[0];
    for (k = 0; k < loop_ub; k++) {
      sinOfEnlargement_data[k + higherVec->size[0] * 2] = 0.0;
    }
    emxInit_boolean_T(&validWeights);
    k = validWeights->size[0] * validWeights->size[1];
    validWeights->size[0] = 180;
    validWeights->size[1] = lowerVec->size[0];
    emxEnsureCapacity_boolean_T(validWeights, k);
    validWeights_data = validWeights->data;
    loop_ub = 180 * lowerVec->size[0];
    for (k = 0; k < loop_ub; k++) {
      validWeights_data[k] = true;
    }
    emxInit_real_T(&lh, 2);
    cross(lowerVec, higherVec, lh);
    lh_data = lh->data;
    loop_ub = lh->size[0];
    emxInit_real_T(&kalphaVec, 2);
    emxInit_real_T(&lk, 2);
    emxInit_real_T(&kh, 2);
    kalpha[2] = 0.0;
    i = validWeights->size[1];
    for (b_k = 0; b_k < 180; b_k++) {
      obj->PolarObstacleDensity[b_k] = obj->AngularSectorMidPoints[b_k];
      constA = cos(obj->PolarObstacleDensity[b_k]);
      obj->PolarObstacleDensity[b_k] = sin(obj->PolarObstacleDensity[b_k]);
      kalpha[0] = constA;
      kalpha[1] = obj->PolarObstacleDensity[b_k];
      repmat(kalpha, lowerVec->size[0], kalphaVec);
      cross(lowerVec, kalphaVec, lk);
      sinOfEnlargement_data = lk->data;
      cross(kalphaVec, higherVec, kh);
      higherAng_data = kh->data;
      nx = lk->size[0];
      k = validScan_InternalAngles->size[0];
      validScan_InternalAngles->size[0] = lk->size[0];
      emxEnsureCapacity_real_T(validScan_InternalAngles, k);
      validScan_InternalAngles_data = validScan_InternalAngles->data;
      for (k = 0; k < nx; k++) {
        constA = sinOfEnlargement_data[k + lk->size[0] * 2];
        if (constA < 0.0) {
          constA = -1.0;
        } else if (constA > 0.0) {
          constA = 1.0;
        } else if (constA == 0.0) {
          constA = 0.0;
        }
        validScan_InternalAngles_data[k] = constA;
      }
      nx = kh->size[0];
      k = sinOfEnlargement->size[0];
      sinOfEnlargement->size[0] = kh->size[0];
      emxEnsureCapacity_real_T(sinOfEnlargement, k);
      sinOfEnlargement_data = sinOfEnlargement->data;
      for (k = 0; k < nx; k++) {
        constA = higherAng_data[k + kh->size[0] * 2];
        if (constA < 0.0) {
          constA = -1.0;
        } else if (constA > 0.0) {
          constA = 1.0;
        } else if (constA == 0.0) {
          constA = 0.0;
        }
        sinOfEnlargement_data[k] = constA;
      }
      k = higherAng->size[0];
      higherAng->size[0] = loop_ub;
      emxEnsureCapacity_real_T(higherAng, k);
      higherAng_data = higherAng->data;
      for (k = 0; k < loop_ub; k++) {
        constA = lh_data[k + lh->size[0] * 2];
        if (constA < 0.0) {
          constA = -1.0;
        } else if (constA > 0.0) {
          constA = 1.0;
        } else if (constA == 0.0) {
          constA = 0.0;
        }
        higherAng_data[k] = constA;
      }
      nx = validScan_InternalAngles->size[0];
      for (k = 0; k < nx; k++) {
        validScan_InternalAngles_data[k] =
            (validScan_InternalAngles_data[k] + sinOfEnlargement_data[k]) +
            higherAng_data[k];
      }
      nx = validScan_InternalAngles->size[0];
      k = sinOfEnlargement->size[0];
      sinOfEnlargement->size[0] = validScan_InternalAngles->size[0];
      emxEnsureCapacity_real_T(sinOfEnlargement, k);
      sinOfEnlargement_data = sinOfEnlargement->data;
      for (k = 0; k < nx; k++) {
        sinOfEnlargement_data[k] = fabs(validScan_InternalAngles_data[k]);
      }
      for (k = 0; k < i; k++) {
        validWeights_data[b_k + 180 * k] = (sinOfEnlargement_data[k] > 1.0);
      }
    }
    emxFree_real_T(&kh);
    emxFree_real_T(&lk);
    emxFree_real_T(&kalphaVec);
    emxFree_real_T(&lh);
    emxFree_real_T(&higherVec);
    emxFree_real_T(&lowerVec);
    emxFree_real_T(&higherAng);
    loop_ub = validWeights->size[1];
    for (k = 0; k < 180; k++) {
      b_validWeights[k] = 0.0;
      for (nx = 0; nx < loop_ub; nx++) {
        b_validWeights[k] +=
            (double)validWeights_data[k + 180 * nx] * weightedRanges_data[nx];
      }
      obj->PolarObstacleDensity[k] = b_validWeights[k];
    }
    emxFree_boolean_T(&validWeights);
  }
  emxFree_real_T(&sinOfEnlargement);
  emxFree_real_T(&weightedRanges);
  emxFree_real_T(&validScan_InternalAngles);
}

/*
 * Arguments    : controllerVFH *obj
 *                double targetDir
 * Return Type  : double
 */
double c_VectorFieldHistogramBase_sele(controllerVFH *obj, double targetDir)
{
  static double b_tmp_data[276];
  static double candidateDirections_data[276];
  static double candidateDirs_data[276];
  static double angles_data[182];
  static double sectors_data[182];
  static double changes[181];
  static double narrowDirs_data[91];
  static double sectorSizes_data[91];
  static double theta2_data[91];
  static bool nearIdx_data[49680];
  emxArray_real_T *c;
  emxArray_real_T *candToSectDiff;
  double cVal;
  double thetaSteer;
  double thetaWrap;
  double *c_data;
  double *candToSectDiff_data;
  int foundSectors_data[181];
  int candidateDirections_size[2];
  int foundSectors_size[2];
  int tmp_size[2];
  int i;
  int loop_ub;
  int m;
  int narrowDirs_size_idx_1;
  int tmp2;
  int trueCount;
  int work;
  short e_tmp_data[276];
  signed char x[182];
  unsigned char d_tmp_data[180];
  signed char c_tmp_data[91];
  signed char tmp_data[91];
  signed char sz_idx_1;
  bool freeDirs_data[276];
  bool x_data[180];
  bool exitg1;
  bool y;
  x[0] = 0;
  for (i = 0; i < 180; i++) {
    x[i + 1] = (signed char)!obj->MaskedHistogram[i];
  }
  x[181] = 0;
  work = x[0];
  for (m = 0; m < 181; m++) {
    tmp2 = work;
    work = x[m + 1];
    changes[m] = work - tmp2;
  }
  y = false;
  tmp2 = 0;
  exitg1 = false;
  while ((!exitg1) && (tmp2 < 181)) {
    if (changes[tmp2] != 0.0) {
      y = true;
      exitg1 = true;
    } else {
      tmp2++;
    }
  }
  if (!y) {
    thetaSteer = rtNaN;
    obj->PreviousDirection = rtNaN;
  } else {
    eml_find(changes, foundSectors_data, foundSectors_size);
    sz_idx_1 = (signed char)(foundSectors_size[1] / 2);
    tmp2 = sz_idx_1;
    m = sz_idx_1 << 1;
    for (i = 0; i < m; i++) {
      sectors_data[i] = foundSectors_data[i];
    }
    if (1 > sz_idx_1) {
      loop_ub = 0;
    } else {
      loop_ub = sz_idx_1;
    }
    for (i = 0; i < loop_ub; i++) {
      work = 2 * i + 1;
      sectors_data[work] = (double)foundSectors_data[work] - 1.0;
    }
    if (0 <= m - 1) {
      memset(&angles_data[0], 0, m * sizeof(double));
    }
    if (1 > sz_idx_1) {
      loop_ub = 0;
    } else {
      loop_ub = sz_idx_1;
    }
    for (i = 0; i < loop_ub; i++) {
      angles_data[2 * i] =
          obj->AngularSectorMidPoints[(int)sectors_data[2 * i] - 1];
    }
    if (1 > sz_idx_1) {
      loop_ub = 0;
    } else {
      loop_ub = sz_idx_1;
    }
    for (i = 0; i < loop_ub; i++) {
      work = 2 * i + 1;
      angles_data[work] =
          obj->AngularSectorMidPoints[(int)sectors_data[work] - 1];
    }
    if (sz_idx_1 != 0) {
      for (m = 0; m < tmp2; m++) {
        work = m << 1;
        sectorSizes_data[m] = sectors_data[work + 1] - sectors_data[work];
      }
    }
    m = sz_idx_1 - 1;
    for (i = 0; i <= m; i++) {
      sectorSizes_data[i] *= obj->AngularDifference;
    }
    cVal = obj->NarrowOpeningThreshold;
    for (i = 0; i < tmp2; i++) {
      candidateDirs_data[i] = cVal;
    }
    trueCount = 0;
    tmp2 = 0;
    for (loop_ub = 0; loop_ub <= m; loop_ub++) {
      if (sectorSizes_data[loop_ub] < candidateDirs_data[loop_ub]) {
        trueCount++;
        tmp_data[tmp2] = (signed char)(loop_ub + 1);
        tmp2++;
      }
    }
    for (i = 0; i < trueCount; i++) {
      narrowDirs_data[i] = angles_data[2 * (tmp_data[i] - 1)];
    }
    for (i = 0; i < trueCount; i++) {
      theta2_data[i] = angles_data[2 * (tmp_data[i] - 1) + 1];
    }
    tmp_size[0] = 1;
    tmp_size[1] = trueCount;
    if (0 <= trueCount - 1) {
      memcpy(&b_tmp_data[0], &narrowDirs_data[0], trueCount * sizeof(double));
    }
    wrapToPi(b_tmp_data, tmp_size);
    narrowDirs_size_idx_1 = tmp_size[1];
    loop_ub = tmp_size[1];
    if (0 <= loop_ub - 1) {
      memcpy(&narrowDirs_data[0], &b_tmp_data[0], loop_ub * sizeof(double));
    }
    tmp_size[0] = 1;
    tmp_size[1] = trueCount;
    if (0 <= trueCount - 1) {
      memcpy(&b_tmp_data[0], &theta2_data[0], trueCount * sizeof(double));
    }
    wrapToPi(b_tmp_data, tmp_size);
    loop_ub = tmp_size[1];
    if (0 <= loop_ub - 1) {
      memcpy(&theta2_data[0], &b_tmp_data[0], loop_ub * sizeof(double));
    }
    tmp_size[0] = 1;
    tmp_size[1] = narrowDirs_size_idx_1;
    for (i = 0; i < narrowDirs_size_idx_1; i++) {
      thetaWrap = narrowDirs_data[i];
      b_tmp_data[i] = thetaWrap - (thetaWrap - theta2_data[i]) / 2.0;
    }
    wrapToPi(b_tmp_data, tmp_size);
    narrowDirs_size_idx_1 = tmp_size[1];
    loop_ub = tmp_size[1];
    if (0 <= loop_ub - 1) {
      memcpy(&narrowDirs_data[0], &b_tmp_data[0], loop_ub * sizeof(double));
    }
    trueCount = 0;
    tmp2 = 0;
    for (loop_ub = 0; loop_ub <= m; loop_ub++) {
      if (!(sectorSizes_data[loop_ub] < candidateDirs_data[loop_ub])) {
        trueCount++;
        c_tmp_data[tmp2] = (signed char)(loop_ub + 1);
        tmp2++;
      }
    }
    work = trueCount + trueCount;
    for (i = 0; i < trueCount; i++) {
      sectors_data[i] = angles_data[2 * (c_tmp_data[i] - 1)] + cVal / 2.0;
    }
    for (i = 0; i < trueCount; i++) {
      sectors_data[i + trueCount] =
          angles_data[2 * (c_tmp_data[i] - 1) + 1] - cVal / 2.0;
    }
    if (rtIsNaN(obj->PreviousDirection)) {
      obj->PreviousDirection = 0.0;
    }
    tmp2 = work + tmp_size[1];
    if (0 <= work - 1) {
      memcpy(&candidateDirs_data[0], &sectors_data[0], work * sizeof(double));
    }
    for (i = 0; i < narrowDirs_size_idx_1; i++) {
      candidateDirs_data[i + work] = narrowDirs_data[i];
    }
    candidateDirs_data[tmp2] = targetDir;
    candidateDirs_data[tmp2 + 1] = 0.0;
    candidateDirs_data[tmp2 + 2] = obj->PreviousDirection;
    loop_ub = tmp2 + 3;
    if (0 <= loop_ub - 1) {
      memcpy(&b_tmp_data[0], &candidateDirs_data[0], loop_ub * sizeof(double));
    }
    emxInit_real_T(&c, 2);
    i = c->size[0] * c->size[1];
    c->size[0] = (short)(tmp2 + 3);
    c->size[1] = 180;
    emxEnsureCapacity_real_T(c, i);
    c_data = c->data;
    i = c->size[0] - 1;
    for (tmp2 = 0; tmp2 < 180; tmp2++) {
      for (work = 0; work <= i; work++) {
        cVal = b_tmp_data[work] - obj->AngularSectorMidPoints[tmp2];
        c_data[work + c->size[0] * tmp2] = cVal;
        if (fabs(cVal) > 3.1415926535897931) {
          thetaWrap = b_mod(cVal + 3.1415926535897931);
          if ((thetaWrap == 0.0) && (cVal + 3.1415926535897931 > 0.0)) {
            thetaWrap = 6.2831853071795862;
          }
          c_data[work + c->size[0] * tmp2] = thetaWrap - 3.1415926535897931;
        }
      }
    }
    emxInit_real_T(&candToSectDiff, 2);
    work = c->size[0] * 180;
    i = candToSectDiff->size[0] * candToSectDiff->size[1];
    candToSectDiff->size[0] = (short)c->size[0];
    candToSectDiff->size[1] = 180;
    emxEnsureCapacity_real_T(candToSectDiff, i);
    candToSectDiff_data = candToSectDiff->data;
    for (tmp2 = 0; tmp2 < work; tmp2++) {
      candToSectDiff_data[tmp2] = fabs(c_data[tmp2]);
    }
    minimum(candToSectDiff, b_tmp_data, &work);
    i = c->size[0] * c->size[1];
    if (candToSectDiff->size[0] == work) {
      c->size[0] = (short)candToSectDiff->size[0];
    } else if (work < candToSectDiff->size[0]) {
      c->size[0] = (short)work;
    } else {
      c->size[0] = (short)candToSectDiff->size[0];
    }
    c->size[1] = 180;
    emxEnsureCapacity_real_T(c, i);
    c_data = c->data;
    i = c->size[0] - 1;
    for (tmp2 = 0; tmp2 < 180; tmp2++) {
      for (work = 0; work <= i; work++) {
        c_data[work + c->size[0] * tmp2] =
            candToSectDiff_data[work + candToSectDiff->size[0] * tmp2] -
            b_tmp_data[work];
      }
    }
    emxFree_real_T(&candToSectDiff);
    m = c->size[0];
    loop_ub = c->size[0] * 180;
    for (i = 0; i < loop_ub; i++) {
      nearIdx_data[i] = (c_data[i] < 1.4901161193847656E-8);
    }
    emxFree_real_T(&c);
    for (i = 0; i < m; i++) {
      freeDirs_data[i] = true;
    }
    for (loop_ub = 0; loop_ub < m; loop_ub++) {
      trueCount = 0;
      tmp2 = 0;
      for (work = 0; work < 180; work++) {
        if (nearIdx_data[loop_ub + m * work]) {
          trueCount++;
          d_tmp_data[tmp2] = (unsigned char)(work + 1);
          tmp2++;
        }
      }
      for (i = 0; i < trueCount; i++) {
        x_data[i] = obj->MaskedHistogram[d_tmp_data[i] - 1];
      }
      y = false;
      work = 1;
      exitg1 = false;
      while ((!exitg1) && (work <= trueCount)) {
        if (x_data[work - 1]) {
          y = true;
          exitg1 = true;
        } else {
          work++;
        }
      }
      freeDirs_data[loop_ub] = !y;
    }
    work = m - 1;
    trueCount = 0;
    candidateDirections_size[0] = 1;
    tmp2 = 0;
    for (loop_ub = 0; loop_ub <= work; loop_ub++) {
      if (freeDirs_data[loop_ub]) {
        trueCount++;
        candidateDirections_data[tmp2] = candidateDirs_data[loop_ub];
        tmp2++;
      }
    }
    candidateDirections_size[1] = trueCount;
    c_VectorFieldHistogramBase_comp(
        obj, candidateDirections_data, candidateDirections_size, targetDir,
        obj->PreviousDirection, candidateDirs_data, foundSectors_size);
    cVal = b_minimum(candidateDirs_data, foundSectors_size);
    m = foundSectors_size[1] - 1;
    for (i = 0; i <= m; i++) {
      candidateDirs_data[i] -= cVal;
    }
    trueCount = 0;
    tmp2 = 0;
    for (loop_ub = 0; loop_ub <= m; loop_ub++) {
      if (candidateDirs_data[loop_ub] < 1.4901161193847656E-8) {
        trueCount++;
        e_tmp_data[tmp2] = (short)(loop_ub + 1);
        tmp2++;
      }
    }
    foundSectors_size[0] = 1;
    foundSectors_size[1] = trueCount;
    for (i = 0; i < trueCount; i++) {
      candidateDirs_data[i] = candidateDirections_data[e_tmp_data[i] - 1];
    }
    thetaSteer = b_minimum(candidateDirs_data, foundSectors_size);
    obj->PreviousDirection = thetaSteer;
  }
  return thetaSteer;
}

/*
 * Arguments    : controllerVFH *obj
 * Return Type  : void
 */
void c_VectorFieldHistogramBase_setu(controllerVFH *obj)
{
  double d2scaled;
  double delta1;
  double delta2;
  int k;
  obj->PreviousDirection = 0.0;
  d2scaled = obj->AngularLimits[0] + 0.017453292519943295;
  obj->AngularSectorMidPoints[179] =
      obj->AngularLimits[1] - 0.017453292519943295;
  obj->AngularSectorMidPoints[0] = obj->AngularLimits[0] + 0.017453292519943295;
  if (obj->AngularLimits[0] + 0.017453292519943295 ==
      -(obj->AngularLimits[1] - 0.017453292519943295)) {
    d2scaled = (obj->AngularLimits[1] - 0.017453292519943295) / 179.0;
    for (k = 0; k < 178; k++) {
      obj->AngularSectorMidPoints[k + 1] =
          (2.0 * ((double)k + 2.0) - 181.0) * d2scaled;
    }
  } else if (((obj->AngularLimits[0] + 0.017453292519943295 < 0.0) !=
              (obj->AngularLimits[1] - 0.017453292519943295 < 0.0)) &&
             ((fabs(obj->AngularLimits[0] + 0.017453292519943295) >
               8.9884656743115785E+307) ||
              (fabs(obj->AngularLimits[1] - 0.017453292519943295) >
               8.9884656743115785E+307))) {
    delta1 = (obj->AngularLimits[0] + 0.017453292519943295) / 179.0;
    delta2 = (obj->AngularLimits[1] - 0.017453292519943295) / 179.0;
    for (k = 0; k < 178; k++) {
      obj->AngularSectorMidPoints[k + 1] =
          (d2scaled + delta2 * ((double)k + 1.0)) - delta1 * ((double)k + 1.0);
    }
  } else {
    delta1 = ((obj->AngularLimits[1] - 0.017453292519943295) -
              (obj->AngularLimits[0] + 0.017453292519943295)) /
             179.0;
    for (k = 0; k < 178; k++) {
      obj->AngularSectorMidPoints[k + 1] =
          d2scaled + ((double)k + 1.0) * delta1;
    }
  }
  d2scaled = obj->AngularSectorMidPoints[1] - obj->AngularSectorMidPoints[0];
  if (fabs(d2scaled) > 3.1415926535897931) {
    delta1 = b_mod(d2scaled + 3.1415926535897931);
    if ((delta1 == 0.0) && (d2scaled + 3.1415926535897931 > 0.0)) {
      delta1 = 6.2831853071795862;
    }
    d2scaled = delta1 - 3.1415926535897931;
  }
  obj->AngularDifference = fabs(d2scaled);
  d2scaled = obj->AngularDifference / 2.0;
  for (k = 0; k < 180; k++) {
    obj->AngularSectorStartPoints[k] =
        obj->AngularSectorMidPoints[k] - d2scaled;
    obj->BinaryHistogram[k] = false;
  }
}

/*
 * Arguments    : controllerVFH *obj
 *                const emxArray_real_T *scan_InternalRanges
 *                const emxArray_real_T *scan_InternalAngles
 * Return Type  : void
 */
void d_VectorFieldHistogramBase_buil(controllerVFH *obj,
                                     const emxArray_real_T *scan_InternalRanges,
                                     const emxArray_real_T *scan_InternalAngles)
{
  static bool blockedL_data[40000];
  static bool blockedR_data[40000];
  emxArray_real_T *DYj;
  emxArray_real_T *a;
  emxArray_real_T *validScan_InternalAngles;
  emxArray_real_T *validScan_InternalRanges;
  emxArray_real_T *x;
  double b_obj[2];
  double DYl;
  double DYr;
  double *DYj_data;
  double *a_data;
  double *validScan_InternalAngles_data;
  double *validScan_InternalRanges_data;
  double *x_data;
  int blockedL_size;
  int blockedR_size;
  int idx;
  int k;
  int nx;
  unsigned short ii_data;
  bool exitg1;
  bool expl_temp;
  emxInit_real_T(&validScan_InternalRanges, 1);
  emxInit_real_T(&validScan_InternalAngles, 1);
  emxInit_real_T(&a, 1);
  DYr = -obj->MinTurningRadius;
  DYl = obj->MinTurningRadius;
  b_obj[0] = obj->DistanceLimits[0];
  b_obj[1] = obj->DistanceLimits[1];
  lidarScan_removeInvalidData(scan_InternalRanges, scan_InternalAngles, b_obj,
                              validScan_InternalRanges,
                              validScan_InternalAngles, &expl_temp);
  validScan_InternalAngles_data = validScan_InternalAngles->data;
  validScan_InternalRanges_data = validScan_InternalRanges->data;
  idx = a->size[0];
  a->size[0] = validScan_InternalAngles->size[0];
  emxEnsureCapacity_real_T(a, idx);
  a_data = a->data;
  nx = validScan_InternalAngles->size[0];
  for (idx = 0; idx < nx; idx++) {
    a_data[idx] = validScan_InternalAngles_data[idx];
  }
  nx = validScan_InternalAngles->size[0];
  for (k = 0; k < nx; k++) {
    a_data[k] = cos(a_data[k]);
  }
  emxInit_real_T(&DYj, 1);
  idx = DYj->size[0];
  DYj->size[0] = validScan_InternalAngles->size[0];
  emxEnsureCapacity_real_T(DYj, idx);
  DYj_data = DYj->data;
  nx = validScan_InternalAngles->size[0];
  for (idx = 0; idx < nx; idx++) {
    DYj_data[idx] = validScan_InternalAngles_data[idx];
  }
  nx = validScan_InternalAngles->size[0];
  for (k = 0; k < nx; k++) {
    DYj_data[k] = sin(DYj_data[k]);
  }
  idx = DYj->size[0];
  DYj->size[0] = validScan_InternalRanges->size[0];
  emxEnsureCapacity_real_T(DYj, idx);
  DYj_data = DYj->data;
  nx = validScan_InternalRanges->size[0];
  for (idx = 0; idx < nx; idx++) {
    DYj_data[idx] *= validScan_InternalRanges_data[idx];
  }
  idx = a->size[0];
  a->size[0] = validScan_InternalRanges->size[0];
  emxEnsureCapacity_real_T(a, idx);
  a_data = a->data;
  nx = validScan_InternalRanges->size[0];
  for (idx = 0; idx < nx; idx++) {
    a_data[idx] = 0.0 - validScan_InternalRanges_data[idx] * a_data[idx];
  }
  emxInit_real_T(&x, 1);
  idx = x->size[0];
  x->size[0] = a->size[0];
  emxEnsureCapacity_real_T(x, idx);
  x_data = x->data;
  nx = a->size[0];
  for (k = 0; k < nx; k++) {
    x_data[k] = a_data[k] * a_data[k];
  }
  idx = a->size[0];
  a->size[0] = DYj->size[0];
  emxEnsureCapacity_real_T(a, idx);
  a_data = a->data;
  nx = DYj->size[0];
  for (idx = 0; idx < nx; idx++) {
    a_data[idx] = DYr - DYj_data[idx];
  }
  idx = validScan_InternalRanges->size[0];
  validScan_InternalRanges->size[0] = a->size[0];
  emxEnsureCapacity_real_T(validScan_InternalRanges, idx);
  validScan_InternalRanges_data = validScan_InternalRanges->data;
  nx = a->size[0];
  for (k = 0; k < nx; k++) {
    validScan_InternalRanges_data[k] = a_data[k] * a_data[k];
  }
  emxFree_real_T(&a);
  idx = validScan_InternalRanges->size[0];
  validScan_InternalRanges->size[0] = x->size[0];
  emxEnsureCapacity_real_T(validScan_InternalRanges, idx);
  validScan_InternalRanges_data = validScan_InternalRanges->data;
  nx = x->size[0];
  for (idx = 0; idx < nx; idx++) {
    validScan_InternalRanges_data[idx] += x_data[idx];
  }
  nx = validScan_InternalRanges->size[0];
  for (k = 0; k < nx; k++) {
    validScan_InternalRanges_data[k] = sqrt(validScan_InternalRanges_data[k]);
  }
  DYr = (obj->MinTurningRadius + obj->RobotRadius) + obj->SafetyDistance;
  blockedR_size = validScan_InternalRanges->size[0];
  nx = validScan_InternalRanges->size[0];
  for (idx = 0; idx < nx; idx++) {
    blockedR_data[idx] = ((validScan_InternalRanges_data[idx] < DYr) &&
                          (validScan_InternalAngles_data[idx] <= 0.0));
  }
  nx = DYj->size[0];
  for (idx = 0; idx < nx; idx++) {
    DYj_data[idx] = DYl - DYj_data[idx];
  }
  idx = validScan_InternalRanges->size[0];
  validScan_InternalRanges->size[0] = DYj->size[0];
  emxEnsureCapacity_real_T(validScan_InternalRanges, idx);
  validScan_InternalRanges_data = validScan_InternalRanges->data;
  nx = DYj->size[0];
  for (k = 0; k < nx; k++) {
    validScan_InternalRanges_data[k] = DYj_data[k] * DYj_data[k];
  }
  emxFree_real_T(&DYj);
  nx = x->size[0];
  for (idx = 0; idx < nx; idx++) {
    x_data[idx] += validScan_InternalRanges_data[idx];
  }
  emxFree_real_T(&validScan_InternalRanges);
  nx = x->size[0];
  for (k = 0; k < nx; k++) {
    x_data[k] = sqrt(x_data[k]);
  }
  blockedL_size = x->size[0];
  nx = x->size[0];
  for (idx = 0; idx < nx; idx++) {
    blockedL_data[idx] =
        ((x_data[idx] < DYr) && (validScan_InternalAngles_data[idx] >= 0.0));
  }
  emxFree_real_T(&x);
  k = (1 <= blockedR_size);
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (blockedR_size > 0)) {
    if (blockedR_data[blockedR_size - 1]) {
      idx = 1;
      ii_data = (unsigned short)blockedR_size;
      exitg1 = true;
    } else {
      blockedR_size--;
    }
  }
  if (k == 1) {
    if (idx == 0) {
      k = 0;
    }
  } else {
    k = (1 <= idx);
  }
  for (idx = 0; idx < k; idx++) {
    DYr = validScan_InternalAngles_data[ii_data - 1];
  }
  blockedR_size = (1 <= blockedL_size);
  idx = 0;
  nx = 0;
  exitg1 = false;
  while ((!exitg1) && (nx <= blockedL_size - 1)) {
    if (blockedL_data[nx]) {
      idx = 1;
      ii_data = (unsigned short)(nx + 1);
      exitg1 = true;
    } else {
      nx++;
    }
  }
  if (blockedR_size == 1) {
    if (idx == 0) {
      blockedR_size = 0;
    }
  } else {
    blockedR_size = (1 <= idx);
  }
  for (idx = 0; idx < blockedR_size; idx++) {
    DYl = validScan_InternalAngles_data[ii_data - 1];
  }
  emxFree_real_T(&validScan_InternalAngles);
  if (k == 0) {
    DYr = obj->AngularSectorMidPoints[0];
  } else if (DYr <= obj->AngularSectorMidPoints[0]) {
    DYr = obj->AngularSectorMidPoints[1];
  }
  if (blockedR_size == 0) {
    DYl = obj->AngularSectorMidPoints[179];
  } else if (DYl >= obj->AngularSectorMidPoints[179]) {
    DYl = obj->AngularSectorMidPoints[178];
  }
  for (idx = 0; idx < 180; idx++) {
    obj->MaskedHistogram[idx] = (obj->BinaryHistogram[idx] ||
                                 ((obj->AngularSectorMidPoints[idx] < DYr) ||
                                  (obj->AngularSectorMidPoints[idx] > DYl)));
  }
}

/*
 * File trailer for VectorFieldHistogramBase.c
 *
 * [EOF]
 */
