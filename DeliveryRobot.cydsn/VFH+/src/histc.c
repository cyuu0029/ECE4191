/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: histc.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "histc.h"
#include "rt_nonfinite.h"
#include "vfhTest_emxutil.h"
#include "vfhTest_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *X
 *                const double edges[180]
 *                double N[180]
 *                emxArray_real_T *BIN
 * Return Type  : void
 */
void histc(const emxArray_real_T *X, const double edges[180], double N[180],
           emxArray_real_T *BIN)
{
  const double *X_data;
  double *BIN_data;
  int high_i;
  int i;
  int k;
  int low_i;
  int low_ip1;
  int mid_i;
  X_data = X->data;
  memset(&N[0], 0, 180U * sizeof(double));
  i = BIN->size[0];
  BIN->size[0] = (unsigned short)X->size[0];
  emxEnsureCapacity_real_T(BIN, i);
  BIN_data = BIN->data;
  low_i = (unsigned short)X->size[0];
  for (i = 0; i < low_i; i++) {
    BIN_data[i] = 0.0;
  }
  i = X->size[0];
  for (k = 0; k < i; k++) {
    low_i = 0;
    if (!rtIsNaN(X_data[k])) {
      if ((X_data[k] >= edges[0]) && (X_data[k] < edges[179])) {
        low_i = 1;
        low_ip1 = 2;
        high_i = 180;
        while (high_i > low_ip1) {
          mid_i = (low_i + high_i) >> 1;
          if (X_data[k] >= edges[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }
      }
      if (X_data[k] == edges[179]) {
        low_i = 180;
      }
    }
    if (low_i > 0) {
      N[low_i - 1]++;
    }
    BIN_data[k] = low_i;
  }
}

/*
 * File trailer for histc.c
 *
 * [EOF]
 */
