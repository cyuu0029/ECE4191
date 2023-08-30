/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: nullAssignment.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "nullAssignment.h"
#include "rt_nonfinite.h"
#include "vfhTest_emxutil.h"
#include "vfhTest_types.h"

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *x
 *                const bool idx_data[]
 *                int idx_size
 * Return Type  : void
 */
void nullAssignment(emxArray_real_T *x, const bool idx_data[], int idx_size)
{
  double *x_data;
  int k;
  int k0;
  int nxin;
  int nxout;
  x_data = x->data;
  nxin = x->size[0];
  k0 = 0;
  for (k = 0; k < idx_size; k++) {
    k0 += idx_data[k];
  }
  nxout = x->size[0] - k0;
  k0 = -1;
  for (k = 0; k < nxin; k++) {
    if ((k + 1 > idx_size) || (!idx_data[k])) {
      k0++;
      x_data[k0] = x_data[k];
    }
  }
  k0 = x->size[0];
  if (1 > nxout) {
    x->size[0] = 0;
  } else {
    x->size[0] = nxout;
  }
  emxEnsureCapacity_real_T(x, k0);
}

/*
 * File trailer for nullAssignment.c
 *
 * [EOF]
 */
