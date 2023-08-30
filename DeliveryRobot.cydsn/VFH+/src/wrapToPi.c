/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: wrapToPi.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "wrapToPi.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double theta_data[]
 *                int theta_size[2]
 * Return Type  : void
 */
void wrapToPi(double theta_data[], int theta_size[2])
{
  static double y_data[276];
  int i;
  int k;
  int nx;
  int y_size_idx_1;
  bool pos_data[276];
  bool tmp_data[276];
  bool exitg1;
  bool varargout_1;
  nx = theta_size[1];
  y_size_idx_1 = (short)theta_size[1];
  for (k = 0; k < nx; k++) {
    y_data[k] = fabs(theta_data[k]);
  }
  for (i = 0; i < y_size_idx_1; i++) {
    pos_data[i] = (y_data[i] > 3.1415926535897931);
  }
  varargout_1 = false;
  nx = 1;
  exitg1 = false;
  while ((!exitg1) && (nx <= y_size_idx_1)) {
    if (pos_data[nx - 1]) {
      varargout_1 = true;
      exitg1 = true;
    } else {
      nx++;
    }
  }
  if (varargout_1) {
    y_size_idx_1 = theta_size[1];
    nx = theta_size[1];
    for (i = 0; i < nx; i++) {
      y_data[i] = theta_data[i] + 3.1415926535897931;
    }
    for (i = 0; i < y_size_idx_1; i++) {
      pos_data[i] = (y_data[i] > 0.0);
    }
    theta_size[0] = 1;
    theta_size[1] = (short)y_size_idx_1;
    nx = theta_size[1];
    for (k = 0; k < nx; k++) {
      theta_data[k] = b_mod(y_data[k]);
    }
    k = theta_size[1];
    nx = theta_size[1];
    for (i = 0; i < nx; i++) {
      tmp_data[i] = (theta_data[i] == 0.0);
    }
    for (nx = 0; nx < k; nx++) {
      if (tmp_data[nx] && pos_data[nx]) {
        theta_data[nx] = 6.2831853071795862;
      }
    }
    theta_size[0] = 1;
    nx = theta_size[1];
    for (i = 0; i < nx; i++) {
      theta_data[i] -= 3.1415926535897931;
    }
  }
}

/*
 * File trailer for wrapToPi.c
 *
 * [EOF]
 */
