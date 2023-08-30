/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: minOrMax.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "vfhTest_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 * Return Type  : double
 */
double b_minimum(const double x_data[], const int x_size[2])
{
  double d;
  double ex;
  int idx;
  int k;
  int last;
  bool exitg1;
  last = x_size[1];
  if (x_size[1] <= 2) {
    if (x_size[1] == 1) {
      ex = x_data[0];
    } else {
      ex = x_data[x_size[1] - 1];
      if ((!(x_data[0] > ex)) && ((!rtIsNaN(x_data[0])) || rtIsNaN(ex))) {
        ex = x_data[0];
      }
    }
  } else {
    if (!rtIsNaN(x_data[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!rtIsNaN(x_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = x_data[0];
    } else {
      ex = x_data[idx - 1];
      idx++;
      for (k = idx; k <= last; k++) {
        d = x_data[k - 1];
        if (ex > d) {
          ex = d;
        }
      }
    }
  }
  return ex;
}

/*
 * Arguments    : const emxArray_real_T *x
 *                double ex_data[]
 *                int *ex_size
 * Return Type  : void
 */
void minimum(const emxArray_real_T *x, double ex_data[], int *ex_size)
{
  const double *x_data;
  double b;
  int i;
  int j;
  int m;
  bool p;
  x_data = x->data;
  m = x->size[0] - 1;
  *ex_size = x->size[0];
  for (i = 0; i <= m; i++) {
    ex_data[i] = x_data[i];
  }
  for (j = 0; j < 179; j++) {
    for (i = 0; i <= m; i++) {
      b = x_data[i + x->size[0] * (j + 1)];
      if (rtIsNaN(b)) {
        p = false;
      } else if (rtIsNaN(ex_data[i])) {
        p = true;
      } else {
        p = (ex_data[i] > b);
      }
      if (p) {
        ex_data[i] = b;
      }
    }
  }
}

/*
 * File trailer for minOrMax.c
 *
 * [EOF]
 */
