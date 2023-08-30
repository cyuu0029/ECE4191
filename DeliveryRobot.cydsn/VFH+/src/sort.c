/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sort.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "sort.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double x[2]
 * Return Type  : void
 */
void sort(double x[2])
{
  double tmp;
  if ((!(x[0] <= x[1])) && (!rtIsNaN(x[1]))) {
    tmp = x[0];
    x[0] = x[1];
    x[1] = tmp;
  }
}

/*
 * File trailer for sort.c
 *
 * [EOF]
 */
