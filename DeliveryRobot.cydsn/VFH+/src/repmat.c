/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: repmat.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "repmat.h"
#include "rt_nonfinite.h"
#include "vfhTest_emxutil.h"
#include "vfhTest_types.h"

/* Function Definitions */
/*
 * Arguments    : const double a[3]
 *                double varargin_1
 *                emxArray_real_T *b
 * Return Type  : void
 */
void repmat(const double a[3], double varargin_1, emxArray_real_T *b)
{
  double *b_data;
  int ibmat;
  int itilerow;
  int jcol;
  int ntilerows;
  ntilerows = b->size[0] * b->size[1];
  b->size[0] = (unsigned short)(int)varargin_1;
  b->size[1] = 3;
  emxEnsureCapacity_real_T(b, ntilerows);
  b_data = b->data;
  ntilerows = (int)varargin_1;
  for (jcol = 0; jcol < 3; jcol++) {
    ibmat = jcol * (int)varargin_1;
    for (itilerow = 0; itilerow < ntilerows; itilerow++) {
      b_data[ibmat + itilerow] = a[jcol];
    }
  }
}

/*
 * File trailer for repmat.c
 *
 * [EOF]
 */
