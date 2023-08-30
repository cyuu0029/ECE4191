/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: cross.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Aug-2023 22:51:35
 */

/* Include Files */
#include "cross.h"
#include "rt_nonfinite.h"
#include "vfhTest_emxutil.h"
#include "vfhTest_types.h"

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *a
 *                const emxArray_real_T *b
 *                emxArray_real_T *c
 * Return Type  : void
 */
void cross(const emxArray_real_T *a, const emxArray_real_T *b,
           emxArray_real_T *c)
{
  const double *a_data;
  const double *b_data;
  double *c_data;
  int dim;
  int i1;
  int i2;
  int i3;
  int iNext;
  int iStart;
  int nHigh;
  int stride;
  int stridem1;
  bool exitg1;
  b_data = b->data;
  a_data = a->data;
  nHigh = c->size[0] * c->size[1];
  c->size[0] = (unsigned short)a->size[0];
  c->size[1] = 3;
  emxEnsureCapacity_real_T(c, nHigh);
  c_data = c->data;
  if (a->size[0] != 0) {
    dim = 0;
    nHigh = 0;
    exitg1 = false;
    while ((!exitg1) && (nHigh < 2)) {
      if (a->size[nHigh] == 3) {
        dim = nHigh + 1;
        exitg1 = true;
      } else {
        nHigh++;
      }
    }
    if (dim >= 2) {
      stride = a->size[0];
      stridem1 = a->size[0] - 1;
    } else {
      stride = 1;
      stridem1 = 0;
    }
    iNext = stride * 3;
    if (dim >= 2) {
      nHigh = 1;
    } else {
      nHigh = iNext * 2 + 1;
    }
    for (iStart = 1; iNext < 0 ? iStart >= nHigh : iStart <= nHigh;
         iStart += iNext) {
      dim = iStart + stridem1;
      for (i1 = iStart; i1 <= dim; i1++) {
        i2 = (i1 + stride) - 1;
        i3 = i2 + stride;
        c_data[i1 - 1] = a_data[i2] * b_data[i3] - a_data[i3] * b_data[i2];
        c_data[i2] = a_data[i3] * b_data[i1 - 1] - a_data[i1 - 1] * b_data[i3];
        c_data[i3] = a_data[i1 - 1] * b_data[i2] - a_data[i2] * b_data[i1 - 1];
      }
    }
  }
}

/*
 * File trailer for cross.c
 *
 * [EOF]
 */
