/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.c
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include files */
#include "main.h"
#include "rt_nonfinite.h"
#include "vfhTest.h"
#include "vfhTest_emxAPI.h"
#include "vfhTest_terminate.h"
#include "vfhTest_types.h"

/* Function Declarations */
static emxArray_real_T *argInit_1xUnbounded_real_T(void);

static double argInit_real_T(void);

static void main_vfhTest(void);

/* Function Definitions */
static emxArray_real_T *argInit_1xUnbounded_real_T(void)
{
  emxArray_real_T *result;
  double *result_data;
  int idx0;
  int idx1;
  /* Set the size of the array.
Change this size to the value that the application requires. */
  result = emxCreate_real_T(1, 2);
  result_data = result->data;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 1; idx0++) {
    for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
      /* Set the value of the array element.
Change this value to the value that the application requires. */
      result_data[idx1] = argInit_real_T();
    }
  }
  return result;
}

static double argInit_real_T(void)
{
  return 0.0;
}

static void main_vfhTest(void)
{
  emxArray_real_T *angles;
  emxArray_real_T *ranges;
  double steeringDir;
  /* Initialize function 'vfhTest' input arguments. */
  /* Initialize function input argument 'ranges'. */
  ranges = argInit_1xUnbounded_real_T();
  /* Initialize function input argument 'angles'. */
  angles = argInit_1xUnbounded_real_T();
  /* Call the entry-point 'vfhTest'. */
  steeringDir = vfhTest(ranges, angles, argInit_real_T());
  emxDestroyArray_real_T(angles);
  emxDestroyArray_real_T(ranges);
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_vfhTest();
  /* Terminate the application.
You do not need to do this more than one time. */
  vfhTest_terminate();
  return 0;
}

/* End of code generation (main.c) */
