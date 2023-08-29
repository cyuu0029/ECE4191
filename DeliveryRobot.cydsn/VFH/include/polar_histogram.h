/**
 * @file polar_histogram.h
 * @brief
 */

#ifndef POLAR_HISTOGRAM_H
#define POLAR_HISTOGRAM_H
    
#include "histogram_grid.h"

/* Polar histogram. */
typedef struct {
  int alpha;
  int sectors;
  double threshold;
  double damping_constant;
  double density_a;
  double density_b;
  double * densities;
} histogram;

/* hist_init: Return a pointer to a new hist. NULL otherwise. */
histogram * initial_histogram(int alpha, double threshold, double density_a, double density_b);

/* hist_update: Update hist with grid's information. */
void hist_update(histogram * hist, grid * map);

/* ind_mod: wraps indices around the polar histogram */
int ind_mod(int a, int b);

#endif
