#include <stdlib.h>
#include <math.h>

#include "histogram_grid.h"
#include "polar_histogram.h"

// Converting Histogram Grid into Polar Histogram for VFH implementation.

histogram * initial_histogram(int alpha, double threshold, double density_a, double density_b) {
  /* Create a histogram pointer and allocate memory to it. */
  histogram * hist = malloc(sizeof(histogram));

  /* Is there enough memory for the histogram? */
  if (NULL == hist) {
    free(hist);
    return NULL;
  }

  /* Initialize the histogram parameters. */
  hist->alpha = alpha;
  hist->sectors = 360 / alpha;
  hist->threshold = threshold;
  hist->densities = (int *)malloc(hist->sectors * sizeof(int));

  if (hist->densities == NULL) {
    free(hist);
    return NULL;
  }

  /* Initialize all densities to 0. */
  for (int i = 0; i < hist->sectors; ++i) {
    hist->densities[i] = 0;
  }

  return hist;
}

void hist_update(histogram * hist, grid * map) {
  int width = map->width;
  int height = map->height;
  double dens_a = hist->density_a;
  double dens_b = hist->density_b;

  /* Calculate densities based on grid. */
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {

      /* Calculate the angular position (beta) of this cell. */
      double beta = atan2((double)(j - height/2), (double)(i - width/2));

      /* Calculate the obstacle density of this cell. */
      double density = pow(map->cells[i * width + j], 2);
      density *= dens_a - dens_b * sqrt(pow(i - width/2, 2) + pow(j - height/2, 2));

      /* Add density to respective point in the histogram. */
      hist->densities[(int) floor(beta / hist->alpha)] += density;
    }
  }
}
