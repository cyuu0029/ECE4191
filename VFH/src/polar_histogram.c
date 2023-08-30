#include <stdlib.h>
#include <math.h>

#include "..\VFH\include\vfh.h"

// Converting Histogram Grid into Polar Histogram for VFH implementation.

int modulo(int x, int m) {
  /* Source: http://crsouza.blogspot.com/2009/09/modulo-and-modular-distance-in-c.html */
  int r;

  if (m < 0) m = -m;

  r = x % m;
  return r < 0 ? r + m : r;
}

histogram * polar_histogram_create(int alpha, double threshold, double density_a, double density_b) {
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
  hist->density_a = density_a;
  hist->density_b = density_b;

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

void polar_histogram_update(histogram *hist, grid *map)
{
  int width = map->width;
  int height = map->height;
  double dens_a = hist->density_a;
  double dens_b = hist->density_b;

  /* Calculate densities based on grid. */
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {

      /* Calculate the angular position (beta) of this cell. */
      double beta = atan2((double)(j - height/2), (double)(i - width/2));
      if (beta < 0) {
        beta += M_PI;
      }
      /* Calculate the obstacle density of this cell. */
      double density = pow(map->cells[i * width + j], 2);
      density *= dens_a - dens_b * sqrt(pow(i - width/2, 2) + pow(j - height/2, 2));

      /* Add density to respective point in the histogram. */
      hist->densities[(int) floor(beta / hist->alpha)] += density;
    }
  }

  /* Polar Histogram Smoothing Function*/
  // The research paper uses smoothing coefficient of 5 which gives decent results
  int smoothing_factor = 5;

  if (hist->sectors > smoothing_factor) {
    double smoothed_POD = 0;

    for (int i = 0; i < hist->sectors; i++) {
      for (int j = 0; j < smoothing_factor; j++) {
        smoothed_POD += (smoothing_factor - j) * (hist->densities[modulo(i-j, hist->sectors)] + hist->densities[modulo(i+j, hist->sectors)]);
      }
      smoothed_POD += smoothing_factor * hist->densities[i];
      smoothed_POD *= 1 / (2 * smoothing_factor + 1);
      hist->densities[i] = smoothed_POD;
    }
  }
}

int * candidate_valley(histogram * smoothed_histogram) {
  /* Selects the candidate valley based on the produced Polar Histogram */
  // NOTE: THIS IMPLEMENTATION IS NOT VER
  int * candidate_idx = (int *)malloc(smoothed_histogram->sectors * sizeof(int));
  int lst_length = sizeof(candidate_idx) / sizeof(int);
  int idx_counter = 0;

  // Define the threshold value for a candidate valley
  int VALLEY_THRESHOLD = 100;

  // Loop through densities and select candidate positions
  for (int i = 0; i < smoothed_histogram->sectors; i++) {

    if (smoothed_histogram->densities[i] < VALLEY_THRESHOLD) {
      candidate_idx[i] = i;
      idx_counter++;
    } else {
      candidate_idx[i] = NULL;
    }
  }

  // Clean list and return indexes of valleys in histogram
  int * candidate_lst = (int *)malloc(idx_counter * sizeof(int));
  int temp = 0;
  for (int i = 0; i < lst_length; i++) {
    if (candidate_idx[i] != NULL) {
      candidate_lst[temp] = candidate_idx[i];
      temp++;
    }
  }
  
  return candidate_lst;
}

