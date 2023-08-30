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
  hist->densities = malloc(hist->sectors * sizeof(double));
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

void hist_update(histogram * hist, grid * map) {
  int width = map->width;
  int height = map->height;
  double dens_a = 1;
  double dens_b = 2 * dens_a / (M_SQRT2*(width-1));

  for (int i = 0; i < hist->sectors; i++) {
    hist->densities[i] = 0;
  }

  /* Calculate densities based on grid. */
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {

      /* Calculate the angular position (beta) of this cell. */
      double beta = 180*atan2((double)(j - height/2), (double)(i - width/2))/M_PI;
      if( beta < 0 ) {
        beta += 360;
    }

      /* Calculate the obstacle density of this cell. */
      double density = pow(map->cells[i * width + j], 2);
      density *= dens_a - dens_b * sqrt(pow(i - width/2, 2) + pow(j - height/2, 2));

      /* Add density to respective point in the histogram. */
      hist->densities[(int) floor(beta / hist->alpha)] += density;
    }
  }
   
    /* Do histogram smoothing */
    int smoothing_size = 5;
    double temp_density[hist->sectors];
    
    if( hist->sectors > smoothing_size ) {
        double smoothed_POD;
        
        for( int i = 0; i < hist->sectors; i++ ) {
            for( int j = 1; j < smoothing_size; j++ ) {
                smoothed_POD += (smoothing_size-j)*(hist->densities[ind_mod(i-j, hist->sectors)] + hist->densities[ind_mod(i+j, hist->sectors)]);
            }
            smoothed_POD += smoothing_size * hist->densities[i];
            smoothed_POD *= 1.0/(2*smoothing_size+1);
            temp_density[i] = smoothed_POD;
        }
    }
    
    for( int i=0; i < hist->sectors; i++ ) {
        hist->densities[i] = temp_density[i];
    }
}

int ind_mod(int a, int b)
{
    int r = a % b;
    return r < 0 ? r + b : r;
}