/*
** Virtual Field Histogram
**
** vfh.c
**
** Author: Carlos Agarie Junior
**
** This is an implementation of the Virtual Field Histogram algorithm, developed
** by J. Borenstein and Y.Koren in 1990.
*/

#include <stdlib.h>
#include <math.h>

#include "histogram_grid.h"
#include "polar_histogram.h"

#include "vfh.h"

/* Helpers. */

int modulo(int x, int m) {
  /* Source: http://crsouza.blogspot.com/2009/09/modulo-and-modular-distance-in-c.html */
  int r;

  if (m < 0) m = -m;

  r = x % m;
  return r < 0 ? r + m : r;
}

int modular_dist(int a, int b, int m) {
  int dist_a = modulo(a - b, m);
  int dist_b = modulo(b - a, m);

  return dist_a < dist_b? dist_a : dist_b;
}


//
// Control signals.
//

/* TODO: Improve the direction calculation. Re-read the paper. */

int calculate_direction(histogram * hist, int objective_direction) {
  int sector, best_direction = -1;
  int dist_best_and_obj, dist_sector_and_obj; /* Just to improve readability. */

  // The objective_direction is given in DEGREES and mapped to a sector.
  objective_direction = (int) floor(objective_direction / hist->alpha);


  // Search the densities array and return the most similar to the objective
  // direction that is below the threshold.

  for (sector = 0; sector < hist->sectors; ++sector) {

    if (hist->densities[sector] < hist->threshold) {

      dist_best_and_obj = modular_dist(best_direction, objective_direction, hist->sectors);
      dist_sector_and_obj = modular_dist(sector, objective_direction, hist->sectors);

      /* If dist_a < dist_sector_and_obj, we maintain the current best_direction. */
      if (-1 == best_direction || dist_sector_and_obj < dist_best_and_obj) {
        /* This serves as initialization. */
        best_direction = sector;
      }
    }
  }

  /* Map the best_direction into degrees. */
  return (int) floor(best_direction * hist->alpha);
}

double calculate_direction2(histogram * hist, int objective_direction) {
    int threshold = 5;
    int smax = 18;
    int nsectors = hist->sectors;
    
    valley * head_valley = malloc(sizeof(valley));
    head_valley->start_sector = nsectors-1;
    head_valley->next_valley = NULL;
    
    // generate binary histogram
    int binary_hist[nsectors];
    int object_sum = 0;
    
    for( int i = 0; i < nsectors; i++ ) {
        binary_hist[i] = hist->densities[i] > threshold;
        object_sum += binary_hist[i];
    }
    
    if( object_sum == 0 ) { 
        free(head_valley);
        return objective_direction; 
    }
    
    // get valleys
    valley * curr_valley = head_valley;
    int edges[nsectors];
    int in_valley = 0;
    for( int i = 0; i < nsectors; i++ ) {
        if( i==0 ) {
            edges[i] = binary_hist[0] - binary_hist[nsectors-1];
        } else {
            edges[i] = binary_hist[i] - binary_hist[i-1];
        }
        
        if (edges[i] == -1) {
            curr_valley->start_sector = i;
            in_valley = 1;
        } else if ( edges[i] == 1 && in_valley ) {
            in_valley = 0;
            curr_valley->end_sector = i;
            curr_valley->width = curr_valley->end_sector-curr_valley->start_sector;
            curr_valley->center_dir = hist->alpha * (curr_valley->start_sector + curr_valley->width/2);

            valley * new_valley = malloc(sizeof(valley));
            new_valley->next_valley = NULL;
            new_valley->start_sector = -1; // used to indicate a valley without an end
            curr_valley->next_valley = new_valley;
            curr_valley = new_valley;
        }
    }
    // handle edge case of valley traversing the 0 degree boundary
    if( curr_valley->start_sector != -1 ) {
        int i = 0;
        while( edges[i] != 1 ) {
            i++;
        }
        curr_valley->end_sector = i;
        curr_valley->width = nsectors - curr_valley->start_sector + curr_valley->end_sector;
        curr_valley->center_dir = (hist->alpha * (curr_valley->start_sector + curr_valley->width/2)) % 360;
        
        valley * new_valley = malloc(sizeof(valley));
        new_valley->next_valley = NULL;
        new_valley->start_sector = -1; // used to indicate a valley without an end
        curr_valley->next_valley = new_valley;
        curr_valley = new_valley;
    }
    
    char out[32];
    
    // choose best valley
    valley * best_v = head_valley;
    valley * v = head_valley;
    double best_dist = 180;
    int obj_sector = objective_direction/hist->alpha;
    
    while( v->next_valley != NULL ) {
        double dist_start = modular_dist(v->start_sector, obj_sector, nsectors);
        double dist_end = modular_dist(v->end_sector, obj_sector, nsectors);
        
        double dist = dist_start<dist_end ? dist_start: dist_end;
        if( dist < best_dist ) {
            best_dist = dist;
            best_v = v;
        }
        //sprintf(out, "dist: %lf\n", dist);
        //UART_PutString(out);
        v = v->next_valley;
    }
    
    valley * c = head_valley;
    while( c->next_valley != NULL ) {
        //sprintf(out, "start: %i, end: %i, center: %lf, width: %i\n", c->start_sector, c->end_sector, c->center_dir, c->width);
        //UART_PutString(out);
        c = c->next_valley;
    }
    
    double return_val;
    
    if( best_v->width <= smax ) { // narrow valley
        return_val = best_v->center_dir;   
    } else { // wide valley
        double dist_start = modular_dist(v->start_sector, obj_sector, nsectors);
        
        if( dist_start == best_dist ) {
            return_val = modulo( hist->alpha * best_v->start_sector + smax / 2, 360 );
        } else {
            return_val = modulo( hist->alpha * best_v->end_sector - smax / 2, 360 );
        }
    }
    
    //sprintf(out, "choosing angle: %lf\n", return_val);
    //UART_PutString(out);
    
    // free the linked list
    valley * tmp;
    while( head_valley != NULL ) {
        tmp = head_valley;
        head_valley = head_valley->next_valley;
        free(tmp);
    }
    
    return return_val;
}
