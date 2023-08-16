// create_histogram_grid.c -- Grid creation test.
//
// This file creates a Histogram Grid with random obstacle densities and prints
// it as a map of 'o' (high density), 'x' (medium density) and ' ' (low density).
//
// The executable accepts a command-line argument for the seed of the RNG,
// which must be an integer. For example:
//
//   ./create_histogram_grid 11
//

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../include/histogram_grid.h"

#define WIDTH 120 // Number of cells.
#define HEIGHT 120 
#define RESOLUTION 1 // Size of the cell in centimeters.

void mprint(grid * map) {
  int width = map->width;
  int height = map->height;

  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      if (map->cells[i * width + j] >= 8) {
        printf("o");
      } else if (map->cells[i * width + j] >= 4) {
        printf("x");
      } else {
        printf(" ");
      }
    }

    printf("\n");
  }
}

int main(int argc, char* argv[]) {
  grid * map = grid_init(WIDTH, HEIGHT, RESOLUTION);

  if (argc > 1) {
    int seed = atoi(argv[1]);
    srand(seed);
  }

  for (int i = 0; i < WIDTH; ++i) {
    for (int j = 0; j < HEIGHT; ++j) {
      map->cells[i * WIDTH + j] = (int) floor((rand() * 10.0) / RAND_MAX);
    }
  }

  mprint(map);
}
