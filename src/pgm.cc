/*

Copyright(c) Ronald Parr 2005
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization

PGM utilities

*/

#include <stdlib.h>
#include <stdio.h>
#include "pgm.h"

//
void get_pgm_dimesions_f(char *fname, int* width, int* height)
{
  FILE *infile;

  infile = fopen(fname,"r");
  get_pgm_dimensions(infile, width, height);
}


// Need to skip comments in pgm file
void get_pgm_dimensions(FILE *infile, int* width, int* height)
{
  fscanf(infile, "P5\n %d %d \n 255\n", width, height);  /* read dimensions */
  //  fprintf(stderr, "Image dimensions:  %d, %d\n", *width, *height); 
}

/*
  Load a PGM pointed to by file.

  If image is NULL, allocate the necessary memory.

*/

unsigned char* loadpgm(FILE *infile, unsigned char* image, int* width, int* height)
{
  fscanf(infile, "P5\n %d %d \n 255\n", width, height);  /* read dimensions */
  //  fprintf(stderr, "Image dimensions:  %d, %d\n", *width, *height); 
  if (image == NULL)
    image = (unsigned char*)malloc(*width * *height * sizeof(unsigned char));
  if (image == NULL) {
    fprintf(stderr, "Memory allocation error.\n");
    exit(1);
  }
  fread(image, 1, *width * *height, infile);
  return image;
}


/*
  Write the PGM pointed to by file.

  This should move moved out to a separate file.

*/

 
void writepgm(FILE *outfile, unsigned char* image, int width, int height)
{
  fprintf(outfile,"P5\n%u %u 255\n", width, height ); 
  fwrite(image, 1, height*width, outfile);
}

void writeImage(char * filename, unsigned char* image, int width, int height)
{
  FILE *f;
  //  fprintf(stderr, "Opening file %s.\n", filename);
  f = fopen(filename, "w");
  //  fprintf(stderr, "File opened.\n");
  if (f == NULL) {
    fprintf(stderr, "Error opening file: %s\n", filename);
    exit(1);
  }
  writepgm(f, image, width, height);
  //  fprintf(stderr, "Wrote %s.\n", filename);
  fclose(f);
}
