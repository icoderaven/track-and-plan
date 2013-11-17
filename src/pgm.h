/*

pgm.h

Copyright(c) Ronald Parr 2005

PGM utilities

*/

void get_pgm_dimensions(FILE *infile, int* width, int* height); unsigned char*
loadpgm(FILE *infile, unsigned char* image, int* width, int* height); void
writepgm(FILE *outfile, unsigned char* image, int width, int height); void
writeImage(char * filename, unsigned char* image, int width, int height); void
get_pgm_dimesions_f(char *fname, int* width, int* height);
