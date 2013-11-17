/*
 * bayer.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See bayer.hh
 */

#include <iostream>
#include <string>
#include <cstdio>
#include <cv.h>
#include <highgui.h>
#include "bayer.hh"

using namespace std;

namespace bayer
{

void four_values(unsigned char five[], int out[])
{
   // The first ten bits
  out[0] = five[0];
  out[0] <<= 2;
  out[0] += ((five[1] >> 6) & BOTTOM_TWO);

  // We first need the bottom six bits of five[1].
  out[1] = five[1];
  out[1]  = (out[1] << 4) & ONLY_TEN;
  // And the top four bits of five[2].
  out[1] += (five[2] >> 4) & (BOTTOM_TWO | L_MIDDLE_TWO);

  // Now we need the bottom four bits of five[2].
  out[2] = five[2];
  out[2] = (out[2] << 6) & ONLY_TEN;
  // And the top six of five[3].
  out[2] += (five[3] >> 2) & (~TOP_TWO);

  // Now we need the bottom two of five[3]
  out[3] = five[3];
  out[3] = (out[3] << 8) & ONLY_TEN;
  // And all of five[4].
  out[3] += five[4];
}

void parse(const string& filename, int values[], bool ten_bits)
{
  FILE* F = fopen(filename.c_str(), "r");
  if (F == NULL)
  {
    cout << "Failed to open " << filename << endl;
    return;
  }

  int vidx = 0;
  unsigned char buffer[5];
  int vals[4];
  while (fread(buffer, 1, 5, F) == 5)
  {
    if (ten_bits)
    {
      swap(buffer[0], buffer[4]);
      swap(buffer[1], buffer[3]);
      four_values(buffer, vals);
      swap(vals[0], vals[3]);
      swap(vals[1], vals[2]);
      for (int i = 0 ; i < 4 ; ++i)
        values[vidx + i] = vals[i];
      vidx += 4;
    }
    else
    {
      for (int i = 0 ; i < 5 ; ++i)
        values[vidx + i] = buffer[i];
      vidx += 5;
    }
  }

  fclose(F);
}

void raw_image(int values[], IplImage* im)
{
  int vidx = 0;
  for (int y = 0 ; y < im->height ; ++y)
  {
    for (int x = 0; x < im->width ; ++x)
    {
      cvSet2D(im, y, x, cvScalar(values[vidx] / 4));
      vidx++;
    }
  }
}

OpenCVInterpolation::OpenCVInterpolation(int bayer_type)
  : _bayer_type(BAYER_TYPES[bayer_type])
{
};

IplImage* OpenCVInterpolation::operator()(IplImage* im)
{
  IplImage* res = 
    cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_8U, 3);
  cvCvtColor(im, res, _bayer_type);
  return res;
}

IplImage* pipeline(const string& input_filename,
                   InterpolationInterface* ii,
                   int width,
                   int height)
{
  int* buffer = new int[width * height];
  parse(input_filename.c_str(), buffer);
  IplImage* im = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
  raw_image(buffer, im);
  IplImage* interped = NULL;
  if (ii != NULL)
  {
    interped = (*ii)(im);
    cvReleaseImage(&im);
  }
  else
  {
    interped = im;
  }
  delete[] buffer;
  return interped;
}

void pipeline(const string& input_filename,
              const string& output_filename,
              InterpolationInterface* ii, 
              int width, 
              int height)
{
  IplImage* im = pipeline(input_filename, ii, width, height);
  cvSaveImage(output_filename.c_str(), im);
  cvReleaseImage(&im);
}

}  // namespace bayer
