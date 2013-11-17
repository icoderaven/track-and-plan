/*
 * linearize.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Linearize the sRGB space of an image.
 */

#include <iostream>
#include <cmath>
#include <cxcore.h>
#include <highgui.h>
#include "linear.hh"

using namespace std;


int main(int argc, char* argv[])
{
  if (argc < 3 || argc > 4)
  {
    cout << "Usage: ./linearize <input file> <output file> [factor]" << endl;
    return 1;
  }

  IplImage* in = cvLoadImage(argv[1]);
  if (argc == 3)
    image_srgb_to_linear(in);
  else
    exposure_compensation(in, atof(argv[3]));
  cvSaveImage(argv[2], in);
  cvReleaseImage(&in);
}
