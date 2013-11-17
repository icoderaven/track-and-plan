/*
 * bayerize.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Turn a raw image into a non-raw image.
 */

#include <iostream>
#include <cstdio>
#include <cv.h>
#include <highgui.h>
#include <boost/format.hpp>
#include "pgm.h"
#include "bayer.hh"

using namespace std;
using namespace boost;
using namespace bayer;

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    cout << "Usage: bayerize <raw image> <output image> [WxH]" << endl;
    return 1;
  }
  int width = 1600;
  int height = 1200;
  if (argc == 4)
    sscanf(argv[3], "%dx%d", &width, &height);

  IplImage* bayer = pipeline(argv[1]);
  cvSaveImage(str(format("%s_bayer.png") % argv[2]).c_str(), bayer);
  cvReleaseImage(&bayer);
  OpenCVInterpolation cv;
  pipeline(argv[1], str(format("%s.png") % argv[2]), &(cv));
}
