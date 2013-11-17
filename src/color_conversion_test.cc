/*
 * color_conversion_test.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Does our color-conversion thing do what we want?
 */

#include <iostream>
#include <sstream>
#include <cv.h>
#include <highgui.h>
#include "TNT/tnt.h"
#include "extended_tnt.hh"
#include "sensormodel.hh"
#include "sensormodels.hh"
#include "kvparser.hh"

using namespace std;
using namespace TNT;
using namespace textured_localization;

int main(int argc, char* argv[])
{
  if (argc != 4)
  {
    cout << "Usage: bin/color_conversion_test <config> <in> <out>" << endl;
    return 1;
  }

  KVParser config(argv[1]);
  if (config["color_conversion"] == "")
  {
    cout << "No color conversion matrix supplied!" << endl;
    return 2;
  }

  stringstream ss(config["color_conversion"]);
  Array2D<double> matrix;
  ss >> matrix;

  IplImage* in = cvLoadImage(argv[2]);
  IplImage* conv = L2SensorModel(1).ColorConversion(in, matrix, true);
  cvSaveImage(argv[3], conv);
  cvReleaseImage(&conv);
}
