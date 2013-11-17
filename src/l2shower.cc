/*
 * l2shower.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Just show what the L2 norm between two images (scaled) looks like.
 */

#include <iostream>
#include <cmath>
#include <cv.h>
#include <highgui.h>
#include "utilities.hh"

using namespace std;
using namespace textured_localization;

double norm(CvScalar a, CvScalar b)
{
  double total = 0;
  for (int i = 0;  i < 3 ; ++i)
    total += pow(a.val[i] - b.val[i], 2);
  return sqrt(total);
}

int main(int argc, char* argv[])
{
  if (argc != 4)
  {
    cout << "Usage: bin/l2shower <input> <ref> <output>" << endl;
    return 1;
  }

  IplImage* one = cvLoadImage(argv[1]);
  IplImage* two = cvLoadImage(argv[2]);

  IplImage* res = 
    cvCreateImage(cvSize(one->width, one->height), IPL_DEPTH_64F, 1);

  for (int i = 0 ; i < res->height ; ++i)
  {
    for (int j = 0 ; j < res->width ; ++j)
    {
      double n = norm(cvGet2D(one, i, j), cvGet2D(two, i, j));
      cvSet2D(res, i, j, cvScalarAll(n));
    }
  }

  double min = minimum_element(res);
  double max = maximum_element(res);

  for (int i = 0 ; i < res->height ; ++i)
  {
    for (int j = 0 ; j < res->width ; ++j)
    {
      CvScalar v = cvGet2D(res, i, j);
      CvScalar n = cvScalarAll(((v.val[0] - min) / max) * 255.0);
      cvSet2D(res, i, j, n);
    }
  }

  cvSaveImage(argv[3], res);
  cvReleaseImage(&one);
  cvReleaseImage(&two);
  cvReleaseImage(&res);
}
