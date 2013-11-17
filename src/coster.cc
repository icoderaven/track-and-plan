/*
 * coster.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Figure out how much various images cost.
 */

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include "sensormodels.hh"

using namespace std;
using namespace textured_localization;

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    cout << "Usage: bin/coster <ref> <probe> [diff]" << endl;
    return 1;
  }

  IplImage* ref_big = cvLoadImage(argv[1]);
  IplImage* probe = cvLoadImage(argv[2]);
  IplImage* ref = cvCreateImage(cvSize(probe->width, probe->height),
                                IPL_DEPTH_8U, 3);
  cvResize(ref_big, ref);
  cvReleaseImage(&ref_big);
  assert(ref->width == probe->width);
  assert(ref->height == probe->height);

  cvNamedWindow("Reference");
  cvShowImage("Reference", ref);
  cvNamedWindow("Probe");
  cvShowImage("Probe", probe);
  cvNamedWindow("Difference");

  L1HueSensorModel HS(ref->width, ref->height);
  GrayScaleL2SensorModel GSL2S(1, true);
  double prob = GSL2S(ref, probe);
  cout << "Prob is " << prob << endl;

  IplImage* diff = 
    cvCreateImage(cvSize(ref->width, ref->height), IPL_DEPTH_8U, 1);

  IplImage* ref_gray = 
    cvCreateImage(cvSize(ref->width, ref->height),
                  IPL_DEPTH_8U, 1);
  IplImage* read_gray = 
    cvCreateImage(cvSize(ref->width, ref->height),
                  IPL_DEPTH_8U, 1);

  cvCvtColor(ref, ref_gray, CV_BGR2GRAY);
  cvCvtColor(probe, read_gray, CV_BGR2GRAY);
  IplImage* old_probe = cvCloneImage(read_gray);
  read_gray = GSL2S.MatchMean(ref_gray, read_gray, true);
  cvShowImage("Reference", ref_gray);
  cvShowImage("Probe", read_gray);
  cvNamedWindow("OldProbe");
  cvShowImage("OldProbe", old_probe);

  for (int i = 0; i < diff->height ; ++i)
  {
    for (int j = 0 ; j < diff->width ; ++j)
    {
      double err = 
        L2HueSensorModel::HueError(cvGet2D(ref, i, j), cvGet2D(probe, i, j));
      assert(err >= 0.0 && err <= 180.0);
      double x = sqrt(pow(cvGet2D(ref_gray, i, j).val[0] - 
                          cvGet2D(read_gray, i, j).val[0], 2));
      cvSet2D(diff, i, j, cvScalarAll(x));
    }
  }

  cvShowImage("Difference", diff);
  if (argc == 4)
    cvSaveImage(argv[3], diff);
  while (true)
  {
    char c = cvWaitKey(30);
    if (c == 'q')
      break;
  }

  cvDestroyWindow("Difference");
  cvDestroyWindow("Probe");
  cvDestroyWindow("Reference");
  cvReleaseImage(&ref);
  cvReleaseImage(&probe);
  cvReleaseImage(&diff);
}
