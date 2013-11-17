/*
 * differ.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * How different are two grayscale images?
 */

#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;

int main(int argc, char* argv[])
{
  if (argc != 4)
  {
    cout << "Usage: bin/differ <img1> <img2> <output>" << endl;
    return 1;
  }

  IplImage* im1 = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  IplImage* im2 = cvLoadImage(argv[2], CV_LOAD_IMAGE_GRAYSCALE);

  IplImage* diff = cvCloneImage(im1);
  for (int i = 0 ; i < diff->height ; ++i)
  {
    for (int j = 0 ; j < diff->width ; ++j)
    {
      cvSet2D(diff, i, j, cvScalarAll(fabs(cvGet2D(im1, i, j).val[0] - 
                                           cvGet2D(im2, i, j).val[0])));
    }
  }

  cvSaveImage(argv[3], diff);
  cvReleaseImage(&diff);
  cvReleaseImage(&im1);
  cvReleaseImage(&im2);
}
