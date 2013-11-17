/*
 * sensormodel.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation; see sensormodel.hh.
 */

#include <iostream>
#include "sensormodel.hh"
using namespace std;

namespace textured_localization
{
  IplImage* SensorModel::Inpaint(IplImage* im, 
                                 const CvScalar& color,
                                 bool* uncertain,
                                 bool del,
                                 double lower_thresh,
                                 double upper_thresh)
  {
    int area = im->width * im->height;
    IplImage* res = cvCloneImage(im);
    IplImage* mask = 
      cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_8U, 1);
    cvSet(mask, cvScalarAll(0.0));
    int green_pixels = 0;
    int yellow_pixels = 0;
    for (int i = 0 ; i < im->height ; ++i)
    {
      for (int j = 0 ; j < im->width ; ++j)
      {
        CvScalar c = cvGet2D(im, i, j);
        if (c.val[1] == 255.0 && c.val[2] == 255.0)
        {
          cvSet2D(mask, i, j, cvScalarAll(255.0));
          yellow_pixels++;
        }
        else if (c.val[1] == 255.0)
        {
          cvSet2D(mask, i, j, cvScalarAll(255.0));
          green_pixels++;
        }
      }
    }
//    cout << "We have " << green_pixels << " greens and " 
//         << yellow_pixels << " yellows." << endl;

    *uncertain = false;
    if (area - green_pixels < lower_thresh * area)
    {
      cvReleaseImage(&mask);
      if (del)
        cvReleaseImage(&im);
      cvReleaseImage(&res);
      return NULL;
    }
    cvInpaint(im, mask, res, 3.0, CV_INPAINT_NS);
    if (area - yellow_pixels < upper_thresh * area)
    {
      *uncertain = true;
    }

    cvReleaseImage(&mask);
    if (del)
      cvReleaseImage(&im);
    return res;
  }

  IplImage* SensorModel::ColorConversion(IplImage* im,
                                         const TNT::Array2D<double>& matrix,
                                         bool del)
  {
    using namespace std;
    using namespace TNT;

    // OpenCV makes me angry, so we're going to do this all in TNT land. That
    // involves some extra copies, but oh well.
    
    Array2D<double> image(im->height * im->width, 4, 0.0);
    int image_idx = 0;
    for (int row = 0 ; row < im->height ; ++row)
    {
      for (int col = 0 ; col < im->width ; ++col)
      {
        CvScalar s = cvGet2D(im, row, col);
        image[image_idx][0] = s.val[2];
        image[image_idx][1] = s.val[1];
        image[image_idx][2] = s.val[0];
        image[image_idx][3] = 1.0; // Homogeneous

        image_idx++;
      }
    }

    // Now, multiply.
    Array2D<double> matrix2(matrix);
    for (int i = 0; i < 3 ; ++i)
      matrix2[3][i] = 0.0;
    Array2D<double> res = matmult(image, matrix2);
    // And convert back.
    IplImage* result = cvCloneImage(im);
    image_idx = 0;
    for (int row = 0 ; row < im->height ; ++row)
    {
      for (int col = 0 ; col < im->width ; ++col)
      {
        CvScalar s = cvScalar(res[image_idx][2], 
                              res[image_idx][1],
                              res[image_idx][0]);
        cvSet2D(result, row, col, s);
        image_idx++;
      }
    }

    if (del)
      cvReleaseImage(&im);
    return result;
  }

  double SensorModel::Entropy(double* array, int length /* fuck you, C */)
  {
    double total = 0.0;
    for (int i = 0 ; i < length ; ++i)
      total += array[i];
    for (int i = 0 ; i < length ; ++i)
      array[i] /= total;

    double ent = 0.0;
    for (int i = 0 ; i < length ; ++i)
      if (array[i] != 0.0)
        ent += array[i] * log(array[i]);

    return -ent;
  }

  IplImage* SensorModel::Normalize(IplImage* image, bool del)
  {
    // Generate our result.
    IplImage* res = 
      cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_64F, 3);
    // Copy in.
    cvConvertScale(image, res);

    // Get the statistics.
    pair<double, double> stats = MeanAndStddev(res);

    // Subtract off the mean.
    cvConvertScale(res, res, 1.0, -stats.first);

    // Divide by the standard deviation.
    cvConvertScale(res, res, 1.0 / stats.second, 0);

    if (del) 
      cvReleaseImage(&image);
    return res;
  }

  pair<double, double> SensorModel::MeanAndStddev(IplImage* image)
  {
    double sum = 0.0;
    int n = image->width * image->height * 3;

    for (int i = 0 ; i < image->height ; ++i)
    {
      for (int j = 0 ; j < image->width ; ++j)
      {
        CvScalar s = cvGet2D(image, i, j);
        sum += s.val[0] + s.val[1] + s.val[2];
      }
    }
    double mean = sum / n;
    double temp = 0.0;
    for (int i = 0 ; i < image->height ; ++i)
    {
      for (int j = 0 ; j < image->width ; ++j)
      {
        CvScalar s = cvGet2D(image, i, j);
        temp += (s.val[0] - mean) * (s.val[0] - mean);
        temp += (s.val[1] - mean) * (s.val[1] - mean);
        temp += (s.val[2] - mean) * (s.val[2] - mean);
      }
    }

    double stddev = sqrt(temp / n);
    return make_pair(mean, stddev);
  }

  IplImage* SensorModel::NormalizePerChannel(IplImage* image, bool del)
  {
    // Generate our result.
    IplImage* res = 
      cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_64F, 3);
    
    // Copy in.
    cvConvertScale(image, res);

    // Build the per-channel images.
    IplImage* red = 
      cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_64F, 1);
    IplImage* green = 
      cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_64F, 1);
    IplImage* blue = 
      cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_64F, 1);

    // Break the channels out.
    cvSplit(res, blue, green, red, NULL);

    // Get the averages and standard deviations.
    CvScalar means;
    CvScalar stddevs;
    cvAvgSdv(image, &means, &stddevs);
    // Subtract off the means.
    cvConvertScale(blue, blue, 1.0, -means.val[0]);
    cvConvertScale(green, green, 1.0, -means.val[1]);
    cvConvertScale(red, red, 1.0, -means.val[2]);

    // Divide by the standard deviations.
    cvConvertScale(blue, blue, 1.0 / stddevs.val[0], 0.0);
    cvConvertScale(green, green, 1.0 / stddevs.val[1], 0.0);
    cvConvertScale(red, red, 1.0 / stddevs.val[2], 0.0);

    // Merge 'em back together.
    cvMerge(blue, green, red, NULL, res);

    // Try it out.
    cvAvgSdv(res, &means, &stddevs);
    // Leaks are bad, mmmkay?
    cvReleaseImage(&red);
    cvReleaseImage(&green);
    cvReleaseImage(&blue);

    if (del)
      cvReleaseImage(&image);
    return res;
  }
}
