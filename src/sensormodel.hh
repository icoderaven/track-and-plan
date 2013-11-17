/*
 * sensormodel.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Interface for sensor models.
 */

#ifndef TEXTURED_LOCALIZATION_SENSORMODEL_HH_INCLUDED
#define TEXTURED_LOCALIZATION_SENSORMODEL_HH_INCLUDED 1

#include <utility>
#include <cv.h>

#include <opencv2/photo/photo.hpp>
#include "TNT/tnt.h"

using namespace std;

namespace textured_localization
{
  class SensorModel
  {
    public:
      // You decide how your constructor works; this is just to have one.
      SensorModel() {};
      virtual ~SensorModel() {};

      /*
       * Return the probability of a reading.
       */
      virtual double operator()(IplImage* reference, IplImage* reading) = 0;

      /*
       * Do an inpainting operation. This returns a new image, which you must
       * take responsibility for releasing. If del is true, the passed-in
       * image is deleted for you, making it possible to safely do this:
       *
       * IplImage* blah = ...
       *
       * blah = Inpaint(blah, ..., true).
       *
       * Thresh defines the percentage of the pixels that must be real (not in
       * the mask). If this threshold is not met, this function returns NULL.
       */
      IplImage* Inpaint(IplImage* im, 
                        const CvScalar& color, 
                        bool* uncertain,
                        bool del=true,
                        double lower_thresh=0.5,
                        double upper_thresh=0.975);

      /*
       * The robot's color space is not quite the same as the Riegl's color
       * space (I'm using "space" here in a very vague way). This applies a
       * color-conversion matrix (which you can calculate using, say,
       * naive_color_calibrator.py) and applies it to the sent-in image. The
       * returned image is allocated internally; the del parameter does just
       * what you expect.
       */
      IplImage* ColorConversion(IplImage* im, 
                                const TNT::Array2D<double>& matrix,
                                bool del = true);

      /*
       * Should the entropy function go here? Probably not. OH WELL. This
       * normalizes the passed-in array, so you don't need to worry about
       * that.
       */
      double Entropy(double* array, int length /* fuck you, C */);

      /*
       * Make the input image have mean 0 and standard deviation 1. Note that
       * the returned image WON'T be an 8UC3. The del parameter does what it
       * does in Inpaint, above. 
       */
      IplImage* Normalize(IplImage* image, bool del = true);

      /* 
       * Compute the mean and standard deviation of an image. Unlike the
       * OpenCV function, this does does not treat channels as independent;
       * you get just one value for each.
       */
      pair<double, double> MeanAndStddev(IplImage* image);

      /* 
       * Another way of normalizing is to normalize each channel
       * independently. This function does that; it treats each channel
       * independently, and makes them mean-zero and stddev-1.
       * 
       * The del parameter does what it does throughout this file.
       */
      IplImage* NormalizePerChannel(IplImage* image, bool del = true);
  };
}

#endif
