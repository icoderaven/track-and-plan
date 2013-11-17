/*
 * utilities.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Just a few random, useful functions to keep around. Most have to do with
 * angles & orientations and the like.
 */

#ifndef TEXTURED_LOCALIZATION_UTILITIES_HH_INCLUDED
#define TEXTURED_LOCALIZATION_UTILITIES_HH_INCLUDED 1

#include <utility>
#include <cv.h>
#include "TNT/tnt.h"
#include "pose.hh"

namespace textured_localization
{
  /* Angular conversion. */
  double radians(double degrees);
  double degrees(double radians);

  /* Move an angle (in radians) into the range [-pi, pi]. */
  double normalize_angle(double angle);

  // Deal specifically with single-channel double-precision IplImages.
  double maximum_element(IplImage* arr);
  double minimum_element(IplImage* arr);

  /* 
   * Convert a (double-precision, single-channel) IplImage into an
   * Array2D<double>. The idea here is to write down the unscaled,
   * fully-detailed heatmap, so that we can examine it in more detail.
   */
   TNT::Array2D<double> image_to_array(IplImage* arr);

  /*
   * Poses are in radians, so we need to be able to turn a location and
   * looking-at point into a pose object.
   */
  Pose VectorsToPose(const TNT::Array2D<double>& pose, 
                     const TNT::Array2D<double>& facing);
}

#endif
