/*
 * motionodel.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * An interface for motion modeling.
 */

#ifndef TEXTURED_LOCALIZATION_MOTION_MODEL_HH_INCLUDED
#define TEXTURED_LOCALIZATION_MOTION_MODEL_HH_INCLUDED 1

#include <vector>

namespace textured_localization
{
  class MotionModel
  {
    public:
      // You decide on the constructor.
      virtual ~MotionModel();

      /*
       * This is the important one. Given an action (dx, dy, dt), sample from
       * whatever model this represents, and return that, in a vector, as
       * [sampled x, sampled y, sampled t].
       */
      virtual std::vector<double> Sample(double dx, 
                                         double dy, 
                                         double dt,
                                         double old_facing) = 0;
  };
}

#endif
