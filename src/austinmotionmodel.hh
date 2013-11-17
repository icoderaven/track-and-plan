/*
 * austinmotionmodel.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Austin's three-part motion model. This means we'll be able to use exactly
 * the same MM in our experiments as we do in 3D SLAM.
 */

#ifndef TEXTURED_LOCALIZATION_AUSTINMOTIONMODEL_HH_INCLUDED
#define TEXTURED_LOCALIZATION_AUSTINMOTIONMODEL_HH_INCLUDED 1

#include <vector>
#include "motionmodel.hh"

namespace textured_localization
{
  class KVParser;
  class AustinMotionModel : public MotionModel
  {
    public:
      // There are so many params; we'll keep them in a config file.
      AustinMotionModel(KVParser& config);
      ~AustinMotionModel();

      // Default everything-else will work just fine.

      std::vector<double> Sample(double dx, double dy, double dt,
                                 double old_facing);
    private:
      // MM means
      double _meanD_D;
      double _meanD_T;
      double _meanC_D;
      double _meanC_T;
      double _meanT_D;
      double _meanT_T;

      // MM stddevs
      double _stddevD_D;
      double _stddevD_T;
      double _stddevC_D;
      double _stddevC_T;
      double _stddevT_D;
      double _stddevT_T;

      // Flooring parameters.
      double _min_trans_stddev;
      double _min_rot_stddev;
  };
}

#endif
