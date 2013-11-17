/*
 * simplemotionmodel.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Does a very simple, two-Gaussians, motion model.
 */

#ifndef TEXTURED_LOCALIZATION_SIMPLE_MOTION_MODEL_HH_INCLUDED
#define TEXTURED_LOCALIZATION_SIMPLE_MOTION_MODEL_HH_INCLUDED 1

#include <vector>
#include <boost/random.hpp>
#include "motionmodel.hh"

using namespace boost;

namespace textured_localization
{
  class SimpleMotionModel : public MotionModel
  {
    public:
      SimpleMotionModel(double tmean,
                        double rmean,
                        double tstddev,
                        double rstddev);
      ~SimpleMotionModel();

      std::vector<double> Sample(double dx, 
                                 double dy, 
                                 double dt,
                                 double old_facing);

    private:
      // Parameters
      double _tmean;
      double _rmean;
      double _tstddev;
      double _rstddev;

      // The PRNG stuff.
      boost::mt19937 _mtwist;
      boost::normal_distribution<double> _dist;
      uniform_real<double> _u_dist;
      variate_generator< boost::mt19937&, boost::normal_distribution<double> > _vg;
  };
}

#endif

