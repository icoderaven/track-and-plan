/*
 * kldmm.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Given a motion model (passed into the constructor), this extends it to do
 * KLD-sampling (Fox). Note that this doesn't implement the MotionModel
 * interface, as pose matters here.
 */

#pragma once

#include <vector>
#include "motionmodel.hh"
#include "particle.hh"

using namespace std;

namespace textured_localization
{
  class KLDMotionModel
  {
    public:
      /* 
       * The MM, and the permitted KL divergence.
       * Note that we take ownership of this motionmodel; don't delete it!
       *
       * The res parameters define the size of our sampling bins, in units and
       * __degrees__.
       */
      KLDMotionModel(MotionModel* mm, double epsilon, 
                     double xyres, double angleres,
                     int max_particles);
      ~KLDMotionModel();

      /*
       * This looks a lot like Sample() in MotionModel, but it uses an entire
       * filter, because it needs to know how the motionmodel distributed
       * things.
       */
      vector<Particle> Sample(const vector<Particle>& filter,
                              double dx, double dy, double dt, 
                              double old_facing); 
    private:
      MotionModel* _mm;
      double _epsilon;
      double _xyres;
      double _angleres;
      int _max_particles;

      /*
       * Our "bin" object.
       */
      class Bin
      {
        public:
          Bin(int x, int y, int z);
          int _x, _y, _z;
      };

      class BinHasher
      {
        public:
          size_t operator()(const Bin& b);
      };
  };
}
