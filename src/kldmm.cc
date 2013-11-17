/*
 * kldmm.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implentation. See kldmm.hh.
 */

#include <boost/unordered_set.hpp>
#include "kldmm.hh"

using namespace std;

namespace textured_localization
{

KLDMotionModel::KLDMotionModel(MotionModel* mm, 
                               double epsilon,
                               double xyres,
                               double angleres,
                               int max_particles)
  : _mm(mm), _epsilon(epsilon), _xyres(xyres), _angleres(angleres),
    _max_particles(max_particles)
{
}

KLDMotionModel::~KLDMotionModel()
{
}

vector<Particle> Sample(const vector<Particle>& filter,
                        double dx, double dy, double dt,
                        double old_facing)
{
  // Our (eventual) result.
  vector<Particle> res;
  // Our bins.
  unordered_set<Bin, BinHasher> bins;
  double kld = _epsilon * 10;
  while (kld > _epsilon)
  {
    vector<double> action = _mm->Sample(dx, dy, dt);
  }


}


}
