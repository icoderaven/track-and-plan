/*
 * random.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See random.hh.
 */

#include <boost/random.hpp>
#include "random.hh"

using namespace boost;

namespace textured_localization
{

Random* Random::Get()
{
  static Random r;
  return &r;
}

double Random::Gaussian(double mean, double stddev)
{
  return (_vg() * stddev) + mean;
}

double Random::Uniform()
{
  return _vg_u();
}

void Random::ReSeed(int seed)
{
  _vg.engine().seed((double)seed);
  _vg.distribution().reset();
}


Random::Random()
  : _mtwist((double)DEFAULT_SEED), 
    _dist(0.0, 1.0),
    _u_dist(0.0, 1.0),
    _vg(_mtwist, _dist),
    _vg_u(_mtwist, _u_dist)
{
}

}
