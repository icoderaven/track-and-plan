/*
 * random.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * A Singleton (see Gamma, et al.) encapsulating our PRNG. 
 */

#ifndef SLAM_RANDOM_HH_INCLUDED
#define SLAM_RANDOM_HH_INCLUDED 1

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

using namespace boost;

namespace textured_localization
{
  class Random
  {
    public:
      /* May as well have some value here.  */
    //static const int DEFAULT_SEED = 706; // 1 for dwing
    static const int DEFAULT_SEED = 1;

      static Random* Get();

      /* Distributions from which we can draw. */
      double Gaussian(double mean, double stddev);
      double Uniform();  // In [0, 1).

      /* Reset the PRNG seed. Handy for testing. */
      void ReSeed(int seed = DEFAULT_SEED);

    private:
      /*
       * Singleton!
       */
      Random();
      Random(const Random& rhs);
      Random& operator=(const Random& rhs);

      /*
       * The PRNG itself.
       */
      boost::mt19937 _mtwist;
      boost::normal_distribution<double> _dist;
      /*
       * TODO: The boost docs speak poorly of uniform_real in 1.37; what should
       * we do about this?
       */
      uniform_real<double> _u_dist;
      variate_generator<boost::mt19937&, boost::normal_distribution<double> > _vg;
      variate_generator<boost::mt19937&, uniform_real<double> > _vg_u;
  };
}


#endif 
