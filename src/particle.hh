/*
 * particle.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * A (very!) simple Particle.
 */

#ifndef TEXTURED_LOCALIZATION_PARTICLE_HH_INCLUDED
#define TEXTURED_LOCALIZATION_PARTICLE_HH_INCLUDED 1

#include <vector>
#include "pose.hh"

using namespace std;

namespace textured_localization
{
  class Particle
  {
    public:
      Particle();  // At (0, 0, 0, 0), weight = 1.
      Particle(Pose p, double weight);
      Particle(const Particle& rhs);
      Particle& operator=(const Particle& rhs);
      ~Particle();

      // Where am I?
      Pose pose() const;

      // All the places I used to be. My entire trajectory is poses + pose.
      vector<Pose> poses() const;

      // INCREMENT the stored pose. 
      // This actually adds the current (unmodified) pose to poses, then sets
      // pose to the moved version.
      void Move(double dx, double dy, double dt);
      // Set the pose (Ignores poses)
      void SetPose(const Pose& p); 

      // Weighting.
      double weight() const;
      void set_weight(double w);
    private:
      Pose _pose;
      double _weight;
      vector<Pose> _poses;

    /* Useful helper functions. */
    public:
      // Make the weights sum to 1.
      static void Normalize(std::vector<Particle>& filter);
  };

  // Sort in order of increasing weight.
  bool operator<(const Particle& lhs, const Particle& rhs);
}

#endif

