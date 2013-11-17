/*
 * sparsemap.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * "sparsemap" is something of a misnomer. This is a 3D hashtable that lets us
 * do "am I inside a cell?" queries very quickly.
 */

#pragma once

#include <vector>
#include <boost/unordered_set.hpp>
#include "barecell.hh"
#include "pose.hh"

namespace textured_localization
{
  class XYZ
  {
    public:
      XYZ(const BareCell& b);
      XYZ(const Pose& p);
      int x, y, z;
  };

  bool operator==(const XYZ& lhs, const XYZ& rhs);

  class XYZHasher
  {
    public:
      XYZHasher() { }
      size_t operator()(const XYZ& xyz) const;
  };

  class SparseMap
  {
    public:
      SparseMap(const std::vector<BareCell>& map);
      ~SparseMap();

      /*
       * Is this pose in an "impossible" position? At the moment, this means
       * "inside a grid square".
       */
      bool Impossible(const Pose& p);

      /*
       * Check to make sure that a Particle has occupied cells both above and
       * below it.
       */
      bool UpDownInBoundsCheck(const Pose& p);

      /*
       * Make sure we aren't too close to a wall. The offset tells us how far
       * (in meters) the camera must be from a wall, on all sides.
       */
      bool InAWall(const Pose& p, double offset = 0.2);

    private:
      boost::unordered_set<XYZ, XYZHasher> _set;
      int _min_z;
      int _max_z;
  };
}
