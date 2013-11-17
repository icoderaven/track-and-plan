/*
 * sparsemap.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See sparsemap.hh.
 */

#include <utility>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "scale.hh"
#include "sparsemap.hh"

using namespace std;
using namespace boost;
namespace textured_localization
{

XYZ::XYZ(const BareCell& b)
  : x(b.x()), y(b.y()), z(b.z())
{
}

XYZ::XYZ(const Pose& p)
  : x((int)p.x()), y((int)p.y()), z((int)p.z())
{
}

bool operator==(const XYZ& lhs, const XYZ& rhs)
{
  return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

size_t XYZHasher::operator()(const XYZ& xyz) const
{
  return xyz.x ^ xyz.y ^ xyz.z;
}

SparseMap::SparseMap(const vector<BareCell>& map)
  : _set(), _min_z(map.at(0).z()), _max_z(map.at(0).z())
{
  foreach(const BareCell& b, map)
  {
    _set.insert(XYZ(b));
    if (b.z() < _min_z)
      _min_z = b.z();
    if (b.z() > _max_z)
      _max_z = b.z();
  }
}

SparseMap::~SparseMap()
{
}

bool SparseMap::Impossible(const Pose& p)
{
  return _set.find(XYZ(p)) != _set.end();
}

bool SparseMap::UpDownInBoundsCheck(const Pose& p)
{
  XYZ xyz(p);
  int old_z = xyz.z;
  bool up = false;
  for (int z = old_z; z <= _max_z ; ++z)
  {
    xyz.z = z;
    if (_set.find(xyz) != _set.end())
    {
      up = true;
      break;
    }
  }
  
  bool down = false;
  for (int z = old_z; z >= _min_z ; --z)
  {
    xyz.z = z;
    if (_set.find(xyz) != _set.end())
    {
      down = true;
      break;
    }
  }

  return up && down;
}

bool SparseMap::InAWall(const Pose& p, double offset)
{
  int size = (int)((offset * SCALE) + 0.5);
  if (size % 2 == 0)
    size++;

  for (int xoff = (-size/2) ; xoff < (size/2) ; ++xoff)
  {
    for (int yoff = (-size/2) ; yoff < (size/2) ; ++yoff)
    {
      if (Impossible(p))
        return true;
    }
  }
  return false;
}

}
