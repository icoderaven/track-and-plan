/*
 * simplemotionmodel.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation; see simplemotionmodel.hh.
 */

#include <cmath>
#include <vector>
#include "simplemotionmodel.hh"
using namespace std;

namespace textured_localization
{

SimpleMotionModel::SimpleMotionModel(double tmean,
                                     double rmean,
                                     double tstddev,
                                     double rstddev)
  : _tmean(tmean), _rmean(rmean), _tstddev(tstddev), _rstddev(rstddev),
    _mtwist(1.0), _dist(0.0, 1.0), _vg(_mtwist, _dist)
{
}

SimpleMotionModel::~SimpleMotionModel()
{
  // Nothin'!
}

vector<double> SimpleMotionModel::Sample(double dx, double dy, double dt, 
                                         double old_facing)
{
  // How far did we go?
  double dist = sqrt((dx * dx) + (dy * dy));

  // Sample that
  double sdist = (_vg() * dist * _tstddev) + (dist * _tmean);
  //cout << "dist: " << dist << " sdist: " << sdist << endl;
  double nx = sdist * cos(old_facing + (dt/2));
  double ny = sdist * sin(old_facing + (dt/2));

  // Sample our rotation.
  double srot = (_vg() * _rstddev * dt) + (_rmean * dt);
  //cout << "dt: " << dt << " srot: " << srot << endl;
  vector<double> res;
  res.push_back(nx);
  res.push_back(ny);
  res.push_back(srot);

  return res;
}

} // namespace textured_localization
