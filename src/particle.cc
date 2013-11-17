/*
 * particle.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See particle.hh.
 */

#include <cassert>
#include <cmath>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "particle.hh"

namespace textured_localization
{

Particle::Particle()
  : _pose(), _weight(1.0), _poses()
{
}

Particle::Particle(Pose p, double weight)
  : _pose(p), _weight(weight), _poses()
{
}

Particle::Particle(const Particle& rhs)
  : _pose(rhs._pose), _weight(rhs._weight), _poses(rhs._poses)
{
}

Particle& Particle::operator=(const Particle& rhs)
{
  if (&rhs == this)
    return *this;

  _pose = rhs._pose;
  _weight = rhs._weight;
  _poses = rhs._poses;

  return *this;
}

Particle::~Particle()
{
}

Pose Particle::pose() const
{
  return _pose;
}

vector<Pose> Particle::poses() const
{
  return _poses;
}

void Particle::Move(double dx, double dy, double dt)
{
  _poses.push_back(_pose);
  _pose.set(_pose.x() + dx, _pose.y() + dy, _pose.z(), _pose.theta() + dt);
}

void Particle::SetPose(const Pose& p)
{
  _pose = p;
}

double Particle::weight() const
{
  return _weight;
}

void Particle::set_weight(double w)
{
  _weight = w;
}

void Particle::Normalize(std::vector<Particle>& filter)
{
  double sum = 0.0;
  foreach(Particle p, filter)
  {
    sum += p.weight();
  }
  assert (sum > 0.0);

  foreach(Particle& p, filter)
  {
    p.set_weight(p.weight() / sum);
  }
}

bool operator<(const Particle& lhs, const Particle& rhs)
{
  return (lhs.weight() < rhs.weight());
}

} // namespace textured_localization
