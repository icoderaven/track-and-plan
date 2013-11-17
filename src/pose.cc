/*
 * pose.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See pose.cc
 */

#include <cmath>
#include <algorithm>
#include <vector>
#include "utilities.hh"
#include "pose.hh"

using namespace std;

namespace textured_localization
{

Pose::Pose()
  : _x(0.0), _y(0.0), _z(0.0), _theta(0.0)
{
  _theta = normalize_angle(_theta);
}

Pose::Pose(double x, double y, double z, double theta)
  : _x(x), _y(y), _z(z), _theta(theta)
{
  _theta = normalize_angle(_theta);
}

Pose::Pose(const Pose& rhs)
  : _x(rhs._x), _y(rhs._y), _z(rhs._z), _theta(rhs._theta)
{
}

Pose& Pose::operator=(const Pose& rhs)
{
  if (&rhs == this)
    return *this;

  _x = rhs._x;
  _y = rhs._y;
  _z = rhs._z;
  _theta = rhs._theta;
  return *this;
}

Pose::~Pose()
{
}

double Pose::x() const
{
  return _x;
}

double Pose::y() const
{
  return _y;
}

double Pose::theta() const
{
  return _theta;
}

void Pose::set_z(double z)
{
  _z = z;
}

void Pose::set_theta(double t)
{
  _theta = normalize_angle(t);
}

void Pose::set(double x, double y, double z, double theta)
{
  _x = x;
  _y = y;
  _z = z;
  _theta = normalize_angle(theta);
}

vector<double> Pose::BoundingBox(const vector<Pose>& poses)
{
  double xmin, ymin, xmax, ymax;
  xmin = xmax = poses[0].x();
  ymin = ymax = poses[0].y();
  for (size_t i = 0 ; i < poses.size() ; ++i)
  {
    if (poses[i].x() < xmin)
      xmin = poses[i].x();
    if (poses[i].x() > xmax)
      xmax = poses[i].x();
    if (poses[i].y() < ymin)
      ymin = poses[i].y();
    if (poses[i].y() > ymax)
      ymax = poses[i].y();
  }

  vector<double> res;
  res.push_back(xmin);
  res.push_back(xmax);
  res.push_back(ymin);
  res.push_back(ymax);
  return res;
}


Pose operator+(const Pose& lhs, const Pose& rhs)
{
  using namespace std;
  double dist = sqrt((rhs.x() * rhs.x()) + (rhs.y() * rhs.y()));
  double dx = dist * cos(lhs.theta());
  double dy = dist * sin(lhs.theta());
  return Pose(lhs.x() + dx,
              lhs.y() + dy,
              lhs.z(),
              lhs.theta() + rhs.theta());
}

Pose operator-(const Pose& lhs, const Pose& rhs)
{
  return Pose(lhs.x() - rhs.x(),
              lhs.y() - rhs.y(),
              lhs.z() - rhs.z(),
              lhs.theta() - rhs.theta());
}

std::ostream& operator<<(std::ostream& out, const Pose& rhs)
{
  out << rhs.x() << " " 
      << rhs.y() << " "
      << rhs.z() << " "
      << rhs.theta();
  return out;
}

std::istream& operator>>(std::istream& in, Pose& rhs)
{
  double x, y, z, theta;
  in >> x >> y >> z >> theta;
  rhs.set(x, y, z, theta);

  return in;
}

}
