/*
 * barecell.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See barecell.hh.
 */

#include <cassert>
#include <fstream>
#include "barecell.hh"

namespace textured_localization
{

BareCell::BareCell()
  : _x(0), _y(0), _z(0), _h(0), _odo(0), _pixel_count(), _r(), _g(), _b(),
    _avg_r(0), _avg_g(0), _avg_b(0), _total_pixel_count(0)
{
  for (int i = 0 ; i < 6 ; ++i)
  {
    _pixel_count.push_back(0.0);
    _r.push_back(0.0);
    _g.push_back(0.0);
    _b.push_back(0.0);
  }
}

BareCell::BareCell(int x, int y, int z, 
                   int h, double odo, 
                   vector<double> pixel_count, 
                   vector<double> r, 
                   vector<double> g, 
                   vector<double> b)
  : _x(x), _y(y), _z(z), 
    _h(h), _odo(odo), 
    _pixel_count(pixel_count), _r(r), _g(g), _b(b),
    _avg_r(0), _avg_g(0), _avg_b(0), _total_pixel_count(0)
{
  for (int i = 0; i < 6 ; ++i)
  {
    _avg_r += _r[i];
    _avg_g += _g[i];
    _avg_b += _b[i];
    _total_pixel_count += _pixel_count[i];
  }
  _avg_r /= (255.0 * _total_pixel_count);
  _avg_g /= (255.0 * _total_pixel_count);
  _avg_b /= (255.0 * _total_pixel_count);

  for (int i = 0 ; i < 6 ; ++i)
  {
    _r[i] /= (255.0 * _pixel_count[i]);
    _g[i] /= (255.0 * _pixel_count[i]);
    _b[i] /= (255.0 * _pixel_count[i]);
  }
}

BareCell::BareCell(const BareCell& rhs)
  : _x(rhs._x), _y(rhs._y), _z(rhs._z), 
    _h(rhs._h), _odo(rhs._odo), 
    _pixel_count(rhs._pixel_count), _r(rhs._r), _g(rhs._g), _b(rhs._b),
    _avg_r(rhs._avg_r), _avg_g(rhs._avg_g), _avg_b(rhs._avg_b),
    _total_pixel_count(rhs._total_pixel_count)
{
}

BareCell& BareCell::operator=(const BareCell& rhs)
{
  if (this == &rhs)
    return *this;

  _x = rhs._x;
  _y = rhs._y;
  _z = rhs._z;
  _h = rhs._h;
  _odo = rhs._odo;
  _pixel_count = rhs._pixel_count;
  _r = rhs._r;
  _g = rhs._g;
  _b = rhs._b;
  _avg_r = rhs._avg_r;
  _avg_g = rhs._avg_g;
  _avg_b = rhs._avg_b;
  _total_pixel_count = rhs._total_pixel_count;

  return *this;
}

BareCell::~BareCell()
{
}

int BareCell::ParseMapFile(const std::string& filename, 
                           std::vector<BareCell>& vec,
                           double& x0,
                           double& y0,
                           double& z0)
{
  using namespace std;
  ifstream mapfile(filename.c_str());
  if (!mapfile)
  {
    cout << "Can't open the ifstream!" << endl;
    return -1;
  }

  int count = 0;
  int size;
  mapfile >> size >> x0 >> y0 >> z0;

  BareCell scratch;
  while (mapfile >> scratch)
  {
    vec.push_back(scratch);
    count++;
  }
  return count;
}

std::vector<int> BareCell::BoundingBox(const std::vector<BareCell>& vec)
{
  assert(vec.size() > 0);
  int minx, maxx, miny, maxy, minz, maxz;
  minx = maxx = vec.at(0).x();  // .at() to enforce >= 1 element.
  miny = maxy = vec.at(0).y();
  minz = maxz = vec.at(0).z();

  std::vector<int> res;

  for (size_t i = 0 ; i < vec.size() ; ++i)
  {
    minx = std::min(minx, vec[i].x());
    maxx = std::max(maxx, vec[i].x());
    miny = std::min(miny, vec[i].y());
    maxy = std::max(maxy, vec[i].y());
    minz = std::min(minz, vec[i].z());
    maxz = std::max(maxz, vec[i].z());
  }

  res.push_back(minx);
  res.push_back(maxx);
  res.push_back(miny);
  res.push_back(maxy);
  res.push_back(minz);
  res.push_back(maxz);
  return res;
}

//std::ostream& operator<<(std::ostream& out, const BareCell& rhs)
//{
//  out << rhs.x() << " " << rhs.y() << " " << rhs.z() << " "
//      << rhs.h() << " " << rhs.odo() << " " << rhs.pixel_count() 
//      << " " << rhs.r() << " " << rhs.g() << " " << rhs.b();
//  return out;
//}

std::istream& operator>>(std::istream& in, BareCell& rhs)
{
  int x, y, z, h;
  double odo;
  vector<double> pc;
  vector<double> r;
  vector<double> g;
  vector<double> b;
  in >> x >> y >> z >> h >> odo;
  for (int i = 0 ; i < 6 ; ++i)
  {
    int rr, gg, bb;
    double pcc;
    in >> rr >> gg >> bb >> pcc;
    r.push_back(rr);
    g.push_back(gg);
    b.push_back(bb);
    pc.push_back(pcc);
  }
  BareCell bc(x, y, z, h, odo, pc, r, g, b);
  std::swap(bc, rhs);
  return in;
}

} // namespace textured_localization
