/*
 * barecell.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Just a 1x1x1 cube, with an occupancy and a color. Nothing fancy.
 *
 * This also includes some code for manipulating vectors of barecells, as that
 * tends to be quite handy.
 */

#ifndef TEXTURED_LOCALIZATION_BARECELL_HH_INCLUDED
#define TEXTURED_LOCALIZATION_BARECELL_HH_INCLUDED 1

#include <iostream>
#include <string>
#include <vector>

using namespace std;

namespace textured_localization
{
  class BareCell
  {
    public:
      /*
       * Build one at (x, y, z) with hitcount h, odometer odo, and so on. 
       *
       * The passed-in values are the accumulated values, not the mean; this
       * does that step for you.
       */
      BareCell();  // All zeros; doesn't make physical sense.
      BareCell(int x, int y, int z, 
               int h, double odo, 
               vector<double> pixel_count, 
               vector<double> r, 
               vector<double> g, 
               vector<double> b);
      BareCell(const BareCell& rhs);
      BareCell& operator=(const BareCell& rhs);
      ~BareCell();

      /*
       * Getters.
       */
      int x() const { return _x; };
      int y() const { return _y; };
      int z() const { return _z; };
      int h() const { return _h; };
      double odo() const { return _odo; };
      double pixel_count(size_t i) const { return _pixel_count[i]; };
      double r(size_t i) const { return _r[i]; };
      double g(size_t i) const { return _g[i]; };
      double b(size_t i) const {return _b[i]; };

      double r() const { return _avg_r; }
      double g() const { return _avg_g; }
      double b() const { return _avg_b; }
      void set_r(int index, double r) { _r[index] = r; }
      void set_g(int index, double g) { _g[index] = g; }
      void set_b(int index, double b) { _b[index] = b; }
      double pixel_count() const { return _total_pixel_count; }
      
      // Because it needs to be able to mutate instances.
      friend std::istream& operator>>(std::istream& in, BareCell& rhs);

    private:
      int _x;
      int _y;
      int _z;
      int _h;
      double _odo;
      vector<double> _pixel_count;
      vector<double> _r;
      vector<double> _g;
      vector<double> _b;

      double _avg_r;
      double _avg_g;
      double _avg_b;
      double _total_pixel_count;

    public:
      /*
       * Parsing a file is a pain in the neck; this does it for us. The second
       * argument is an output argument; any parsed cells are appended. If
       * there are already cells in that vector, nothing bad happens.
       *
       * The return value is the number of Cells parsed, or -1 if something
       * went haywire. The three doubles are set to x0, y0, z0.
       */
      static int ParseMapFile(const std::string& filename, 
                              std::vector<BareCell>& vec,
                              double& x0,
                              double& y0,
                              double& z0);

      /*
       * Wouldn't it be nice to know the bounding box of a map? 
       * The returned vector is [minx, maxx, miny, maxy, minz, maxz].
       */
      static std::vector<int> BoundingBox(const std::vector<BareCell>& vec);

  };
  std::ostream& operator<<(std::ostream& out, const BareCell& rhs);
  std::istream& operator>>(std::istream& in, BareCell& rhs);
}

#endif
