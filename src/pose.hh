/*
 * pose.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Represent a pose; how about that? Orientation is in radians.
 */

#ifndef TEXTURED_LOCALIZATION_POSE_HH_INCLUDED
#define TEXTURED_LOCALIZATION_POSE_HH_INCLUDED 1

#include <iostream>
#include <vector>
namespace textured_localization
{
  class Pose
  {
    public:
      Pose();  // All zeros
      Pose(double x, double y, double z, double theta);  // 3D-ish pose.
      Pose(const Pose& rhs);
      Pose& operator=(const Pose& rhs);
      ~Pose();
      double x() const;
      double y() const;
      double z() const { return _z; };
      double theta() const;

      // Setters.
      void set_z(double z);
      void set_theta(double t);
      // Set all the parameters at once.
      void set(double x, double y, double z, double theta);

    private:
      double _x;
      double _y;
      double _z;
      double _theta;

    public:
      // Helper for drawing. Returns [xmin xmax ymin ymax].
      static std::vector<double> BoundingBox(const std::vector<Pose>& poses);
  };

  /*
   * Operator+ is smart; lhs is a pose, and rhs is an action, and it gets the
   * trigonometry right. Operator- is just a straight element-by-element
   * subtraction.
   */
  Pose operator+(const Pose& lhs, const Pose& rhs);
  Pose operator-(const Pose& lhs, const Pose& rhs);

  std::ostream& operator<<(std::ostream& out, const Pose& rhs);
  std::istream& operator>>(std::istream& in, Pose& rhs);
}

#endif
