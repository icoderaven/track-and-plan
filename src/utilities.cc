/*
 * utilities.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See utilities.hh.
 */

#include <cmath>
#include "TNT/tnt.h"
#include "TNT/jama_lu.h"
#include "utilities.hh"

namespace textured_localization
{

double radians(double degrees)
{
  return normalize_angle(degrees * (M_PI / 180.0));
}

double degrees(double radians)
{
  return radians * (180.0 / M_PI);
}

double normalize_angle(double angle)
{
  double res = angle;
  while (res >= 2 * M_PI)
    res -= 2 * M_PI;
  while (res < -M_PI)
    res += 2 * M_PI;

  return res;
}

TNT::Array2D<double> inverse(const TNT::Array2D<double>& A)
{
  // Make sure it's square.
  assert(A.dim1() == A.dim2());
  JAMA::LU<double> lu(A);

  // create identity matrix
  TNT::Array2D<double> id(A.dim1(), A.dim2(), 0.0);
  for (int i = 0; i < A.dim1(); i++) id[i][i] = 1.0;

  // solves A * A_inv = Identity
  return lu.solve(id);
}

double maximum_element(IplImage* arr)
{
  double res = 0.0;
  for (int r = 0 ; r < arr->height ; ++r)
  {
    for (int c = 0 ; c < arr->width ; ++c)
    {
      CvScalar s = cvGet2D(arr, r, c);
      if (res < s.val[0])
        res = s.val[0];
    }
  }
  return res;
}

double minimum_element(IplImage* arr)
{
  double res = 10e10;
  for (int r = 0 ; r < arr->height ; ++r)
  {
    for (int c = 0 ; c < arr->width ; ++c)
    {
      CvScalar s = cvGet2D(arr, r, c);
      if (res > s.val[0])
        res = s.val[0];
    }
  }
  return res;
}

TNT::Array2D<double> image_to_array(IplImage* arr)
{
  TNT::Array2D<double> res(arr->height, arr->width, 0.0);
  for (int r = 0 ; r < arr->height ; ++r)
    for (int c = 0 ; c < arr->width ; ++c)
      res[r][c] = cvGet2D(arr, r, c).val[0];

  return res;
}

Pose VectorsToPose(const TNT::Array2D<double>& pose,
                   const TNT::Array2D<double>& facing)
{
  // x y z are easy.
  Array2D<double> fvec = facing - pose;
  double theta = atan2(fvec[1][0], fvec[0][0]);
  return Pose(pose[0][0], pose[1][0], pose[2][0], theta);
}

}
