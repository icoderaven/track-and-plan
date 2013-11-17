/*
 * extended_tnt.h
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * TNT is lacking some stuff. I'm adding it.
 */

#ifndef TNT_EXTENDED_TNT_INCLUDED 
#define TNT_EXTENDED_TNT_INCLUDED 1

#include <cassert>
#include <cmath>
#include "TNT/tnt.h"
#include "TNT/jama_lu.h"

namespace TNT
{
  // Construct the size-by-size identity matrix.
  template<typename T>
  Array2D<T> ident(size_t size)
  {
    Array2D<double> res(size, size, (T)0.0);
    for (size_t i = 0 ; i < size ; ++i)
      res[i][i] = (T)1.0;

    return res;
  }

  // Invert a matrix.
  template<class T>
  Array2D<T> invert(const Array2D<T> &M)
  {
    assert(M.dim1() == M.dim2()); // square matrices only please

    // solve for inverse with LU decomposition
    JAMA::LU<T> lu(M);

    Array2D<T> id(M.dim1(), M.dim1(), (T)0);
    for (int i = 0; i < M.dim1(); i++) 
      id[i][i] = 1;

    // solves A * A_inv = Identity
    return lu.solve(id);
  }

  // Transpose a matrix.
  template<class T>
  Array2D<T> transpose(const Array2D<T> &M)
  {
    Array2D<T> tran(M.dim2(), M.dim1());
    for(int r=0; r<M.dim1(); ++r)
      for(int c=0; c<M.dim2(); ++c)
        tran[c][r] = M[r][c];
    return tran;
  }

  // Construct the "RotX" matrix (rotate around the X-axis, moving y towards
  // z). Note that this returns a 4x4 homogenous-coordinates matrix.
  //
  // angle is in radians.
  template <typename T>
  Array2D<T> RotX(double angle)
  {
    using namespace std;
    Array2D<double> res(4, 4, (T)0.0);
    // First row
    res[0][0] = (T)1.0; 
    res[0][1] = (T)0.0; 
    res[0][2] = (T)0.0; 
    res[0][3] = (T)0.0;

    // Second row
    res[1][0] = (T)0.0; 
    res[1][1] = (T)cos(angle);
    res[1][2] = (T)-sin(angle);
    res[1][3] = (T)0.0;

    // Third row
    res[2][0] = (T)0;
    res[2][1] = (T)sin(angle);
    res[2][2] = (T)cos(angle);
    res[2][3] = (T)0;

    // Last row
    res[3][0] = res[3][1] = res[3][2] = (T)0.0;
    res[3][3] = (T)1.0;

    return res;
  }

  // Same as above, but around y, moving z towards x.
  template <typename T>
  Array2D<T> RotY(double angle)
  {
    using namespace std;
    Array2D<double> res(4, 4, (T)0.0);
    // First row
    res[0][0] = (T)cos(angle);
    res[0][1] = (T)0.0; 
    res[0][2] = (T)sin(angle);
    res[0][3] = (T)0.0;

    // Second row
    res[1][0] = (T)0.0; 
    res[1][1] = (T)1.0;
    res[1][2] = (T)0.0;
    res[1][3] = (T)0.0;

    // Third row
    res[2][0] = (T)-sin(angle);
    res[2][1] = (T)0;
    res[2][2] = (T)cos(angle);
    res[2][3] = (T)0;

    // Last row
    res[3][0] = res[3][1] = res[3][2] = (T)0.0;
    res[3][3] = (T)1.0;

    return res;
  }

  // Finally, around z, moving x towards y.
  template <typename T>
  Array2D<T> RotZ(double angle)
  {
    using namespace std;
    Array2D<T> res(4, 4, (T)0.0);
    // First row
    res[0][0] = (T)cos(angle);
    res[0][1] = (T)-sin(angle);
    res[0][2] = (T)0.0;
    res[0][3] = (T)0.0;

    // Second row
    res[1][0] = (T)sin(angle);
    res[1][1] = (T)cos(angle);
    res[1][2] = (T)0.0;
    res[1][3] = (T)0.0;

    // Third row
    res[2][0] = (T)0.0;
    res[2][1] = (T)0.0;
    res[2][2] = (T)1.0;
    res[2][3] = (T)0.0;

    // Last row
    res[3][0] = res[3][1] = res[3][2] = (T)0.0;
    res[3][3] = (T)1.0;

    return res;
  }

  // Some useful values.
  template <typename T>
  Array2D<T> Origin()
  {
    Array2D<T> res(4, 1, (T)0.0);
    res[3][0] = (T)1.0;
    return res;
  }

  template <typename T>
  Array2D<T> PlusX()
  {
    Array2D<T> res = Origin<T>();
    res[0][0] = (T)1.0;
    return res;
  }

  template <typename T>
  Array2D<T> PlusY()
  {
    Array2D<T> res = Origin<T>();
    res[1][0] = (T)1.0;
    return res;
  }

  template <typename T>
  Array2D<T> PlusZ()
  {
    Array2D<T> res = Origin<T>();
    res[2][0] = (T)1.0;
    return res;
  }

  // Return A, but with norm 1. Assumes A is a column vector!
  template<class T>
  Array2D<T> normalize(Array2D<T> A)
  {
    assert (A.dim2() == 1);
    Array2D<T> A2 = transpose(A);
    assert (A2.dim1() == 1);
    Array2D<T> A3 = matmult(A2, A);
    assert (A3.dim1() == 1);
    assert (A3.dim2() == 1);

    Array2D<T> div = Array2D<T>(A.dim1(), 1, sqrt(A3[0][0]));
    return A / div;
  }
}

#endif
