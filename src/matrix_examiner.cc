/*
 * matrix_examiner.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * What does a matrix look like and do?
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include "TNT/tnt.h"

using namespace std;

int main(int argc, char* argv[])
{
  if (argc != 2)
  {
    cout << "Usage: bin/matrix_examiner <matrix>" << endl;
    return 1;
  }

  ifstream Min(argv[1]);
  if (!Min)
  {
    cout << "Matrix " << argv[1] << " cannot be opened!" << endl;
    return 1;
  }

  TNT::Array2D<double> M;
  Min >> M;
  Min.close();
  cout << "M:" << endl << M << endl;
  TNT::Array2D<double> O;
  TNT::Array2D<double> X;
  TNT::Array2D<double> Y;
  TNT::Array2D<double> Z;

  stringstream S;
  S << "4 1\n 0 0 0 1" << endl;  // O
  S << "4 1\n 1 0 0 1" << endl;  // X
  S << "4 1\n 0 1 0 1" << endl;  // Y
  S << "4 1\n 0 0 1 1" << endl;  // Z
  S >> O >> X >> Y >> Z;
  O = TNT::matmult(M, O);
  X = TNT::matmult(M, X);
  Y = TNT::matmult(M, Y);
  Z = TNT::matmult(M, Z);
  cout << "O:" << endl << O << endl;
  cout << "X:" << endl << X << endl;
  cout << "Y:" << endl << Y << endl;
  cout << "Z:" << endl << Z << endl;
  cout << "Sum: " << endl << X + Y + Z << endl;

}
