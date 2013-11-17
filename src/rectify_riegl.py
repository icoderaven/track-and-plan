#!/usr/bin/env python
"""
rectify_riegl.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

Take the lists of correspondences from the Riegl's TPLs, and construct the
homogeneous-coordinate transformation matrices that go from one to another.

Here's the math being performed. The two input files give us a list of
keypoints, some of which correspond. We drop them into Nx4
homogenous-coordinate matrices:

  | x1 y1 z1 1 |
  | x1 y1 z2 1 |
    ...etc...

letting A and B denote these matrices, we solve AR = B for R, the 4x4
homogeneous-coordinate transformation matrix, which we print the output.

Note that we provide the matrix turning the SECOND input into the FIRST input,
not the other way around. Luckily, matrix inversion is easy...
"""
import sys
import os
from numpy import *
import scipy.linalg

def parse_input(filename):
  """
  Turn a filename into a dictionary mapping the id (a string) to an [x, y, z]
  list (coordinates, in meters).
  """
  input = file(filename, 'r')
  res = {}
  for line in input:
    vals = line.strip().split()
    nums = map(float, vals[1:])
    res[vals[0]] = nums
  input.close()
  return res

def dict_to_matrix(dictionary, both):
  """
  Convert one of our dictionaries into a matrix. dictionary is the dictionary
  we're converting; both is the set of keys common to both input files.
  """
  res = zeros((len(both), 4))
  res[:, 3] = 1.0  # Homegeneous coordinates.
  for i, key in enumerate(both):
    res[i][0:3] = dictionary[key]
  return res

if __name__ == "__main__":
  if len(sys.argv) != 4:
    print "Usage: bin/rectify_riegl <1> <2> <output>"
    print "    The matrix that transforms 2 to 1 will be printed to output."
    print "    If output is '-', print to stdout."
    print "    The file format is uid x y z, space-delimited, one per line."
    sys.exit(1)

  # Handle parsing our input files.
  one = parse_input(sys.argv[1])
  two = parse_input(sys.argv[2])

  # We need to construct the points which appear in both one and two.
  both_keys = set(one.keys()).intersection(set(two.keys()))
  sys.stderr.write("There are %d correspondences.\n" % len(both_keys))
  sys.stderr.flush()

  # Now, build the A and B matrices, as described above.
  A = dict_to_matrix(one, both_keys)
  B = dict_to_matrix(two, both_keys)

  # Now, we actually do some mathematics.
  res = linalg.lstsq(A, B)[0].T  # [0] for the matrix; don't need the rest.

  # Now, let's figure out where that sent our various points.
  transformed = dot(A, res)
  print "Errors:"
  print transformed - B
  # Figure out where we're sending the output.
  if sys.argv[3] == '-':
    output = sys.stdout
  else:
    output = file(sys.argv[3], 'w')

  # Finally, print it out in TNT format, for the benefit of riegl_many_to_map.
  output.write("%d %d\n" % res.shape)
  for i in range(res.shape[0]):
    for j in range(res.shape[1]):
      output.write("%f " % res[i, j])
    output.write("\n")
  output.close()
