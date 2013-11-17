#!/usr/bin/env python
"""
naive_color_calibrator.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

A REALLY naive color-calibration algorithm; given two images (ref and render),
compute the 4x4 transformation matrix that best converts the colors of ref
into those of render.

This uses every pixel, which makes it not-very-smart-at-all.
"""

import sys
from PIL import Image
from numpy import *
import scipy.linalg

if len(sys.argv) < 3:
  print "Usage: bin/naive_color_calibrator <ref> <render> [output]"
  print "    If you specify [output], we write the converted <ref> to"
  print "    that file; either way, we print the conversion matrix."
  sys.exit(1)

ref_im = Image.open(sys.argv[1])
ren_im = Image.open(sys.argv[2])

ref_im = ref_im.resize(ren_im.size, Image.ANTIALIAS)

SHAPE = asarray(ref_im).shape

# Make stacked vectors.
ref_mat  = asarray(ref_im).reshape((-1, 3))
ren_mat  = asarray(ren_im).reshape((-1, 3))

# Put ourselves into homogeneous coordinates.
ref_mat = hstack((ref_mat, ones((ref_mat.shape[0], 1))))
ren_mat = hstack((ren_mat, ones((ren_mat.shape[0], 1))))

# Solve it.
T = scipy.linalg.lstsq(ref_mat, ren_mat)[0]

if len(sys.argv) == 4:
  scipy.misc.imsave(sys.argv[3], dot(ref_mat, T)[:, :3].reshape(SHAPE))

print "4 4"
for i in range(4):
  for j in range(4):
    print T[i, j],
  print ""
