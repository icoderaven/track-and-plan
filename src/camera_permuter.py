#!/usr/bin/env python
"""
camera_permuter.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

If we plugged the cameras in in the wrong order, this will rename the files to
sort that out.
"""

import sys
import os
import shutil

# The permutation. This maps current numbers to correct numbers.
P = {3 : 0,
     4 : 1, 
     5 : 2,
     2 : 3,
     1 : 4,
     0 : 5}

for i in range(5):
  assert i in P.keys()
  assert i in P.values()

if len(sys.argv) != 2:
  print "Usage: bin/camera_permuter <logdir>"
  sys.exit(1)

BASE = sys.argv[1] + "/"
logfile = file(BASE + os.path.basename(sys.argv[1]) + ".log", 'r')
step = 0
for line in logfile:
  S = line.strip().split()
  if S[0] != "Camera:":
    continue
  filenames = S[2:]
  for (ci, fn) in enumerate(filenames):
    #print "mv %s temp%d.jpg" % (fn, ci)
    shutil.move(BASE + fn, BASE + "temp%d.jpg" % ci)
  for (ci, fn) in enumerate(filenames):
    #print "mv temp%d.jpg step%04d-cam%d.jpg" % (ci, step, P[ci])
    shutil.move(BASE + "temp%d.jpg" % ci, 
                BASE + "step%04d-cam%d.jpg" %(step, P[ci]))
  step += 1
