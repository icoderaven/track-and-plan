#!/usr/bin/env python
"""
decimate_trajectory.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

Given a trajectory, keep only every Nth step.
"""

import sys
import os

if len(sys.argv) != 3:
  print "Usage: bin/decimate_trajectory <input> <N>"
  sys.exit(1)

IN = file(sys.argv[1], 'r')
N = int(sys.argv[2])
OUT = file(os.path.splitext(sys.argv[1])[0] + ("_by%d.txt" % N), 'w')

lines = [line.strip() for line in IN]
for l in lines[::N]:
  OUT.write(l)
  OUT.write("\n")

print "Wrote", OUT.name
