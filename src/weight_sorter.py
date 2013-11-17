#!/usr/bin/env python

"""
weight_sorter.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

After printing out a set of particles, in the format

%s: %f

(where f may be a string, not necessarily a float), we want to sort them in
decreasing order. So, this does that.
"""

import sys

if len(sys.argv) != 2:
  print "Usage: bin/weight_sorter <file>"
  sys.exit(1)

IN = file(sys.argv[1], 'r');
vals = []
for line in IN:
  things = line.strip().split(':')
  if 'Particle' not in things[0]:
    continue
  try:
    val = float(things[1])
  except:
    val = 0.0
  vals.append((val, things[0]))

vals.sort()
vals.reverse()

for val in vals:
  print "%0.10f : %s" % val
