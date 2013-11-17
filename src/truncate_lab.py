#!/usr/bin/env python
"""
truncate_dwing.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

Take a written-out LAB image and make it figureful, for the paper.
"""

import sys
import os
from PIL import Image

if len(sys.argv) != 3:
  print "Usage: bin/truncate_lab <input> <output>"
  sys.exit(1)

# Load it.
I = Image.open(sys.argv[1])

# Make sure the background stays transparent.
I = I.convert("RGBA")

# Rotate it.
IR = I.rotate(-68.5, Image.BICUBIC, expand = True)

# A bevy of magic numbers, that I like.
IC = IR.crop((80, 100, 400, 360))

IC.save(sys.argv[2])

# Now, do something evil.
os.system("mogrify -background white -layers flatten %s" % sys.argv[2])
