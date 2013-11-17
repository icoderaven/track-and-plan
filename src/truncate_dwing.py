#!/usr/bin/env python
"""
truncate_dwing.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

Take a written-out DWING image and make it figureful, for the paper.
"""

import sys
from PIL import Image

if len(sys.argv) != 3:
  print "Usage: bin/truncate_dwing <input> <output>"
  sys.exit(1)

# Load it.
I = Image.open(sys.argv[1])

# Make sure the background stays transparent.
I = I.convert("RGBA")

# Rotate it.
IR = I.rotate(-26.75, Image.BICUBIC)

# A bevy of magic numbers, that I like.
IC = IR.crop((105, 300, 465, 790))

IC.save(sys.argv[2])
