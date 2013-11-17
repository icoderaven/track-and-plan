#!/usr/bin/env python
"""
magnitude.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

Compute the magnitude of a grayscale image.
"""

import sys
from PIL import Image

I = Image.open(sys.argv[1])

print "L_1 mag is", sum(I.getdata())
print "L_2 mag is", sum(map(lambda x: x*x, I.getdata()))



