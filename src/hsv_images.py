#!/usr/bin/env python
"""
hsv_images.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

Turn an RGB image into three different HSV images.
"""

import sys
from PIL import Image
import colorsys

if len(sys.argv) != 3:
  print "Usage: bin/saturation_image <in> <out_basename>"
  sys.exit(1)

I = Image.open(sys.argv[1])

def convert((r, g, b), idx):
  hsv = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
  return (int(255 * hsv[idx]),) * 3

def h_convert((r, g, b)):
  hsv = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
  rgb2 = map(lambda x: int(x*255.0), colorsys.hsv_to_rgb(hsv[0], 1.0, 1.0))
  return tuple(rgb2)

def hs_convert((r, g, b)):
  hsv = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
  rgb2 = colorsys.hsv_to_rgb(hsv[0], 1.0, hsv[0] * hsv[1])
  rgb2 = map(lambda x: int(x * 255.0), rgb2)
  return tuple(rgb2)


h = [h_convert(x) for x in I.getdata()]
hs = [hs_convert(x) for x in I.getdata()]
s = [convert(x, 1) for x in I.getdata()]
v = [convert(x, 2) for x in I.getdata()]
I.putdata(h)
I.save(sys.argv[2] + "_h.png")
I.putdata(s)
I.save(sys.argv[2] + "_s.png")
I.putdata(v)
I.save(sys.argv[2] + "_v.png")
I.putdata(hs)
I.save(sys.argv[2] + "_hs.png")

