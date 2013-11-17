/*
 * colorspaces.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Some colorspace conversion code, most of which is stolen from Alex Kuhl.
 */

#ifndef TEXTURED_LOCALIZATION_COLORSPACES_HH_INCLUDED
#define TEXTURED_LOCALIZATION_COLORSPACES_HH_INCLUDED 1

namespace textured_localization
{
  // (RGB) (in [0, 255]) to hsv (h in [0.0, 360.0); s&v in [0.0, 1.0]).
  void rgb_to_hsv(int r, int g, int b, double& h, double& s, double& v);
}

#endif
