/*
 * linear.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Linearize (and de-linearize) colors in the sRGB colorspace. 
 * See <http://en.wikipedia.org/wiki/SRGB> for details, the values of the
 * constants, and so on.
 *
 * Stuff is stored in the range [0.0, 1.0].
 */

#ifndef LINEAR_HH_INCLUDED
#define LINEAR_HH_INCLUDED 1

#include "cxcore.h"

#define SRGB_ALPHA 0.055
#define SRGB_LINEAR_TO_SRGB_THRESHOLD 0.0031308
#define SRGB_SRGB_TO_LINEAR_THRESHOLD 0.04045

/*
 * Do just one element (R, G, or B).
 */
double one_srgb_to_linear(double c);
double one_linear_to_srgb(double c);


/*
 * Do an RGB triple, all at once. Doesn't change the alpha-value.
 */
CvScalar srgb_to_linear(const CvScalar& c);
CvScalar linear_to_srgb(const CvScalar& c);

/*
 * Since OpenCV is going to be using [0, 255], it's handy to be able to
 * convert back and forth. Note that these work in-place.
 */
void to_01(CvScalar& c);
void to_0255(CvScalar& c);

/*
 * Do an entire image, in-place. The passed-in images are going to be [0, 255]
 * images, hence the need for the conversion functions above. 
 */
void image_srgb_to_linear(IplImage* input);
void image_linear_to_srgb(IplImage* input);

/*
 * Fake exposure compensation; linearize, scale by factor, de-linearize.
 */
void exposure_compensation(IplImage* im, double factor);

#endif
