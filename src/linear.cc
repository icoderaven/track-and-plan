/*
 * linear.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See linear.hh.
 */

#include <iostream>
#include <cmath>
#include "linear.hh"

double one_srgb_to_linear(double c)
{
  if (c <= SRGB_SRGB_TO_LINEAR_THRESHOLD)
  {
    return c / 12.92;
  }
  else
  {
    return std::pow((c + SRGB_ALPHA)/(1 + SRGB_ALPHA), 2.4);
  }
}

double one_linear_to_srgb(double c)
{
  if (c <= SRGB_LINEAR_TO_SRGB_THRESHOLD)
  {
    return 12.92 * c;
  }
  else
  {
    return ((1.0 + SRGB_ALPHA) * std::pow(c, 1/2.4)) - SRGB_ALPHA;
  }
}

CvScalar srgb_to_linear(const CvScalar& c)
{
  return cvScalar(one_srgb_to_linear(c.val[0]),
                  one_srgb_to_linear(c.val[1]),
                  one_srgb_to_linear(c.val[2]),
                  c.val[3]);
}

CvScalar linear_to_srgb(const CvScalar& c)
{
  return cvScalar(one_linear_to_srgb(c.val[0]),
                  one_linear_to_srgb(c.val[1]),
                  one_linear_to_srgb(c.val[2]),
                  c.val[3]);
}

void to_01(CvScalar& c)
{
  for (int i = 0 ; i < 4 ; ++i)
    c.val[i] /= 255.0;
}

void to_0255(CvScalar& c)
{
  for (int i = 0 ; i < 4 ; ++i)
    c.val[i] *= 255.0;
}

void image_srgb_to_linear(IplImage* input)
{
  for (int x = 0 ; x < input->width ; ++x)
  {
    for (int y = 0 ; y < input->height ; ++y)
    {
      CvScalar s = cvGet2D(input, y, x);
      to_01(s);
      CvScalar s2 = srgb_to_linear(s);
      to_0255(s2);
      cvSet2D(input, y, x, s2);
    }
  }
}

void image_linear_to_srgb(IplImage* input)
{
  for (int x = 0 ; x < input->width ; ++x)
  {
    for (int y = 0 ; y < input->height ; ++y)
    {
      CvScalar s = cvGet2D(input, y, x);
      to_01(s);
      CvScalar s2 = linear_to_srgb(s);
      to_0255(s2);
      cvSet2D(input, y, x, s2);
    }
  }
}

void exposure_compensation(IplImage* im, double factor)
{
  image_srgb_to_linear(im);
  for (int x = 0 ; x < im->width ; ++x)
  {
    for (int y = 0 ; y < im->height ; ++y)
    {
      CvScalar s = cvGet2D(im, y, x);
      for (int i = 0 ; i < 4 ; ++i)
        s.val[i] *= factor;
      cvSet2D(im, y, x, s);
    }
  }
  image_linear_to_srgb(im);
}
