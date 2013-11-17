/*
 * bayer.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Deal with the (goofy, 10-bit) bayer pattern images from our Webcam Pro
 * 9000s.
 */

#ifndef BAYER_HH_INCLUDED
#define BAYER_HH_INCLUDED 1

#include <string>
#include <cv.h>

namespace bayer
{
  /* Convenient bitmasks. */
  const int BOTTOM_TWO = 3;
  const int L_MIDDLE_TWO = 3 << 2;
  const int U_MIDDLE_TWO = 3 << 4;
  const int TOP_TWO = 3 << 6;
  const int TOPPEST_TWO = 3 << 8;
  const int ONLY_TEN = (BOTTOM_TWO | L_MIDDLE_TWO | 
                        U_MIDDLE_TWO | TOP_TWO | TOPPEST_TWO);

  /*
   * All the different ways OpenCV can do Bayer conversion for us. 
   * The correct index seems to be 0.
   * Many of these things are the same; that's odd, but ok.
   */
  const int BAYER_TYPES[] = 
  {
    CV_BayerBG2BGR, 
    CV_BayerGB2BGR, 
    CV_BayerRG2BGR, 
    CV_BayerGR2BGR,
    CV_BayerBG2RGB, 
    CV_BayerRG2BGR, 
    CV_BayerGB2RGB, 
    CV_BayerGR2BGR,
    CV_BayerRG2RGB, 
    CV_BayerBG2BGR, 
    CV_BayerGR2RGB, 
    CV_BayerGB2BGR
  };
  const int NUM_BAYER_TYPES = 12;

  /*
   * Given five bytes (in five), parse out four values, and put them in out
   * (which must be big enough to hold four values).
   */
  void four_values(unsigned char five[], int out[]);

  /*
   * Parse out an entire file, putting the results into values, written out
   * flat. If ten_bits, assume a 10-bit image; otherwise, an 8-bit.
   */
  void parse(const std::string& filename, int values[], bool ten_bits=true);

  /*
   * Write the data into an image, with no interpolation. The values array is
   * the output parameter of parse, and im needs to be a single-channel image
   * that's large enough to handle it.
   */
  void raw_image(int values[], IplImage* im);

  /*
   * Everything above here deals with reading Logitech's 10-bit raw images.
   * Now, we need to deal with actually doing Bayer Interpolation. First, an
   * interface.
   */
  class InterpolationInterface
  {
    public:
      /*
       * Only required function; do Bayer Interpolation. Allocates, and
       * returns, an image of the same size (but three channels) as the input.
       */
      virtual IplImage* operator()(IplImage* im) = 0;
  };

  /*
   * OpenCV can do this for us!
   */
  class OpenCVInterpolation : public InterpolationInterface
  {
    public:
      OpenCVInterpolation(int bayer_type = 0);
      IplImage* operator()(IplImage* im);
    private:
      int _bayer_type;
  };

  /*
   * Two functions that wrap everything else. The first reads in a file, and
   * undoes Logitech's encoding; you get back an IplImage* which you must
   * personally release. If ii is non-NULL, you get back an interpolated
   * image, according to the interpolation strategy you provide.  
   *
   * The second has the same behavior, but writes out a file for you, rather
   * than returning it.
   */
  IplImage* pipeline(const std::string& input_filename,
                     InterpolationInterface* ii = NULL,
                     int width=1600,
                     int height=1200);
       
  void pipeline(const std::string& input_filename,
                const std::string& output_filename,
                InterpolationInterface* ii = NULL,
                int width=1600,
                int height=1200);
}

#endif 
