/*
 * bit_variance.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * We're planning on taking HDR images for our robot runs, but this means we
 * need to know how much bitwise overlap we have; this computes that,
 * bit-by-bit, for a set of images.
 */ 

#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "bayer.hh"
#include "linear.hh"

using namespace std;
namespace bfs = boost::filesystem;
using boost::format;

/*
 * Our 1600x1200 raw images have this size, but sometimes the camera drops
 * part of a frame; this lets us detect that.
 */
const unsigned int RAW_SIZE = 2400000;

// Mean & Standard Deviation.
pair<double, double> stats(vector<int> values)
{
  double total = 0;
  foreach(int i, values)
    total += i;

  double mean = total / values.size();

  double centered_total = 0;
  foreach(int i, values)
  {
    centered_total += pow(i - mean, 2.0);
  }

  return make_pair(mean, sqrt(centered_total / values.size()));
}

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    cout << "Usage: bin/bit_variance <dir> <basename> [start] [end]" << endl;
    return 1;
  }
  int start = 0;
  if (argc >= 4)
    start = atoi(argv[3]);
  int end = 20;
  if (argc >= 5)
    end = atoi(argv[4]);

  // Assemble our working directory.
  bfs::path dirname(argv[1]);
  if (dirname.filename() == ".")  // Trailing '/'; get rid of it.
    dirname.remove_filename();

  // Assemble the names of the files we're using.
  vector<bfs::path> filenames;
  for (int i = start ; i < end ; ++i)
  {
    bfs::path filename(str(format("%s%d.jpg") % argv[2] % i));
    filenames.push_back(dirname / filename);
  }
  
  // Make sure these images are really available.
  foreach (bfs::path s, filenames)
  {
    if (!bfs::exists(s))
    {
      cout << "File " << s << " does not exist!" << endl;
      return 1;
    }
  }

  // Load the images. Also, make sure the size is correct.
  vector<IplImage*> images;
  bool skipped = false;
  foreach(bfs::path s, filenames)
  {
//    if (bfs::file_size(s) < RAW_SIZE)
//    {
//      skipped = true;
//      cout << "*" << flush;
//      continue;
//    }
//    images.push_back(bayer::pipeline(s.string(), NULL));
    images.push_back(cvLoadImage(s.string().c_str(), CV_LOAD_IMAGE_GRAYSCALE));
  }
  if (skipped)
    cout << endl;
  cout << "Loaded " << images.size() << " images." << endl;

  // Build the gigantic array.
  typedef vector< vector< vector< int > > > Cube;
  Cube cube;
  for (int i = 0 ; i < 1600 ; ++i)
  {
    vector< vector< int > > temp;
    for (int j = 0 ; j < 1200 ; ++j)
    {
      temp.push_back(vector<int>());
    }
    cube.push_back(temp);
  }

  // Fill in the gigantic array.
  foreach(IplImage* image, images)
  {
    cout << "." << flush;
    for (int i = 0 ; i < 1600 ; ++i)
    {
      for (int j = 0 ; j < 1200 ; ++j)
      {
        cube[i][j].push_back(cvGet2D(image, j, i).val[0]);
      }
    }
  }
  cout << endl;

  // Generate our output images
  IplImage* mean_out = cvCreateImage(cvSize(1600, 1200), IPL_DEPTH_8U, 1);
  IplImage* stddev_out = cvCreateImage(cvSize(1600, 1200), IPL_DEPTH_8U, 1);
  IplImage* snr_out = cvCreateImage(cvSize(1600, 1200), IPL_DEPTH_8U, 1);

  double stddev_min = 256;
  double stddev_max = 0;
  for (int i = 0 ; i < 1600 ; ++i)
  {
    for (int j = 0 ; j < 1200 ; ++j)
    {
      pair<double, double> S = stats(cube[i][j]);
      cvSet2D(mean_out, j, i, cvScalarAll(S.first));
      cvSet2D(stddev_out, j, i, cvScalarAll(S.second));
      if (S.second == 0)
        cvSet2D(snr_out, j, i, cvScalarAll(255));
      else
        cvSet2D(snr_out, j, i, cvScalarAll(S.first / S.second));
      stddev_min = min(stddev_min, S.second);
      stddev_max = max(stddev_min, S.second);
    }
  }

  cout << format("stddev range: [%d, %d]") % stddev_min % stddev_max << endl;

  cvConvertScale(stddev_out, stddev_out, 255.0 / stddev_max, -stddev_min);
  cvSaveImage(str(format("%s_mean.png") % argv[2]).c_str(), mean_out);
  cvSaveImage(str(format("%s_stddev.png") % argv[2]).c_str(), stddev_out);
  cvSaveImage(str(format("%s_snr.png") % argv[2]).c_str(), snr_out);

  // Generate a colorized version of our mean image.
  IplImage* bayer_mean_out = bayer::OpenCVInterpolation()(mean_out);
//  cvSaveImage(str(format("%s_color_mean.png") % argv[2]).c_str(), 
//              bayer_mean_out);

  // Clean up after ourselves.
  cvReleaseImage(&mean_out);
  cvReleaseImage(&stddev_out);
  cvReleaseImage(&snr_out);
  cvReleaseImage(&bayer_mean_out);

  foreach(IplImage* image, images)
    cvReleaseImage(&image);
}
