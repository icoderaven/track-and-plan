/*
 * log_to_movie.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Convert one of our image-equipped logfiles into a movie, for ease of
 * viewing. Will, eventually, support camera transformations, etc.
 */

#include <cstdio>
#include <iostream>
#include <string>
#include <deque>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace std;
namespace bfs = boost::filesystem;

/* 
 * The images we take are 1600x1200; that's too big. This is how big they
 * are when we put them in the movie.
 */
const int width = 1600 / 6;
const int height = 1200 / 6;

// Read in a distortion map from the given file. The returned pointer is
// allocated interally, and must be dealt with by the caller.

int main(int argc, char* argv[])
{
  if (argc != 3)
  {
    cout << "Usage: ./log_to_movie <directory of log> <output file>" << endl;
    return 1;
  }

  // Extract the log name itself. 
  bfs::path dirname(argv[1]);
  if (dirname.filename() == ".")  // Trailing '/'; get rid of it.
    dirname.remove_filename();
  bfs::path logname = dirname.filename().string() + ".log";
  bfs::path fullname = dirname / logname;

  // Verify that the logfile exists.
  bfs::ifstream log(fullname);
  if (!log)
  {
    cout << "Couldn't open logfile \"" << fullname << "\"!" << endl;
    return 1;
  }

  // Video-file creator. On OS X, this (weirdly) needs the full path to the
  // output file; relative paths won't do. This leads to some minor goofiness.
  CvVideoWriter* writer = 
    cvCreateVideoWriter((bfs::current_path() / argv[2]).string().c_str(),
                        -1,//CV_FOURCC('M', 'J', 'P', 'G'),
                        2,
                        cvSize(width * 6, height));  // 6 images => * 6

  // Process the logfile
  string line;
  int count = 0;
  while (getline(log, line))
  {
    boost::trim(line);
    // We only care about Camera: lines
    if (line[0] != 'C')
      continue;

    cout << "." << flush;

    // Parse out the image names.
    deque<string> image_names;
    boost::split(image_names, line, boost::is_any_of(" "));
    image_names.pop_front();  // Camera: 
    image_names.pop_front();  // '6', usually

    // To ease checking our access to the images.
    deque<bfs::path> paths;
    foreach(string i, image_names)
      paths.push_back(dirname / i);

    // Make sure we can read each image.
    foreach(bfs::path p, paths)
    {
      bfs::ifstream im(p);
      if (!im)
      {
        cout << "Image " << p.filename()
             << " is listed, but " 
             << p << " cannot be opened!"
             << endl;
        return 1;
      }
    }

    /*
     * Moment of truth time. We need to read in the six images specified on
     * this line, resize them, stitch them together, and add the resulting
     * image to the videowriter.
     *
     * Load the images:
     */
    deque<IplImage*> images;
    foreach(bfs::path p, paths)
    {
      IplImage* big = cvLoadImage(p.string().c_str());
      IplImage* small = cvCreateImage(cvSize(width, height),
                                      big->depth, 
                                      big->nChannels);
      cvResize(big, small);
      cvReleaseImage(&big);
      images.push_back(small);
    }

    // Create the blank; inherit some parameters from a read-in image.
    IplImage* stitched = cvCreateImage(cvSize(width * 6, height),
                                       images[0]->depth,
                                       images[0]->nChannels);
    cvSet(stitched, cvScalar(0, 0, 0));
    // For setting ROI
    int x_offset = 5 * width;

    // Stitch them together
    foreach(IplImage* im, images)
    {
      cvSetImageROI(stitched, cvRect(x_offset, 0, width, height));
      cvSetImageROI(im, cvRect(0, 0, width, height));
      cvAdd(im, stitched, stitched);
      x_offset -= width;

      // We're done with this image.
      cvReleaseImage(&im);
    }
    cvResetImageROI(stitched);
    cvWriteFrame(writer, stitched);
    cvReleaseImage(&stitched);

    count++;
  }

  // Don't forget to clean up.
  cvReleaseVideoWriter(&writer);
}
