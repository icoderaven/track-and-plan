/*
 * sorted_viewer.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Show a bunch of images, sorted according to their weights.
 */

#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <cv.h>
#include <highgui.h>

#include "utilities.hh"

using namespace std;
using namespace textured_localization;
using boost::format;

/* Yeah, it's not really pair. Shaddup. */
class MyPair
{
  public:
    string filename;
    double prob;
    IplImage* img;
    MyPair(string filename_, double prob_)
      : filename(filename_), prob(prob_), img(NULL)
    {
    }
    ~MyPair()
    {
      if (img != NULL)
        cvReleaseImage(&img);
    }
};

ostream& operator<<(ostream& out, const MyPair& rhs)
{
  out << rhs.prob << " : " << rhs.filename;
  return out;
}

bool operator<(const MyPair& lhs, const MyPair& rhs)
{
  return lhs.prob < rhs.prob;
}

bool operator==(const MyPair& lhs, const MyPair& rhs)
{
  return lhs.prob == rhs.prob;
}

// From l2shower.cc
double norm(CvScalar a, CvScalar b)
{
  double total = 0;
  for (int i = 0;  i < 3 ; ++i)
    total += pow(a.val[i] - b.val[i], 2);
  return sqrt(total);
}

// Apply the norm function, above, to a whole image.
IplImage* l2_image(IplImage* one, IplImage* two)
{
  if (one == NULL || two == NULL)
  {
    cout << "RETURNING NULL" << endl;
    return NULL;
  }
  IplImage* res = cvCreateImage(cvSize(one->width, one->height), 
                                IPL_DEPTH_64F, 1);

  for (int i = 0 ; i < res->height ; ++i)
  {
    for (int j = 0 ; j < res->width ; ++j)
    {
      double n = norm(cvGet2D(one, i, j), cvGet2D(two, i, j));
      cvSet2D(res, i, j, cvScalarAll(n));
    }
  }

  double min = minimum_element(res);
  double max = maximum_element(res);
  cout << "min is " << min << " max is " << max << endl;

  for (int i = 0 ; i < res->height ; ++i)
  {
    for (int j = 0 ; j < res->width ; ++j)
    {
      CvScalar v = cvGet2D(res, i, j);
      CvScalar n = cvScalarAll(((v.val[0] - min) / max));// * 255.0);
      cvSet2D(res, i, j, n);
    }
  }

  return res;
}

vector<MyPair> pairs;
vector<IplImage*> l2s;

/* OpenCV foolishness. */
const char* window = "Sorted Images";
const char* l2window = "L2 Difference";
const char* refwindow = "Reference";
IplImage* reference;
void SelectorCallBack(int position)
{
  if (pairs.at(position).img == NULL)
  {
    pairs.at(position).img = cvLoadImage(pairs.at(position).filename.c_str());
  }
  if (pairs.at(position).img == NULL)
  {
    cout << pairs.at(position).filename << " STILL NULL!" << endl;
  }

  // Resize reference, if we need to.
  IplImage* at = pairs.at(position).img;
  if (at != NULL && 
      (at->width != reference->width || at->height != reference->height))
  {
    IplImage* reference_small = 
      cvCreateImage(cvSize(at->width, at->height), 
                    reference->depth,
                    reference->nChannels);
    cvResize(reference, reference_small);
    cvReleaseImage(&reference);
    reference = reference_small;
  }
  if (l2s.at(position) == NULL)
    l2s.at(position) = l2_image(reference, pairs.at(position).img);

  cout << "Idx " << position 
       << " file: " << pairs.at(position).filename 
       << " P: " << pairs.at(position).prob << endl;
  cvShowImage(window, pairs.at(position).img);
  cvShowImage(l2window, l2s.at(position));
  cvShowImage(refwindow, reference);
}


int main(int argc, char** argv)
{
  if (argc != 4 && argc != 5)
  {
    cout << "Usage: bin/sorted_viewer <file> <step index> <reference> [max = -1]" << endl
         << "    max is the number of images to read; -1 means \"all\"." 
         << endl;
    return 1;
  }
  int STEP = atoi(argv[2]);
  reference = cvLoadImage(argv[3]);
  assert(reference);
  int MAX = -1;
  if (argc == 5)
    MAX = atoi(argv[4]);
  
  ifstream IN(argv[1]);
  string line;
  while (getline(IN, line))
  {
    if (line.find("Particle") == string::npos || 
        line.find("floored")  != string::npos || 
        line.find("NULL")     != string::npos)
    {
      continue;
    }

    /*
     * Four values: the string "Particle", the index, a colon, and the value.
     * -1 is the placeholder for "doom", but really shouldn't happen.
     */
    string throwaway;
    int index;
    string colon;
    double value = -1;
    stringstream ss(line);
    ss >> throwaway >> index >> colon >> value;
    pairs.push_back(MyPair(str(format("%d_%d_img.png") % STEP % index), value));
  }

  sort(pairs.begin(), pairs.end());
  reverse(pairs.begin(), pairs.end());

  while (l2s.size() < pairs.size())
    l2s.push_back(NULL);

  if (MAX != -1)
  {
    while (pairs.size() > size_t(MAX))
      pairs.pop_back();
  }

  /*
   * At this point, we have what we need to view them in order; now, we need
   * to get OpenCV doing the right thing. 
   */
  cvNamedWindow(window, 1);
  cvNamedWindow(l2window, 1);
  cvNamedWindow(refwindow, 1);
  cvCreateTrackbar("Idx", window, 0, pairs.size() - 1, SelectorCallBack);
  cvWaitKey();

  foreach(IplImage* i, l2s)
    cvReleaseImage(&i);
}
