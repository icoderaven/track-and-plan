/*
 * viewer.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * A 3-D mapviewer.
 */

#include <sys/time.h>
#include <iostream>
#include <vector>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <cv.h>
#include <highgui.h>
#include <GL/gl.h>
#include <GL/glu.h>
// May as well make it easy on ourselves.
#include "barecell.hh"
#include "kvparser.hh"
#include "viewcontext.hh"
#include "scale.hh"

using namespace std;
using namespace textured_localization;
namespace bfs = boost::filesystem;
using boost::format;
using namespace TNT;

void PrintUsageAndDie(int msg = 0)
{
  cout << "Usage: bin/viewer <mapfile> <configfile> <output image>" << endl
       << "  Render an image to output image." << endl << endl
       << "  <configfile> contains camera information " << endl
       << "  (see viewcontext.hh) as well as matrices needed to get" << endl
       << "  the renderer in the right place." << endl << endl
       << "  Specifically: scanner, camera, and spin, all as one-line" << endl
       << "  strings, TNT-style (that is, TNT matrices with the" << endl
       << "  newlines removed)."
       << endl;

  if (msg == 1)
    cout << "**** Died because of an unopenable file. ****" << endl;

  if (msg == 2)
    cout << "**** Died because of a missing matrix! ****" << endl;

  exit(1);
}

int main(int argc, char* argv[])
{
  try
  {
    /*
     * Make sure you've done the right thing on the command line.
     */
    if (argc != 4)
      PrintUsageAndDie();

    // Open the map.
    bfs::path mappath(argv[1]);
    if (!bfs::exists(mappath))
      PrintUsageAndDie(1);

    vector<BareCell> cells;
    bfs::ifstream mapstream(mappath);
    int size;
    double x0, y0, z0;
    mapstream >> size >> x0 >> y0 >> z0;
    BareCell scratch;
    for (int i = 0 ; i < size ; ++i)
    {
      mapstream >> scratch;
      cells.push_back(scratch);
    }
    mapstream.close();
    cout << "Read in " << cells.size() << " map cells." << endl;

    // Open the config file.
    KVParser config(argv[2]);

    // Start setup.
    ViewContext::Get().Init(config, argc, argv, cells);
    // The origin, from the logfile.
    ViewContext::Get().SetPoseFromConfig(config, x0, y0, z0);
    IplImage* im = ViewContext::Get().Render();
    cvSaveImage(argv[3], im);
    cvReleaseImage(&im);
  }
  catch (string s)
  {
    cout << "Main caught an exception:" << endl << s << endl;
    return (1);
  }
}
