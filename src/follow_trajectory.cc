/*
 * follow_trajectory.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Given a trajectory file (see manual_trajectory or noisify_trajectory.py),
 * render how it went.
 */

#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <cv.h>
#include <highgui.h>
#define foreach BOOST_FOREACH
#include "TNT/tnt.h"
#include "scale.hh"
#include "barecell.hh"
#include "kvparser.hh"
#include "viewcontext.hh"
#include "pose.hh"
#include "utilities.hh"

using namespace std;
using boost::format;
using namespace textured_localization;

int main(int argc, char* argv[])
{
  try
  {
    if (argc < 4)
    {
      cout << "Usage: bin/follow_trajectory <map> <config> <traj> [img prefix]"
           << endl;
      return 1;
    }

    // Open the map.
    double x0, y0, z0;
    vector<BareCell> map;
    int read = BareCell::ParseMapFile(argv[1], map, x0, y0, z0);
    if (read == -1)
    {
      cout << "Read or parse failure of file " << argv[1] << endl;
      return 1;
    }
    cout << "Read in " << read << " cells." << endl;

    // Open the configuration file.
    KVParser config(argv[2]);

    // Spin up OpenGL.
    ViewContext::Get().Init(config, argc, argv, map);
    // If we know where the floor is...
    if (config["robot_cam_height"] != "" && 
        config["floor_z"] != "")
    {
      double cam_height = 
        atof(config["robot_cam_height"].c_str()) * SCALE + 
        atof(config["floor_z"].c_str());

      z0 = cam_height;
      cout << "z0: " << z0 << endl;
    }
    ViewContext::Get().SetPose(Pose(x0, y0, z0, 0));
    ViewContext::Get().Yellowize(0.6);
    ViewContext::Get().DisableKeyboard();

    // Open the trajectory.
    ifstream TRAJ(argv[3]);
    if (!TRAJ)
    {
      cout << "Couldn't open the trajectory!" << endl;
      return 1;
    }

    int step = 0;
    Pose p;
    TNT::Stopwatch Q;
    Q.start();
    while (TRAJ >> p)
    {
      p.set_z(z0);
      ViewContext::Get().SetPose(p);
      cout << "Just set pose to " << p << endl;
      IplImage* im = ViewContext::Get().Render();
      if (argc == 5)
      {
        cvSaveImage((format("%s_%d.png") % argv[4] % step).str().c_str(), im);
      }
      cvReleaseImage(&im);
      step++;
      usleep(1e4);
    }
    Q.stop();
    cout << "That trajectory took " << Q.read() << " seconds." << endl;
  }
  catch (string s)
  {
    cout << "Main caught: " << s << endl;
  }
}
