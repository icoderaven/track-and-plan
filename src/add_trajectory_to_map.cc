/*
 * add_trajectory_to_map.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Do what it sounds like; given a map, add the trajectory.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "TNT/tnt.h"
#include "scale.hh"
#include "barecell.hh"
#include "pose.hh"
#include "particle_visualizer.hh"
#include "extended_tnt.hh"

using namespace std;
using namespace textured_localization;
using namespace TNT;

const bool MFILE = true;

int main(int argc, char* argv[])
{
  if (argc < 4)
  {
    cout << "Usage: bin/add_trajectory_to_map <map>"
            " <output> <cam> <deterministic> <laser>" << endl;
    return 1;
  }

  vector<BareCell> cells;
  double x0, y0, z0;
  int cellcount = BareCell::ParseMapFile(argv[1], cells, x0, y0, z0);
  if (cellcount == -1 || cellcount == 0)
  {
    cout << "Problem reading map file!" << endl;
    return 1;
  }

  vector< vector<Pose> > trajectories;
  for (int i = 3 ; i < argc ; ++i)
  {
    vector<Pose> traj;
    ifstream T(argv[i]);
    if (!T)
    {
      cout << "Couldn't open trajectory file \""
           << argv[i] << "\"" << endl
           << "Bailing out!" << endl;
      return 1;
    }
    Pose p;
    while (T >> p)
    {
      traj.push_back(p);
      // Laser poses are given in a laser-rangefinder-centered coordinate
      // frame. For purposes of best pictures, they should be drawn in a
      // robot-centered frame (like the camera coordinates already are). This
      // does that.
      if (i == 4 || i == 5) // Laser & deterministic poses.
      {
        Array2D<double> offset(4, 1, 0.0);
        offset[0][0] = -0.1675 * SCALE;
        offset[1][0] = -0.015 * SCALE;
        offset[2][0] = 0.0;
        offset[3][0] = 1.0;

        Array2D<double> R = matmult(RotZ<double>(p.theta()), offset);
        Pose temp = traj.back();
        traj.pop_back();
        traj.push_back(Pose(R[0][0] + temp.x(),
                            R[1][0] + temp.y(),
                            temp.z(),
                            temp.theta()));
      }
    }
    trajectories.push_back(traj);
    cerr << "Parsed a trajectory of size " 
         << trajectories.back().size() 
         << flush << endl;
  }

  vector<CvScalar> colors;
  colors.push_back(cvScalar(0, 0, 255)); // R
  colors.push_back(cvScalar(0, 255, 0)); // G 
  colors.push_back(cvScalar(255, 0, 0)); // B; fuck you, OpenCV
  // Draw us on top.
  DrawTrajectoryAndSave(argv[2], trajectories, colors, cells);

  // Add an mfile to stdout.
  if (MFILE)
  {
    cout << "camera = [";
    foreach(Pose& p, trajectories[0])
    {
      cout << p << ";" << endl;
    }
    cout << "];" << endl;

    cout << "deterministic = [";
    foreach(Pose& p, trajectories[1])
    {
      cout << p << ";" << endl;
    }
    cout << "];" << endl;

    cout << "laser = [";
    foreach(Pose& p, trajectories[2])
    {
      cout << p << ";" << endl;
    }
    cout << "];" << endl;
  }
}
