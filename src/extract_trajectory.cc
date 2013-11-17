/*
 * extract_trajectory.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * It would be handy to be able to extract a trajectory (for, say,
 * follow_trajectory or simulated_robot_tracking) from a SLAM logfile; this
 * does that.
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "scale.hh"
#include "pose.hh"
#include "utilities.hh"

using namespace std;
using namespace textured_localization;

void PrintUsageAndDie()
{
  cout << "Usage: bin/extract_trajectory <map> <logfile> <output> [theta]" 
       << endl;
  exit(1);
}

int main(int argc, char* argv[])
{
  if (argc < 4 || argc > 5)
    PrintUsageAndDie();

  ifstream map(argv[1]);
  if (!map)
  {
    cout << "Couldn't open mapfile " << argv[1] << endl;
    PrintUsageAndDie();
  }
  // We only need the inital pose, not the map itself.
  int count;
  double x0, y0, z0;
  map >> count >> x0 >> y0 >> z0;
  map.close();

  ifstream logfile(argv[2]);
  if (!logfile)
  {
    cout << "Couldn't open logfile " << argv[2] << endl;
    PrintUsageAndDie();
  }

  // Turn the logfile into poses; note that these are really actions, not
  // poses.
  double offset = 0.0;
  if (argc == 5)
    offset = radians(atof(argv[4]));
  cout << "offset: " << offset << endl;
  vector<Pose> actions;
  actions.push_back(Pose(x0, y0, z0, offset));
  string line;
  while (getline(logfile, line))
  {
    stringstream ss(line);
    string keyword;
    ss >> keyword;
    if (keyword == "Action:")
    {
      double x, y, t;
      ss >> x >> y >> t;
      actions.push_back(Pose(SCALE * x, SCALE * y, z0, t));
    }
  }
  logfile.close();

  ofstream OUT(argv[3]);
  // A trajectory is composed of full poses, not deltas, so we need to do
  // that.
  Pose p(actions.front());
  for (size_t i = 1 ; i < actions.size() ; ++i)
  {
    p = p + actions.at(i);
    OUT << p << endl;
  }
}
