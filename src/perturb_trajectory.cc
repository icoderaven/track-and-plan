/*
 * perturb_trajectory.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Given a manually-generated trajectory (see manual_trajectory.cc), plus some
 * MM parameters, generate a new, perturbed, trajectory.
 */

#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <boost/format.hpp>
#include "pose.hh"
#include "kvparser.hh"
#include "austinmotionmodel.hh"

using namespace std;
using namespace textured_localization;
using boost::format;

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 4)
    {
      cout << "Usage: bin/perturb_trajectory <config> <input> <output>" 
           << endl;
      return 1;
    }

    // Build the motion model.
    KVParser config(argv[1]);
    AustinMotionModel MM(config);

    // Open the logs.
    ifstream IN(argv[2]);
    if (!IN)
      throw string("Couldn't open input file ") + argv[2];
    ofstream OUT(argv[3]);
    if (!OUT)
      throw string("Couldn't open output file ") + argv[3];

    // Figure out what we did.
    vector<Pose> poses;
    Pose p;
    while (IN >> p)
      poses.push_back(p);

    /*
     * The sequence of poses implies a sequence of actions of the same length;
     * the first action is (0, 0, 0, 0).
     */
    vector<Pose> actions;
    actions.push_back(Pose(0, 0, 0, 0));
    for (size_t i = 1 ; i < poses.size() ; ++i)
    {
      actions.push_back(poses.at(i) - poses.at(i-1));
    }
  
    // Quick sanity check.
    assert (actions.size() == poses.size());

    // Construct the noised-up trajectory.
    double facing = poses.at(0).theta();
    vector<Pose> noisy_actions;
    foreach(Pose& action, actions)
    {
      vector<double> deltas = 
        MM.Sample(action.x(), action.y(), action.theta(), facing);
      facing += deltas.at(2);
      noisy_actions.push_back(
          Pose(deltas.at(0), deltas.at(1), 0, deltas.at(2)));
    }
    assert (actions.size() == noisy_actions.size());
//    for (size_t i = 0 ; i < actions.size() ; ++i)
//    {
//      cout << "A: " << actions.at(i) 
//           << " NA: " << noisy_actions.at(i) << endl;
//    }
    
    // Now, to dump this back out to our output file.
    Pose now = poses.at(0);
    foreach(Pose p, noisy_actions)
    {
      OUT << now << endl;
      now = now + p;
    }
  }
  catch (string s)
  {
    cout << "Failed with: " << endl << "    " << s << endl;
    return 1;
  }
}
