/*
 * particle_visualizer.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Turn particle filters into pictures.
 */

#ifndef TEXTURED_LOCALIZATION_PARTICLE_VISUALIZER_HH_INCLUDED
#define TEXTURED_LOCALIZATION_PARTICLE_VISUALIZER_HH_INCLUDED 1

#include <vector>
#include <string>
#include <cv.h>
#include <highgui.h>
#include "particle.hh"
#include "barecell.hh"
#include "pose.hh"

using namespace std;

namespace textured_localization
{
  IplImage* Draw(const vector<Particle>& particles, 
                 const vector<BareCell>& map, Pose* truth=NULL,
                 bool draw_particles=true,
                 vector<Pose>* poses=NULL);
  void DrawAndSave(string filename, 
                   const vector<Particle>& particles, 
                   const vector<BareCell>& map, Pose* truth=NULL,
                   bool draw_particles=true,
                   vector<Pose>* poses=NULL);

  IplImage* DrawTrajectory(const vector< vector<Pose> >& poses,
                           const vector<CvScalar>& colors,
                           const vector<BareCell>& map);

  void DrawTrajectoryAndSave(string filename,
                             const vector< vector<Pose> >& poses,
                             const vector<CvScalar> colors,
                             const vector<BareCell>& map);
}
#endif
