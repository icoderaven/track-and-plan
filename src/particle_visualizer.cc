/*
 * particle_visualizer.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation; see particle_visualizer.hh.
 */

#include <cassert>
#include <algorithm>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "particle_visualizer.hh"

using namespace std;
namespace textured_localization
{

IplImage* Draw(const vector<Particle>& particles, 
               const vector<BareCell>& map,
               Pose* truth, bool draw_particles,
               vector<Pose>* poses)
{
  // Step 1: limit ourselves to the cells we care about.
  int z = (int)particles.at(0).pose().z();
  vector<BareCell> cells;
  foreach(BareCell b, map)
  {
    if (b.z() == z)
      cells.push_back(b);
  }

  // Step 2: figure out how big our image is going to be.
  vector<int> bbox = BareCell::BoundingBox(cells);
  foreach(Particle p, particles)
  {
    if (bbox[0] > p.pose().x())
      bbox[0] = p.pose().x();
    if (bbox[1] < p.pose().x())
      bbox[1] = p.pose().x();
    if (bbox[2] > p.pose().y())
      bbox[2] = p.pose().y();
    if (bbox[3] < p.pose().y())
      bbox[3] = p.pose().y();
    if (bbox[4] > p.pose().z())
      bbox[4] = p.pose().z();
    if (bbox[5] < p.pose().z())
      bbox[5] = p.pose().z();
  }
  if (truth != NULL)
  {
    if (bbox[0] > truth->x())
      bbox[0] = truth->x();
    if (bbox[1] < truth->x())
      bbox[1] = truth->x();
    if (bbox[2] > truth->y())
      bbox[2] = truth->y();
    if (bbox[3] < truth->y())
      bbox[3] = truth->y();
    if (bbox[4] > truth->z())
      bbox[4] = truth->z();
    if (bbox[5] < truth->z())
      bbox[5] = truth->z();
  }
  if (poses != NULL)
  {
    vector<double> pbb = Pose::BoundingBox(*poses);
    bbox[0] = min(bbox[0], (int)floor(pbb[0]));
    bbox[1] = max(bbox[1], (int)ceil(pbb[1]));
    bbox[2] = min(bbox[2], (int)floor(pbb[2]));
    bbox[3] = max(bbox[3], (int)ceil(pbb[3]));
  }
  
  int xsize = bbox[1] - bbox[0];
  int ysize = bbox[3] - bbox[2];

  // Step 3: build the image.
  IplImage* res = cvCreateImage(cvSize(xsize+5, ysize+5), IPL_DEPTH_8U, 3);
  //cvSet(res, cvScalarAll(0.0));
  cvSet(res, cvScalarAll(255.0));

  // 4: fill in the map.
  foreach(BareCell b, cells)
  {
    cvSet2D(res, b.y() - bbox[2], b.x() - bbox[0], 
            //cvScalar(b.b(0)*255, b.g(0)*255, b.r(0)*255));
            cvScalarAll(0.0));
  }

  // 5: fill in the particles.
  if (draw_particles)
  {
    foreach(Particle p, particles)
    {
      cvSet2D(res, p.pose().y() - bbox[2], p.pose().x() - bbox[0],
              cvScalar(0, 0, 255));
    }
  }

  if (truth != NULL)
  {
    CvScalar old = cvGet2D(res, truth->y() - bbox[2], truth->x() - bbox[0]);
    if (old.val[1] == 255.0)  // Particle + truth
    {
      cvSet2D(res, truth->y() - bbox[2], truth->x() - bbox[0],
              cvScalar(0, 255, 255));
    }
    else  // Just truth.
    {
      cvSet2D(res, truth->y() - bbox[2], truth->x() - bbox[0],
              cvScalar(0, 0, 255));
    }
  }

  cvFlip(res, 0);
  return res;
}

void DrawAndSave(string filename,
                 const vector<Particle>& particles,
                 const vector<BareCell>& map,
                 Pose* truth,
                 bool draw_particles,
                 vector<Pose>* poses)
{
  IplImage* im = Draw(particles, map, truth, draw_particles, poses);
  cvSaveImage(filename.c_str(), im);
  cvReleaseImage(&im);
}

IplImage* DrawTrajectory(const vector< vector<Pose> >& poses, 
                         const vector<CvScalar>& colors,
                         const vector<BareCell>& map)
{
  assert (poses.size() == colors.size());
  // Fake a Particle, so we can use DrawMap.
  vector<Particle> filter;
  filter.push_back(Particle(poses.at(0).at(0), 1.0));
  // Flatten the poses, to help Draw.
  vector<Pose> flat;
  for (size_t i = 0; i < poses.size() ; ++i)
    for (size_t j = 0;  j < poses[i].size() ; ++j)
      flat.push_back(poses[i][j]);
  IplImage* res = Draw(filter, map, NULL, false, &flat);
  cvFlip(res, 0);  // Need to remember to undo that.

  // Need the bounding box.
  vector<BareCell> zs;
  foreach(BareCell b, map)
    if (b.z() == (int)poses[0][0].z())
      zs.push_back(b);
  vector<int> bbox = BareCell::BoundingBox(zs);
  vector<double> pbb = Pose::BoundingBox(flat);
  bbox[0] = min(bbox[0], (int)floor(pbb[0]));
  bbox[1] = max(bbox[1], (int)ceil(pbb[1]));
  bbox[2] = min(bbox[2], (int)floor(pbb[2]));
  bbox[3] = max(bbox[3], (int)ceil(pbb[3]));

  // This for loop is top-down to put the camera colors on top; ergo, we can
  // always see the camera trajectory. Note that i must be a signed value, or
  // the shit goes wack, yo. (This insight due to Ricco)
  for (int i = poses.size() - 1 ; i >= 0 ; --i)
  {
    for (size_t j = 0 ; j < poses[i].size() - 1; ++j)
    {
//      cvSet2D(res, 
//              poses[i][j].y() - bbox[2], poses[i][j].x() - bbox[0],
//              colors[i]);
      cvLine(res, cvPoint(poses[i][j].x() - bbox[0], 
                          poses[i][j].y() - bbox[2]),
                  cvPoint(poses[i][j+1].x() - bbox[0],
                          poses[i][j+1].y() - bbox[2]),
                  colors[i], 1, CV_AA);

    }
  }
  cvFlip(res, 0);
  return res;
}

void DrawTrajectoryAndSave(string filename,
                           const vector< vector<Pose> >& poses,
                           const vector<CvScalar> colors,
                           const vector<BareCell>& map)
{
  IplImage* res = DrawTrajectory(poses, colors, map);
  cvSaveImage(filename.c_str(), res);
  cvReleaseImage(&res);
}




}
