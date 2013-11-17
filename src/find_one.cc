/*
 * find_one.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Using a single reference image, generate a HUGE number of samples, and try
 * to find it.
 */

#include <cassert>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include <cv.h>
#include <highgui.h>
#include "barecell.hh"
#include "particle.hh"
#include "particle_visualizer.hh"
#include "kvparser.hh"
#include "pose.hh"
#include "utilities.hh"
#include "random.hh"
#include "austinmotionmodel.hh"
#include "sensormodels.hh"
#include "viewcontext.hh"
#include "scale.hh"

using namespace std;
using namespace boost;
namespace bfs = boost::filesystem;
using namespace textured_localization;

void PrintUsageAndDie()
{
  cout << "Usage: bin/find_one <map> <configuration> <N> <reference>"
       << endl;
  exit(1);
}

// Best at the front; worst at the back.
pair<size_t, size_t> FindBestAndWorstParticles(const vector<Particle>& filter)
{
  size_t best = 0;
  size_t worst = 0;
  double best_w = -1;
  double worst_w = 10;
  for (size_t i = 0; i < filter.size() ; ++i)
  {
    if (filter[i].weight() > best_w)
    {
      best_w = filter[i].weight();
      best = i;
    }
    if (filter[i].weight() < worst_w)
    {
      worst_w = filter[i].weight();
      worst = i;
    }
  }
  return make_pair(best, worst);
}

/* Go cat go. */
int main(int argc, char* argv[])
{
  try
  {
    if (argc != 5)
      PrintUsageAndDie();

    // Read in the map.
    double x0, y0, z0;
    vector<BareCell> map;
    int cells_read = BareCell::ParseMapFile(argv[1], map, x0, y0, z0);
    if (cells_read == -1)
    {
      cout << "Read or parse failure of " << argv[1] << endl;
      return 1;
    }

    // Open the configuration file.
    KVParser config(argv[2]);

    if (config["scalefactor"] == "")
    {
      cout << "You must provide a scalefactor!" << endl;
      PrintUsageAndDie();
    }
    double SCALEFACTOR = atof(config["scalefactor"].c_str());

    // The particle count.
    int PARTICLE_COUNT = atoi(argv[3]);
    if (PARTICLE_COUNT == 0)
    {
      cout << "Need more than 0 particles!" << endl;
      PrintUsageAndDie();
    }

    // The reference image.
    IplImage* reference_big = cvLoadImage(argv[4]);
    IplImage* reference = 
      cvCreateImage(cvSize((int)(reference_big->width * SCALEFACTOR),
                           (int)(reference_big->height * SCALEFACTOR)),
                    reference_big->depth,
                    reference_big->nChannels);
    cvResize(reference_big, reference);
    cvReleaseImage(&reference_big);
    // For all-synth-mode.
    //cvReleaseImage(&reference);

    // Get OpenGL up and running.
    ViewContext::Get().Init(config, argc, argv, map);
    // If we know where the floor is...
    if (config["robot_cam_height"] != "" && config["floor_z"] != "")
    {
      double cam_height = 
        atof(config["robot_cam_height"].c_str()) * SCALE + 
        atof(config["floor_z"].c_str());

      z0 = cam_height;
    }
    ViewContext::Get().SetPose(Pose(x0, y0, z0, radians(51)));
    ViewContext::Get().DisableKeyboard();
    //reference = ViewContext::Get().Render();
    cvNamedWindow("Reference", CV_WINDOW_AUTOSIZE);
    cvShowImage("Reference", reference);
    cvWaitKey(30);

    // Build our sensor models.
    L1SensorModel L1S(ViewContext::Get().width(), 
                      ViewContext::Get().height());
    L2SensorModel L2S(10);
    L1HueSensorModel L1HS(ViewContext::Get().width(),
                          ViewContext::Get().height());
    L2HueSensorModel HS(5000);
    GrayScaleL2SensorModel GSL2S(1, true);
    
    // We're doing kidnapped, so let's scatter some particles; we start by
    // building the bounding box, and we go from there.
    //
    // TODO: Add the inside-outside test, 'cause that would be nice.
    vector<Particle> filter;
    vector<int> bbox = BareCell::BoundingBox(map);
    double bbox_x_width = bbox[1] - bbox[0];
    double bbox_y_width = bbox[3] - bbox[2];
    //for (int i = 0 ; i < PARTICLE_COUNT ; ++i)
    //{
    //  double xpos = bbox_x_width * Random::Get()->Uniform();
    //  double ypos = bbox_y_width * Random::Get()->Uniform();
    //  double angle = radians(360 * Random::Get()->Uniform());
    //  filter.push_back(Particle(Pose(xpos + bbox[0], 
    //                                 ypos + bbox[2],
    //                                 z0,
    //                                 angle),
    //                            1.0));

    //}

    // Add some "truth" particles
    //filter.push_back(Particle(Pose(4088.02, 4091.02, z0, radians(50)), 1.0));
    filter.push_back(Particle(Pose(x0, y0, z0, radians(0)), 1.0));
    Particle::Normalize(filter);
    Pose p = ViewContext::Get().pose();
    DrawAndSave("all.png", filter, map, &p);

    cvWaitKey(50);
    int pidx = 0;
    foreach(Particle& p, filter)
    {
      // Render what I'm seeing.
      ViewContext::Get().SetPose(p.pose());
      cout << "Set pose to " << p.pose() << endl;
      IplImage* im = ViewContext::Get().Render();
      cvNamedWindow("W");
      cvShowImage("W", im);
      cvWaitKey(0);
      cvDestroyWindow("W");
      p.set_weight(GSL2S(reference, im));
      if (p.weight() < 0.0000000001)
        p.set_weight(0.0000000001);

      cvSaveImage((format("img_%d.png") % pidx).str().c_str(), im);
      //cout << format("Particle %d: %05.5f") % pidx % p.weight() << endl;
      cvReleaseImage(&im);
      pidx++;
    }

    Particle::Normalize(filter);
    pair<size_t, size_t> result = FindBestAndWorstParticles(filter);
    format fmt("Best: (%d) %05.5f | Worst: (%d) %05.5f | Ratio: %05.5f");
    cout << fmt % result.first
                % filter[result.first].weight()
                % result.second
                % filter[result.second].weight()
                % (filter[result.first].weight() / 
                   filter[result.second].weight())
         << endl
         << "Best pose: " << filter[result.first].pose()
         << endl;
    ViewContext::Get().SetPose(filter[result.first].pose());
    cout << "Pose is now: " << ViewContext::Get().pose() << endl;
    IplImage* best = ViewContext::Get().Render();
    cvSaveImage("best.png", best);
    cvReleaseImage(&best);
  }
  catch (string e)
  {
    cout << "Main caught: " << e << endl;
  }
}
