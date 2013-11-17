/*
 * robot_tracking.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Let's do tracking, and see if that works.
 */

#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <cv.h>
#include <highgui.h>
#define foreach BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH
#include "barecell.hh"
#include "random.hh"
#include "kvparser.hh"
#include "viewcontext.hh"
#include "pose.hh"
#include "particle.hh"
#include "austinmotionmodel.hh"
#include "simplemotionmodel.hh"
#include "particle_visualizer.hh"
#include "utilities.hh"
#include "sensormodels.hh"
#include "scale.hh"

using namespace std;
using boost::format;
using namespace textured_localization;

int main(int argc, char* argv[])
{
  try
  {
    if (argc < 7)
    {
      cout << "Usage: bin/robot_tracking <map> <config> <true traj> " 
           << "<random traj> <N> <image prefix> [kidnapped]"
           << endl;
      return 1;
    }

    bool tracking = true;
    if (argc == 8)  // Kidnapped.
      tracking = false;

    if (argc > 8)
    {
      cout << "Too many arguments!" << endl;
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

    // Bounding box, in case we're kidnapped; lets us scatter particles
    // starting at the center of the map.
    vector<int> bbox = BareCell::BoundingBox(map);

    // Open the configuration file.
    KVParser config(argv[2]);

    // Set up the MM.
    AustinMotionModel MM(config);
    //SimpleMotionModel MM(1.0, 1.0, 1.0, 0.5);

    // Spin up OpenGL.
    ViewContext::Get().Init(config, argc, argv, map);
    if (config["robot_cam_height"] != "" && 
        config["floor_z"] != "")
    {
      double cam_height = 
        atof(config["robot_cam_height"].c_str()) * SCALE + 
        atof(config["floor_z"].c_str());

      z0 = cam_height;
    }
    ViewContext::Get().SetPose(Pose(x0, y0, z0, 0));
    ViewContext::Get().DisableKeyboard();
    
    L2SensorModel L2S(20);
    L2HueSensorModel HS(5000);
    GrayScaleL2SensorModel GSL2S(0.1);

    // Open the trajectories.
    ifstream TRUTH(argv[3]);
    if (!TRUTH)
      throw string("Couldn't open ") + argv[3];

    ifstream RANDOM(argv[4]);
    if (!RANDOM)
      throw string("Couldn't open ") + argv[4];

    // Set up the filter.
    vector<Particle> filter;
    for (int i = 0 ; i < atoi(argv[5]) ; ++i)
    {
      if (tracking)
      {
        //vector<double> perturbation = MM.Sample(0, 0, 0, 0);
        vector<double> perturbation;
        perturbation.push_back(0);
        perturbation.push_back(0);
        perturbation.push_back(0);
        filter.push_back(Particle(Pose(x0 + perturbation.at(0),
                                       y0 + perturbation.at(1),
                                       z0,
                                       perturbation.at(2)), 1.0));
      }
      else  // kidnapped
      {
        double xwidth = bbox[1] - bbox[0];
        double ywidth = bbox[3] - bbox[2];
        double xpos = xwidth * Random::Get()->Uniform();
        double ypos = ywidth * Random::Get()->Uniform();
        double angle = radians(360 * Random::Get()->Uniform());
        filter.push_back(Particle(Pose(xpos + bbox[0], 
                                       ypos + bbox[2], z0, angle), 1.0));

      }
    }
    Particle::Normalize(filter);
    DrawAndSave((format("%s_start.png") % argv[6]).str().c_str(), 
                filter, map);

    // We need the true trajectory.
    Pose temp;
    vector<Pose> truth;
    while (TRUTH >> temp)
      truth.push_back(temp);

    // We need to turn the noisy trajectory into actions.
    vector<Pose> noisy_poses;
    while (RANDOM >> temp)
      noisy_poses.push_back(temp);

    vector<Pose> noisy_actions;
    for (size_t i = 1 ; i < noisy_poses.size() ; ++i)
      noisy_actions.push_back(noisy_poses.at(i) - noisy_poses.at(i-1));
    // One fewer actions than poses; good. Now, iterate through them.
    for (size_t step = 0 ; step < noisy_actions.size() ; ++step)
    {
      cout << "Step " << step << endl;
      Pose action = noisy_actions.at(step);
      // Perturb the particles according to this action.
      foreach(Particle& p, filter)
      {
        vector<double> perturbation = MM.Sample(action.x(),
                                                action.y(),
                                                action.theta(),
                                                p.pose().theta());
        p.Move(perturbation.at(0), perturbation.at(1), perturbation.at(2));
      }
      DrawAndSave((format("%s_%d_all.png") % argv[6] % step).str().c_str(),
                  filter, map, &(truth.at(step)));

      // Construct the "truth" image.
      Pose truth2 = truth.at(step);
      truth2.set_z(z0);
      ViewContext::Get().SetPose(truth2);
      IplImage* ref = ViewContext::Get().Render();
      cvSaveImage((format("%s_%d_ref.png") % argv[6] % step).str().c_str(),
                  ref);

      // Weight the particles.
      int floored = 0;
      for (size_t pidx = 0 ; pidx < filter.size() ; ++pidx)
      {
        if (pidx % 1000 == 0)
          cout << "." << flush;
        Particle p = filter.at(pidx);
        ViewContext::Get().SetPose(p.pose());
        IplImage* img = ViewContext::Get().Render();

        double prob = GSL2S(ref, img);

//        cout << format("Particle: %02d: %05.5f %05.5f...") 
//                  % pidx 
//                  % logprob 
//                  % prob;

        filter.at(pidx).set_weight(prob);

        if (filter.at(pidx).weight() < 0.000000001)
        {
          floored++;
          filter.at(pidx).set_weight(0.000000001);
        }
        //cout << endl;

        //cvSaveImage((format("%s_%d_particle_%d.png") 
        //               % argv[6] 
        //               % step 
        //               % pidx).str().c_str(), img);
        cvReleaseImage(&img);
      }
      cout << endl;
      // Show the un-normalized values.
//      sort(filter.begin(), filter.end());
//      cout << "Un-normalized sorted weight:" << endl;
//      reverse_foreach(Particle p, filter)
//      {
//        cout << "  " << p.weight() << endl;
//      }
      Particle::Normalize(filter);
      cvReleaseImage(&ref);

      // How wrong are we, at the moment? Find the best Particle.
      sort(filter.begin(), filter.end());
      assert (filter.front().weight() <= filter.back().weight());
      Particle best = filter.back();
      ViewContext::Get().SetPose(best.pose());
      IplImage* best_im = ViewContext::Get().Render();
      cvSaveImage((format("%s_%d_best.png") % argv[6] % step).str().c_str(),
                  best_im);
      cvReleaseImage(&best_im);
      cout << "Sorted weights: " << endl;
      int printed = 0;
      reverse_foreach(Particle p, filter)
      {
        cout << "  " << p.weight() << endl;
        printed++;
        if (printed > 10)
          break;
      }
      cout << "Floored " << floored << endl;
      cout << "Best / Worst: " 
           << filter.back().weight() / filter.front().weight()
           << endl;
      cout << "Error in "
           << "distance: " 
           << sqrt(pow(best.pose().x() - truth.at(step).x(), 2) + 
                   pow(best.pose().y() - truth.at(step).y(), 2))
           << " angle: " 
           << degrees(fabs(best.pose().theta() - truth.at(step).theta()))
           << endl;

      // Do we need to resample?
      double Neff = 0.0;
      foreach(Particle p, filter)
      {
        Neff += pow(p.weight(), 2);
      }
      Neff = 1.0 / Neff;
      cout << "Neff is: " << Neff << endl;
//      if (Neff < filter.size() / 2)
//      {
      // Resample.
      vector<Particle> new_filter;
      for (int i = 0 ; i < atoi(argv[5]) ; ++i)
      {
        double sample = Random::Get()->Uniform();
        double total = 0.0;
        int idx = 0;
        while (true)
        {
          total += filter.at(idx).weight();
          if (total >= sample)
          {
            new_filter.push_back(filter.at(idx));
            new_filter.back().set_weight(1.0);
            break;
          }
          idx++;
        }
      }
      filter.swap(new_filter);
      // Get the sum back to 1.0.
      Particle::Normalize(filter);
      new_filter.clear();
//      }
      DrawAndSave((format("%s_%d_resampled.png") 
                     % argv[6] 
                     % step).str().c_str(),
                  filter, map, &(truth.at(step)));
    }
  }
  catch (string s)
  {
    cout << "Main caught: " << s << endl;
  }
}
