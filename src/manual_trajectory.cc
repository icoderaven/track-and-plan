/*
 * manual_trajectory.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Generate a trajectory by hand; this is to support kidnapped-robot
 * experiments.
 */

#include <iostream>
#include <fstream>
#include <cv.h>
#include <highgui.h>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "scale.hh"
#include "barecell.hh"
#include "kvparser.hh"
#include "viewcontext.hh"
#include "utilities.hh"
#include "sensormodels.hh"

using namespace std;
using boost::format;
using namespace textured_localization;

IplImage* reference = NULL;

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 4 && argc != 5)
    {
      cout << "Usage: bin/manual_trajectory <map> <config> <output file> [ref]" 
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

    // Set up the reference image, if we want one.
    if (argc == 5) 
    {
      reference = cvLoadImage(argv[4]);
      if (reference == NULL)
      {
        cout << "Can't open provided reference image " 
             << argv[4] 
             << "!" << endl;
        return 1;
      }
      IplImage* ref_small = 
        cvCreateImage(cvSize(ViewContext::Get().width(), 
                             ViewContext::Get().height()),
                      reference->depth,
                      reference->nChannels);
      cvResize(reference, ref_small);
      cvReleaseImage(&reference);
      reference = ref_small;
      cvShowImage("Reference", reference);
    }

    // If we know where the floor is...
    if (config["robot_cam_height"] != "" && 
        config["floor_z"] != "")
    {
      double cam_height = 
        atof(config["robot_cam_height"].c_str()) * SCALE + 
        atof(config["floor_z"].c_str());

      z0 = cam_height;
    }
    //ViewContext::Get().SetPose(Pose(x0-8, y0-5, z0, radians(51)));
    ViewContext::Get().SetPose(Pose(3950.86, 4369.04, 4071.96, 2.04202));
    double cellx, celly, cellz = 0.0;
    foreach(const BareCell& b, ViewContext::Get().cells())
    {
      cellx += b.x();
      celly += b.y();
      cellz += b.z();
    }
    cellx /= ViewContext::Get().cells().size();
    celly /= ViewContext::Get().cells().size();
    cellz /= ViewContext::Get().cells().size();


    // Rather than set that manually, let's find put ourselves at the
    // "origin", and hope that works.
//    ViewContext::Get().SetPose(Pose(4096.02, 4096.02, 4096.02, 0));
    ViewContext::Get().Yellowize(1.3);
    ViewContext::Get().EnableKeyboard();

    // Construct out output file.
    ofstream OUT(argv[3]);

    L2SensorModel L2S(10, false);
    NormalizedL2SensorModel NL2S(10);
    PerChannelNormalizedL2SensorModel PCNL2S(10);
    MutualInformationSensorModel MIS(16);
    bool foo;
    cvNamedWindow("Inpainted");
    cvNamedWindow("Reference");
    cvNamedWindow("Normed Reference");
    cvNamedWindow("Normed Inpainted");
    IplImage* im = NULL;
    IplImage* ref_norm = NULL;
    IplImage* rea_norm = NULL;
    while (true)
    {
      cout << "Pose: " << ViewContext::Get().pose() << endl;
//      cout << format("avg: %f %f %f") % cellx % celly % cellz << endl;
      if (im != NULL)
        cvReleaseImage(&im);
      im = ViewContext::Get().Render();
      im = L2S.Inpaint(im, cvScalar(0, 255, 0), &foo);
      if (reference != NULL)
      {
        cout << "L2: " << L2S(reference, im) << endl;
        cout << "PCNL2: " << PCNL2S(reference, im) << endl;
//        cout << "Mutual information: " << MIS(reference, im) << endl;
//        cout << "AutoMutual information: " 
//             << MIS(reference, reference) 
//             << endl;
      }
      cvWaitKey(30);
      if (im != NULL)
      {
        cvShowImage("Inpainted", im);
        if (rea_norm != NULL)
          cvReleaseImage(&rea_norm);
        if (ref_norm != NULL)
          cvReleaseImage(&ref_norm);

        ref_norm = PCNL2S.NormalizePerChannel(reference, false);
        rea_norm = PCNL2S.NormalizePerChannel(im, false);
        cvShowImage("Normed Reference", ref_norm);
        cvShowImage("Normed Inpainted", rea_norm);
      }
      if (ViewContext::Get().quit_requested())
        break;
    }

    OUT << Pose(x0, y0, z0, 0.0) << endl;
    foreach(Pose p, ViewContext::Get().Poses())
    {
      OUT << p << endl;
    }
  }
  catch (string s)
  {
    cout << "Main caught: " << s << endl;
  }
  if (reference != NULL)
    cvReleaseImage(&reference);
}
