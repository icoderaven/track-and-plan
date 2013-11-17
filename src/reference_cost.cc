/*
 * reference_cost.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Drive around manually, and watch how your cost function changes.
 */

#include <cmath>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <cv.h>
#include <highgui.h>
#include "scale.hh"
#include "barecell.hh"
#include "kvparser.hh"
#include "viewcontext.hh"
#include "sensormodels.hh"
#include "utilities.hh"
#include "linear.hh"

using namespace std;
using boost::format;
using namespace textured_localization;

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 4)
    {
      cout << "Usage: bin/reference_cost <map> <config> <reference image>"
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

    if (config["scalefactor"] == "")
    {
      cout << "You must provide a scalefactor!" << endl;
      exit(1);
    }
    double SCALEFACTOR = atof(config["scalefactor"].c_str());
    
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
    }
    ViewContext::Get().SetPose(Pose(x0, y0, z0, 0.0));
    ViewContext::Get().Yellowize(0.6);
    //ViewContext::Get().SetPose(Pose(4104.14, 4136.58, 4070.03, 0));
    //ViewContext::Get().SetPose(Pose(3952.87, 4371.55, 4072.02, 1.8675));
    //ViewContext::Get().SetPose(Pose(3946.26, 4379.41, 4071.75, 1.37878));

    // Pose from Mac's Notebook. Dwing?
    ViewContext::Get().SetPose(Pose(4094.68, 4406.69, 4071.74, 1.39623));

    ViewContext::Get().EnableKeyboard();

    // Read in the reference image.
    IplImage* reference_big = cvLoadImage(argv[3]);
    IplImage* reference = 
      cvCreateImage(cvSize((int)(reference_big->width * SCALEFACTOR),
                           (int)(reference_big->height * SCALEFACTOR)),
                    reference_big->depth,
                    reference_big->nChannels);
    cvResize(reference_big, reference);
    cvReleaseImage(&reference_big);
    exposure_compensation(reference, 2.0);
    cvNamedWindow("Reference", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Difference (L2)", CV_WINDOW_AUTOSIZE);
    cvShowImage("Reference", reference);
    cvWaitKey(30);

    L2SensorModel L2S(10);
    L1SensorModel L1S(reference->width, reference->height);
    L2HueSensorModel H(5000);
    L1HueSensorModel HL1S(reference->width, reference->height);
    GrayScaleL2SensorModel GSL2S(1, false);
    ChiSquaredSensorModel CSS(1.0 / 1500.0);

    IplImage* diff1 = NULL;
    IplImage* diff2 = NULL;

    while (true)
    {
      if (diff1 != NULL)
        cvReleaseImage(&diff1);
      if (diff2 != NULL)
        cvReleaseImage(&diff2);

      IplImage* im = ViewContext::Get().Render();
      if (ViewContext::Get().quit_requested())
        break;
      cout << format("GSL2S: %05.5f  CSS: %05.10f  |  Pose: ") 
                % 0.0 //GSL2S(reference, im)
                % 0.0 //CSS(reference, im)
           << ViewContext::Get().pose()
           << endl;

      bool uncertain;
      im = GSL2S.Inpaint(im, cvScalar(0, 255, 0), &uncertain);
      if (im)
        cvSaveImage("view.png", im);
      if (!im)
        continue;
      if (uncertain)
        cout << "FLOORED" << endl;
      diff2 = cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_8U, 1);
      diff1 = cvCreateImage(cvSize(im->width, im->height), IPL_DEPTH_8U, 1);
      for (int i = 0 ; i < diff1->height ; ++i)
      {
        for (int j = 0 ; j < diff1->width ; ++j)
        {
          CvScalar read_v = cvGet2D(im, i, j);
          CvScalar ref_v = cvGet2D(reference, i, j);
          double l2norm = 0.0;
          double l1norm = 0.0;
          for (int k = 0; k < 3 ; ++k)
          {
            l2norm += pow((read_v.val[k] - ref_v.val[k]), 2);
            l1norm += fabs(read_v.val[k] - ref_v.val[k]);
          }
          cvSet2D(diff2, i, j, cvScalarAll(l2norm));
          cvSet2D(diff1, i, j, cvScalarAll(l1norm));
        }
      }

      cvShowImage("Difference (L2)", diff2);
      cvShowImage("Difference (L1)", diff1);
      cvWaitKey(30);
      cvReleaseImage(&im);

      if (ViewContext::Get().quit_requested())
        break;
    }
    if (diff1 != NULL)
      cvReleaseImage(&diff1);
    if (diff2 != NULL)
      cvReleaseImage(&diff2);

    cvDestroyWindow("Reference");
  }
  catch (string s)
  {
    cout << "Main caught: " << s << endl;
  }
}
