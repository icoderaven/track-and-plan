/*
 * heatmap_rotation.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Just like heatmap.cc, but does it for orientation (in theta), not pose.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "barecell.hh"
#include "kvparser.hh"
#include "viewcontext.hh"
#include "utilities.hh"
#include "sensormodels.hh"
#include "sparsemap.hh"
#include "scale.hh"
#include "extended_tnt.hh"

using namespace std;
using namespace textured_localization;
using namespace boost;

int main(int argc, char* argv[])
{
  if (argc != 6)
  {
    cout << "Usage: bin/heatmap <mapfile> <config file> "
         << "<reference image> <stepsize (degrees)> <output file>"
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
  SparseMap sparsemap(map);

  // How big is the map we've just parsed?
  vector<int> bbox = BareCell::BoundingBox(map);
  cout << format("BBox: X: [%d, %d] (%d) Y: [%d, %d] (%d), Z: [%d, %d] (%d)")
            % bbox[0] % bbox[1] % (bbox[1] - bbox[0])
            % bbox[2] % bbox[3] % (bbox[3] - bbox[2])
            % bbox[4] % bbox[5] % (bbox[5] - bbox[4])
       << endl;

  // Open the configuration file.
  KVParser config(argv[2]);

  // Spin up OpenGL.
  ViewContext::Get().Init(config, argc, argv, map);
  if (config["robot_cam_height"] != "" && 
      config["floor_z"] != "")
  {
    double cam_height = 
      (atof(config["robot_cam_height"].c_str()) * SCALE) + 
      atof(config["floor_z"].c_str());

    z0 = cam_height;
  }
  if (config["color_conversion"] == "")
  {
    cout << "NO COLOR CONVERSION MATRIX" << endl;
    return 3;
  }
  Array2D<double> colormatrix;
  stringstream ss(config["color_conversion"]);
  ss >> colormatrix;
  assert (colormatrix.dim1() == 4 && colormatrix.dim2() == 4);
  //ViewContext::Get().SetPose(Pose(x0-8, y0-5, z0, 0));
  //ViewContext::Get().SetPose(Pose(4104.14, 4136.58, 4070.03, 0));
  ViewContext::Get().SetPose(Pose(4102.94, 4133.88, 4072.03, 0));
  ViewContext::Get().Yellowize(0.6);

  // Open the reference image, and resize it.
  IplImage* ref_big = cvLoadImage(argv[3]);
  IplImage* ref = cvCreateImage(cvSize(ViewContext::Get().width(),
                                       ViewContext::Get().height()),
                                IPL_DEPTH_8U,
                                3);
  cvResize(ref_big, ref);
  cvReleaseImage(&ref_big);

  // Construct the sensor model.
  L2HueSensorModel L2HS(5000);
  L2SensorModel L2S(50);
  GrayScaleL2SensorModel GSL2S(1);

  //IplImage* ref2 = GSL2S.ColorConversion(ref, invert(colormatrix), false);
  //IplImage* ref2 = GSL2S.ColorConversion(ref, colormatrix, false);

  // Show us a picture.
  cvNamedWindow("Reference");
  cvShowImage("Reference", ref);
  cvNamedWindow("Reference2");

  // Get the stepsize (which we need to know how big the heatmap should be).
  double stepsize = atof(argv[4]);
  int size = (int)((360.0 / stepsize) + 0.5);

  // Construct an empty heat map.
  IplImage* heatmap = cvCreateImage(cvSize(size, 50), IPL_DEPTH_64F, 1);
  cvSet(heatmap, cvScalarAll(0.0));


  // Walk over the heatmap.
  double prob_best = 0.0;
  int best = 0;
  double prob = 0.0;
  for (int i = 0 ; i < size ; ++i)
  {
    if (i % 50 == 0)
      cvWaitKey(15);
    ViewContext::Get().Rotate(radians(stepsize));
    IplImage* im = ViewContext::Get().Render();
    bool foo = false;
    im = GSL2S.Inpaint(im, cvScalar(0, 255, 0), &foo);
    if (im == NULL || foo)
    {
      prob = 0.0001;
    }
    else
    {
      //prob = GSL2S(ref2, im);
      prob = L2S(ref, im);
    }
    //cout << "Current pose: " << degrees(ViewContext::Get().pose().theta()) << endl;
    Pose p = ViewContext::Get().pose();
    if (prob > prob_best && im != NULL)
    {
      double t = degrees(ViewContext::Get().pose().theta());
      cout << format("New best: [%d]: P: %f T: %f") % i % prob % t << endl;
      prob_best = prob;
      best = i;
      cvSaveImage("best.png", im);
    }
    for (int j = 0 ; j < 50 ; ++j)
      cvSet2D(heatmap, j, i, cvScalarAll(prob));
    cvReleaseImage(&im);
  }
  //cvWaitKey(0);
  double min = minimum_element(heatmap);
  double max = maximum_element(heatmap);
  cout << format("Min: %f Max: %f") % min % max << endl;

  // Before we scale for the image, write the heatmap to a text file.
//  ofstream heatmap_out((format("%s.dat") % argv[5]).str().c_str());
//  heatmap_out << image_to_array(heatmap) << endl;
//  heatmap_out.close();

  cvAddS(heatmap, cvScalarAll(-min), heatmap);
  min = minimum_element(heatmap);
  max = maximum_element(heatmap);
  cout << format("Min: %f Max: %f") % min % max << endl;
  cvConvertScale(heatmap, heatmap, 255.0 / max);
  cvSaveImage(argv[5], heatmap);
  cvReleaseImage(&ref);
  cvReleaseImage(&heatmap);
}
