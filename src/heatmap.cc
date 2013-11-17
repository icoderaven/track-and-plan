/*
 * heatmap.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Generate a "heat map" describing the cost of various poses in a given map.
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

// How big is our heatmap? (SIZE on a side; odd to make sure there's a middle)
const int SIZE = 41;

int main(int argc, char* argv[])
{
  try
  {
  if (argc != 5)
  {
    cout << "Usage: bin/heatmap <mapfile> <config file> "
         << "<reference image> <output file>"
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
  // Build our sparse version.
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
  //ViewContext::Get().SetPose(Pose(x0, y0, z0, 0.0));
  //ViewContext::Get().SetPose(Pose(x0, y0, z0+1, radians(48)));
  //ViewContext::Get().SetPose(Pose(x0, y0, z0+1, radians(-110)));
  ViewContext::Get().SetPose(Pose(4076.98, 4047.51, 4073.03, -2.111 + radians(10)));
  // Color conversion
  if (config["color_conversion"] == "")
  {
    cout << "NO COLOR CONVERSION MATRIX" << endl;
    return 3;
  }
  Array2D<double> colormatrix;
  stringstream ss(config["color_conversion"]);
  ss >> colormatrix;
  assert (colormatrix.dim1() == 4 && colormatrix.dim2() == 4);

  // Open the reference image, and resize it.
  IplImage* ref_big = cvLoadImage(argv[3]);
  IplImage* ref = cvCreateImage(cvSize(ViewContext::Get().width(),
                                       ViewContext::Get().height()),
                                IPL_DEPTH_8U,
                                3);
  cvResize(ref_big, ref);
  cvReleaseImage(&ref_big);

  L2HueSensorModel L2HS(5000);
  L2SensorModel L2S(50);
  GrayScaleL2SensorModel GSL2S(1, true);
  IplImage* ref2 = GSL2S.ColorConversion(ref, ident<double>(4), false);

  // Construct an empty heat map.
  IplImage* heatmap = cvCreateImage(cvSize(SIZE, SIZE), IPL_DEPTH_64F, 1);
  cvSet(heatmap, cvScalarAll(0.0));

  // Store our original pose.
  Pose orig = ViewContext::Get().pose();

  // Walk over the heatmap.
  int r_best = 0;
  int c_best = 0;
  double prob_best = 0.0;
  for (int r = 0 ; r < heatmap->height ; ++r)
  {
    for (int c = 0 ; c < heatmap->width ; ++c)
    {
      // r & c are image coords; we need their corresponding spatial offsets.
      int roff = r - (SIZE/2);
      int coff = c - (SIZE/2);

      // Move us to this spot.
      ViewContext::Get().SetPose(Pose(orig.x() + roff, 
                                      orig.y() + coff, 
                                      orig.z(),
                                      orig.theta()));
      Pose p = ViewContext::Get().pose();
      double prob = 0.0;
      IplImage* im = NULL;
      if (sparsemap.Impossible(p))
      {
        // 0.99 here and below to keep the heatmap from getting too fouled up.
        prob = 0.0005;
      }
      else
      {
        im = ViewContext::Get().Render();
        bool foo;
        im = L2HS.Inpaint(im, cvScalar(0, 255, 0), &foo);
        if (im == NULL)
        {
          prob = 0.0005;
        }
        else
        {
          //prob = GSL2S(ref, im);
          prob = L2S(ref, im);
          if (prob > prob_best && im != NULL)
          {
            r_best = r;
            c_best = c;
            prob_best = prob;
            cout << format("New best (P: %f) at (%f, %f, %f)")
                      % prob
                      % p.x()
                      % p.y()
                      % p.z()
                 << endl; 
            cvSaveImage("best.png", im);
          }
        }
      }
      //cvSaveImage((format("%d_%d.png") % r % c).str().c_str(), im);
      //cout << format("(%d, %d): %f") % r % c % prob << endl;
      cvSet2D(heatmap, r, c, cvScalarAll(prob));
      if (im != NULL)
        cvReleaseImage(&im);
    }
  }
  double min = minimum_element(heatmap);
  double max = maximum_element(heatmap);
  cout << format("Min: %f Max: %f") % min % max << endl;

  // Before we scale for the image, write the heatmap to a text file.
  ofstream heatmap_out((format("%s.dat") % argv[4]).str().c_str());
  heatmap_out << image_to_array(heatmap) << endl;
  heatmap_out.close();

  cvAddS(heatmap, cvScalarAll(-min), heatmap);
  min = minimum_element(heatmap);
  max = maximum_element(heatmap);
  cvConvertScale(heatmap, heatmap, 255.0 / max);
  cvSaveImage(argv[4], heatmap);
  cvReleaseImage(&ref);
  cvReleaseImage(&ref2);
  cvReleaseImage(&heatmap);
  }
  catch (string s)
  {
    cout << "Caught:" << endl << s << endl;
    return 1;
  }
}
