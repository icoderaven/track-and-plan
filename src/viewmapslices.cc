/*
 * viewmapslices.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * View an x-y slice through a map, with variable z.
 */

#include <stdio.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cv.h>
#include <highgui.h>

using namespace std;

class Cube
{
  public:
    int x;
    int y;
    int z;
    int r;  
    int g;  
    int b;  

    Cube(int _x, int _y, int _z, int _r, int _g, int _b)
      : x(_x), y(_y), z(_z), 
        r(_r), g(_g), b(_b)
    {
    }
};

char* windowname;
int zt = 0;
int xsize;
int ysize;
int zsize;
int robot_x;
int robot_y;
// We don't bother with the robot's Z.

vector<CvMat*>* ims = NULL;
vector<Cube>* cubes = NULL;

CvMat* DrawLayer(int z)
{
  assert(cubes != NULL);
  // Collect up the interesting z-values.
  vector<Cube> C;
  for (int i = 0; i < cubes->size() ; ++i)
    if (cubes->at(i).z == z)
      C.push_back(cubes->at(i));

  //CvMat* result = cvCreateMat(ysize, xsize, CV_8UC1);
  CvMat* result = cvCreateMat(ysize, xsize, CV_8UC3);
  cvSet(result, cvScalar(255, 255, 255));
  for (int i = 0 ; i < C.size() ; ++i)
  {
    CvScalar v = 
      cvScalar(C[i].b, C[i].g, C[i].r);
    cvSet2D(result, (ysize - C[i].y) - 1, C[i].x, v);
  }

  return result;
}

void SelectorCallBack(int position)
{
  //cout << "Z: " << position << endl;
  if (ims->at(position) == NULL)
  {
    ims->at(position) = DrawLayer(position);
  }
  cvShowImage(windowname, ims->at(position));
}

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    cout 
      << "Usage: ./viewmapslices <mapfile> [write images with this basename]" 
      << endl;
    return 1;
  }

  windowname = argv[1];

  // Need to do the parsing and the sizing.
  int min_x = 1000000;
  int max_x = -1000000;
  int min_y = 1000000;
  int max_y = -1000000;
  int min_z = 1000000;
  int max_z = -1000000;
  cubes = new vector<Cube>();
  ifstream map(argv[1], ifstream::in);
  if (!map)
  {
    cout << "File \"" << argv[1] << "\" cannot be opened!" << endl;
    return 1;
  }
  // Get the count, and robot pose.
  int count;
  double rx, ry, rz;
  map >> count >> rx >> ry >> rz;

  int x, y, z, hits; double odo;

  while (map >> x >> y >> z >> hits >> odo)
  {
    vector<double> pc;
    vector<double> r;
    vector<double> g;
    vector<double> b;
    min_x = min(min_x, x);
    min_y = min(min_y, y);
    min_z = min(min_z, z);
    max_x = max(max_x, x);
    max_y = max(max_y, y);
    max_z = max(max_z, z);
    for (int i = 0 ; i < 6 ; ++i)
    {
      double rr, gg, bb, pcc;
      map >> rr >> gg >> bb >> pcc;
      pc.push_back(pcc);
      r.push_back(rr);
      g.push_back(gg);
      b.push_back(bb);
    }
    double avg_r = 0.0; 
    double avg_g = 0.0;
    double avg_b = 0.0;
    double pc_t = 0.0;
    for (int i = 0 ; i < 6 ; ++i)
    {
      avg_r += r[i];
      avg_g += g[i];
      avg_b += b[i];
      pc_t += pc[i];
    }
    //cout << "pc_t is " << pc_t << endl;
    avg_r /= pc_t;
    avg_g /= pc_t;
    avg_b /= pc_t;
    //cout << avg_r << " " << avg_g << " " << avg_g << endl;
    cubes->push_back(Cube(x, y, z, avg_r, avg_g, avg_b));
  }

//  int x, y, z, hits, r, g, b, pc;
//  double odo;
//  while (map >> x >> y >> z >> hits >> odo >> pc >> r >> g >> b)
//  
//    min_x = min(min_x, x);
//    min_y = min(min_y, y);
//    min_z = min(min_z, z);
//
//    max_x = max(max_x, x);
//    max_y = max(max_y, y);
//    max_z = max(max_z, z);
//
//    cubes->push_back(Cube(x, y, z, r, g, b));
//  }

  cout << "min_x: " << min_x << " max_x: " << max_x << endl;
  cout << "min_y: " << min_y << " max_y: " << max_y << endl;
  cout << "min_z: " << min_z << " max_z: " << max_z << endl;

  // Translate to 0
  for (int i = 0 ; i < cubes->size() ; ++i)
  {
    cubes->at(i).x -= min_x;
    cubes->at(i).y -= min_y;
    cubes->at(i).z -= min_z;
  }

  xsize = (int)(max_x - min_x) + 5;
  ysize = (int)(max_y - min_y) + 5;
  zsize = (int)((max_z - min_z) + 5);
  cout << "sizes: " << xsize << " " << ysize << " " << zsize << endl;
  ims = new vector<CvMat*>(zsize);

  if (argc > 2)
  {
    for (size_t i = 0; i < ims->size(); ++i)
    {
      stringstream fname;
      fname << argv[2] << "-" << i << ".png";
      if (ims->at(i) == NULL)
      {
        ims->at(i) = DrawLayer(i);
      }
      cvSaveImage(fname.str().c_str(), ims->at(i));
    }
    delete ims;
    return 0;
  }

  SelectorCallBack(0);
  cvNamedWindow(windowname, 1);
  cvCreateTrackbar("Z", windowname, &zt, zsize - 1, SelectorCallBack);
  cvWaitKey();

  delete ims;
}
