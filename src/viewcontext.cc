/*
 * viewcontext.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See viewcontext.hh.
 */

#include <vector>
#include <sstream>
#include <cv.h>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#define TNT_BOUNDS_CHECK
#include "TNT/tnt.h"
#include "extended_tnt.hh"
#include "utilities.hh"
#include "barecell.hh"
#include "scale.hh"
#include "viewcontext.hh"

using namespace std;
using namespace TNT;

// Are we using the riegl's version, or Monika's version? If not RIEGL, then
// Monika.
#undef RIEGL
//#define RIEGL

namespace textured_localization
{

/*
 * OpenGL's (really, GLUT's) drawing command requires a plain-old C function.
 * We'd much rather use a member function, so that our various parameters
 * aren't globals. All this plain-old function does is pass through to
 * ViewContext's Draw function.
 */
void PlainOldDraw()
{
  ViewContext::Get().Draw();
}

/*
 * Keyboard control.
 */
void PlainOldKeyboard(unsigned char key, int x, int y)
{
  // quit
  if (key == 'q')
  {
    ViewContext::Get().request_quit();
  }
  // rectilinear movement
  else if (key == 'w')
  {
    ViewContext::Get().Translate(1);
  }
  else if (key == 'W')
  {
    ViewContext::Get().Translate(10);
  }
  else if (key == 's')
  {
    ViewContext::Get().Translate(-1);
  }
  else if (key == 'S')
  {
    ViewContext::Get().Translate(-10);
  }
  else if (key == 'a')
  {
    ViewContext::Get().TranslateSideways(1);
  }
  else if (key == 'A')
  {
    ViewContext::Get().TranslateSideways(10);
  }
  else if (key == 'd')
  {
    ViewContext::Get().TranslateSideways(-1);
  }
  else if (key == 'D')
  {
    ViewContext::Get().TranslateSideways(-10);
  }
  // left-right rotation.
  else if (key == 'j')  // Left swing
  {
    ViewContext::Get().Rotate(radians(1));
  }
  else if (key == 'J')  // Left swing
  {
    ViewContext::Get().Rotate(radians(10));
  }
  else if (key == 'l')  // Right swing
  {
    ViewContext::Get().Rotate(radians(-1));
  }
  else if (key == 'L')  // Right swing
  {
    ViewContext::Get().Rotate(radians(-10));
  }
  // up-down rotation
  else if (key == 'i')
  {
    ViewContext::Get().Tilt(radians(-1));
  }
  else if (key == 'I')
  {
    ViewContext::Get().Tilt(radians(-10));
  }
  else if (key == 'k')
  {
    ViewContext::Get().Tilt(radians(1));
  }
  else if (key == 'K')
  {
    ViewContext::Get().Tilt(radians(10));
  }
  // Up & down
  else if (key == 't')
  {
    ViewContext::Get().GenericTranslate(PlusZ<double>(), 1);
  }
  else if (key == 'T')
  {
    ViewContext::Get().GenericTranslate(PlusZ<double>(), 10);
  }
  else if (key == 'g')
  {
    ViewContext::Get().GenericTranslate(PlusZ<double>(), -1);
  }
  else if (key == 'G')
  {
    ViewContext::Get().GenericTranslate(PlusZ<double>(), -10);
  }
  // Sometimes, rotation gets goofy, and it's nice to reset it.
  else if (key == 'r')
  {
    Pose p = ViewContext::Get().pose();
    p.set_theta(0);
    ViewContext::Get().SetPose(p);
  }
}

ViewContext& ViewContext::Get()
{
  static ViewContext vc;
  return vc;
}

void ViewContext::Init(KVParser& config,
                       int argc, char* argv[],
                       const vector<BareCell>& cells)
{
  double scalefactor = 1.0;
  if (config["scalefactor"] != "")
    scalefactor = atof(config["scalefactor"].c_str());

  Get()._width = (int)ceil(atoi(config["Ny"].c_str()) * scalefactor);
  Get()._height = (int)ceil(atoi(config["Nx"].c_str()) * scalefactor);

  if (config["robot_cam"] != "")
  {
    /*
     * Here's the deal, and it's complicated. In Monika's calibration code,
     * the coordinate frame is +z for forward, +y for down, and +x for right
     * (which is chosen to be analogous to image coordinates). Plus, distances
     * are in MM, not meters or units. 
     *
     * So here's the plan. We have a matrix (called M, below) which converts
     * from my coordinate frame to Monika's. Then, there is a camera matrix
     * (what we read in) that converts from camera to robot, in Monika's
     * coordinates. Then, there's _robot, which converts from robot to world,
     * in my coordinates.
     *
     * So, we want (after doing unit conversions on the read-in camera
     * matrix):
     *
     * R * M * C * M^-1 * v (for some camera-facing-vector v).
     *
     * We store M * C * M^-1 in _camera, because that never changes.
     *
     * Got that? Egad.
     */
    stringstream camstream(config["robot_cam"]);
    camstream >> _camera;
    assert(_camera.dim1() == 4 && _camera.dim2() == 4);

    // Convert the translation to units:
    for (int i = 0;  i < 3 ; ++i)
    {
#ifdef RIEGL
      _camera[i][3] *= (SCALE);
#else
      _camera[i][3] *= (SCALE / 1000.0);  // MM conversion, re: Monika.
#endif
    }

    // Also, we don't believe the y-value (up-down, in Monika-world).
#ifndef RIEGL
    _camera[1][3] = 0.0;
#endif

    /*
     * Now, the coordinate-frame trickery. Consider the three unit vectors,
     * plus the origin, in Monika's frame; writing each as a column vector
     * and stacking gives us the identity matrix I. Written in my basis, this
     * is the matrix M:
     *  |  0  0 1 0 |
     *  | -1  0 0 0 |
     *  |  0 -1 0 0 |
     *  |  0  0 0 1 |
     *
     *  so TI = M => M is our transformation.
     */
    // For Monika's coordinate frame
    Array2D<double> M(4, 4, 0.0);
#ifdef RIEGL
    //Riegl version
    M[0][2] = 1.0;
    M[1][1] = -1.0;
    M[2][0] = 1.0;
    M[3][3] = 1.0;
#else
    //Monika's version.
    M[0][2] = 1.0;
    M[1][0] = -1.0;
    M[2][1] = -1.0;
    M[3][3] = 1.0;
#endif

    /*
     * My eight-degree downward twist. Let's see how that helps.
     */
#ifndef RIEGL
    //_camera = matmult(_camera, RotX<double>(radians(-8)));
#endif

#ifdef RIEGL
    _camera = matmult(_camera, M);
#else
    _camera = matmult(_camera, invert(M));  // _camera = C * M^-1
    _camera = matmult(M, _camera);  // _camera = M * C * M^-1
    _camera = matmult(_camera, RotY<double>(radians(6)));
    stringstream s;
    s << "4 4 0.9786 0.2059 -0.0030 0 -0.2059 0.9781 -0.0289 0 -0.0030    0.0289    0.9996         0          0         0         0    1.0000";
    Array2D<double> magicRotation;
    s >> magicRotation;
    _camera = matmult(_camera, magicRotation);
    //_camera = matmult(_camera, RotX<double>(radians(-5)));
    _camera = matmult(_camera, RotX<double>(radians(-7)));
#endif
  }

  // Copy in the cells.
  Get()._cells.clear();
  foreach(BareCell c, cells)
    Get()._cells.push_back(c);

  StartGLUTUp(argc, argv, config);
  // Make sure the window starts correctly.
  IplImage* throwaway = Render();
  cvReleaseImage(&throwaway);
}

ViewContext::~ViewContext()
{
  glutDestroyWindow(_window);
}

void ViewContext::Draw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  /*
   * To figure out where we are, and where we're looking, we generate the
   * origin, +X, and +Z, and see where they end up.
   */
  Array2D<double> O = Origin<double>();
  Array2D<double> F = PlusX<double>();
  Array2D<double> Z = PlusZ<double>();

  Array2D<double> TO = matmult(matmult(_robot, _camera), O);
  Array2D<double> TF = matmult(matmult(_robot, _camera), F);
  Array2D<double> TZ = matmult(matmult(_robot, _camera), Z);
  TZ -= TO;
  
  gluLookAt(TO[0][0], TO[1][0], TO[2][0], 
            TF[0][0], TF[1][0], TF[2][0],
            TZ[0][0], TZ[1][0], TZ[2][0]);

  // The vector of the camera's facing angle.
  double fvx = TF[0][0] - TO[0][0];
  double fvy = TF[1][0] - TO[1][0];
  double fvz = TF[2][0] - TO[2][0];
  double fvnorm = norm(fvx, fvy, fvz);
  fvx /= fvnorm;
  fvy /= fvnorm;
  fvz /= fvnorm;

  for (vector<BareCell>::iterator b = _cells.begin(); b != _cells.end(); ++b)
  {
    /*
     * For each cell, we need to construct the vector from our current
     * position to the center of that cell. This is the "facing direction" to
     * that cell; the dot-product of the facing direction with the various
     * surface normals give us visibility. Furthermore, the dot-product with
     * the facing direction of the camera can let us drop entire cells.
     */
    double vx = b->x() + 0.5 - TO[0][0];
    double vy = b->y() + 0.5 - TO[1][0];
    double vz = b->z() + 0.5 - TO[2][0];
    double vnorm = norm(vx, vy, vz);
    vx /= vnorm;
    vy /= vnorm;
    vz /= vnorm;

#define CUTOFF (0.8)
//#define CUTOFF (0.93969)

    if ((fvx * vx) + (fvy * vy) + (fvz * vz) < CUTOFF)
      continue;

    /*
     * The face indices are (from 3dpslam/ocnode.hh):
     * TOP 0
     * BOTTOM 1
     * LEFT 2
     * RIGHT 3
     * FRONT 4 
     * BACK 5
     */

    glColor3f(b->r(), b->g(), b->b());
    glBegin(GL_QUADS);
      // Front face; normal of [-1, 0, 0].
      if (vx > 0)
      {
        glVertex3f(b->x(),     b->y()    , b->z());
        glVertex3f(b->x(),     b->y()    , b->z() + 1);
        glVertex3f(b->x(),     b->y() + 1, b->z() + 1);
        glVertex3f(b->x(),     b->y() + 1, b->z());
      }
      else // Back face; normal of [1, 0, 0].
      {
        glVertex3f(b->x() + 1, b->y()    , b->z());
        glVertex3f(b->x() + 1, b->y() + 1, b->z());
        glVertex3f(b->x() + 1, b->y() + 1, b->z() + 1);
        glVertex3f(b->x() + 1, b->y()    , b->z() + 1);
      }

      // Left face; normal of [0, 1, 0].
      if (vy < 0)
      {
        glVertex3f(b->x(),     b->y() + 1, b->z());
        glVertex3f(b->x(),     b->y() + 1, b->z() + 1);
        glVertex3f(b->x() + 1, b->y() + 1, b->z() + 1);
        glVertex3f(b->x() + 1, b->y() + 1, b->z());
      }
      else // Right face; normal of [0, -1, 0].
      {
        glVertex3f(b->x(),     b->y(),     b->z());
        glVertex3f(b->x() + 1, b->y(),     b->z());
        glVertex3f(b->x() + 1, b->y(),     b->z() + 1);
        glVertex3f(b->x(),     b->y(),     b->z() + 1);
      }

      // Top face; normal of [0, 0, 1].
      if (vz < 0)
      {
        glVertex3f(b->x(),     b->y()    , b->z() + 1);
        glVertex3f(b->x(),     b->y() + 1, b->z() + 1);
        glVertex3f(b->x() + 1, b->y() + 1, b->z() + 1);
        glVertex3f(b->x() + 1, b->y()    , b->z() + 1);
      }
      else // Bottom face; normal of [0, 0, -1].
      {
        glColor3f(b->r(1), b->g(1), b->b(1));
        glVertex3f(b->x(),     b->y()    , b->z());
        glVertex3f(b->x(),     b->y() + 1, b->z());
        glVertex3f(b->x() + 1, b->y() + 1, b->z());
        glVertex3f(b->x() + 1, b->y()    , b->z());
      }
    glEnd();
  }
  //cout << "Drew " << drawn << " faces." << endl;
  glutSwapBuffers();
}

IplImage* ViewContext::Render()
{
  unsigned char* data = new unsigned char[3 * _width * _height];
  //memset(data, 128, 3 * _width * _height);
  glutPostRedisplay();
  glutMainLoopEvent();
  glReadPixels(0, 0, _width, _height, GL_BGR, GL_UNSIGNED_BYTE, data);

  IplImage* result = cvCreateImage(cvSize(_width, _height), IPL_DEPTH_8U, 3);

  // The slow way.
  int i = 0;
  for (int r = 0 ; r < _height ; ++r)
  {
    for (int c = 0 ; c < _width ; ++c)
    {
      cvSet2D(result, r, c, cvScalar(data[i], data[i+1], data[i+2]));
      i += 3;
    }
  }
  delete[] data;

  cvFlip(result, 0);
  return result;
}

void ViewContext::EnableKeyboard()
{
  glutKeyboardFunc(PlainOldKeyboard);
}

void ViewContext::DisableKeyboard()
{
  glutKeyboardFunc(NULL);
}

vector<Pose> ViewContext::Poses()
{
  return _poses;
}

void ViewContext::AddPose(Pose p)
{
  _poses.push_back(p);
}

bool ViewContext::quit_requested() const
{
  return _quit_requested;
}

void ViewContext::request_quit()
{
  _quit_requested = true;
}

int ViewContext::height() const { return _height; }
int ViewContext::width() const { return _width; }

Array2D<double> ViewContext::Robot() const
{
  return _robot;
}

Array2D<double> ViewContext::Camera() const
{
  return _camera;
}

Pose ViewContext::pose() const
{
  // To do this, we transform the origin. 
  Array2D<double> O = Origin<double>();
  Array2D<double> X = PlusX<double>();
  O = matmult(_robot, O);
  X = matmult(_robot, X);
  X = X - O;
  return Pose(O[0][0], O[1][0], O[2][0], atan2(X[1][0], X[0][0]));
}

void ViewContext::SetPose(const Pose& p)
{
  // We construct the matrix thusly. First, generate the rotation:
  Array2D<double> P = RotZ<double>(p.theta());
  // Next, translate it:
  P[0][3] += p.x();
  P[1][3] += p.y();
  P[2][3] += p.z();

  // And set it.
  _robot = P;

  /*
   * This means we've rotated about the robot's center, and then moved it to
   * the right place; note how much more convenient this is than doing it the
   * other way around!
   */
}

void ViewContext::Yellowize(double riegl_height)
{
  foreach(BareCell& b, _cells)
  {
    if (b.z() < pose().z() + (riegl_height * SCALE))
    {
      b.set_r(1, 1);
      b.set_g(1, 1);
      b.set_b(1, 0);
    }
    else
    {
      b.set_r(1, b.r());
      b.set_g(1, b.g());
      b.set_b(1, b.b());
    }
  }
}

void ViewContext::GenericTranslate(Array2D<double> axis, double d)
{
  /*
   * We need to construct the vector between the robot's origin (0, 0, 0, in
   * xyz) and axis.
   */
  Array2D<double> O = Origin<double>();

  Array2D<double> TO = matmult(_robot, O);
  Array2D<double> TF = matmult(_robot, axis);

  Array2D<double> T = TF - TO;
  _robot[0][3] += d * T[0][0];
  _robot[1][3] += d * T[1][0];
  _robot[2][3] += d * T[2][0];
}

void ViewContext::Translate(double d)
{
  GenericTranslate(PlusX<double>(), d);
}

void ViewContext::TranslateSideways(double d)
{
  GenericTranslate(PlusY<double>(), d);
}

void ViewContext::Rotate(double r)
{
  // This looks a bit like rotate and translate. We begin by constructing the
  // correct rotation matrix around +Z ("up"), and then put it into robot
  // coordinates (giving us rotation about +z)
  _robot = matmult(_robot, RotZ<double>(r));
}

void ViewContext::Tilt(double r)
{
  // Just like Rotate.
  _robot = matmult(_robot, RotY<double>(r));
}

void ViewContext::SetPoseFromConfig(KVParser& config, 
                                    double x0, double y0, double z0)
{
  // First step.
  Array2D<double> scanner;
  Array2D<double> camera;
  Array2D<double> spin;
  // Make sure we actually have these matrices.
  if (config["scanner"] == "" || 
      config["camera"] == ""  || 
      config["spin"] == "")
    throw string("SetPoseFromConfig: A matrix is missing!");

  stringstream scannerstream(config["scanner"]);
  stringstream camerastream(config["camera"]);
  stringstream spinstream(config["spin"]);
  scannerstream >> scanner;
  camerastream >> camera;
  spinstream >> spin;

  _robot = matmult(scanner, matmult(spin, invert(camera)));
  _robot[0][3] += x0;
  _robot[1][3] += y0;
  _robot[2][3] += z0;
}

void ViewContext::SetCamera(const Array2D<double>& cam)
{
  _camera = cam;
}

const vector<BareCell>& ViewContext::cells() const { return _cells; }

ViewContext::ViewContext()
  : _width(0), _height(0), 
    _robot(), _camera(),
    _cells(), _window(0),
    _poses(), _quit_requested(false)
{
  // We begin at (0, 0, 0), facing along +x.
  _robot = ident<double>(4);
  _camera = ident<double>(4);
}

double ViewContext::norm(double a, double b, double c)
{
  return sqrt((a*a) + (b*b) + (c*c));
}

void ViewContext::StartGLUTUp(int argc, char* argv[], KVParser& config)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  glutInitWindowSize(_width, _height);
  _window = glutCreateWindow("ViewContext");

  // Turn a bunch of stuff off.
  glutIdleFunc(PlainOldDraw);
  glutReshapeFunc(NULL);
  glutKeyboardFunc(PlainOldKeyboard);
  // Tell us how to draw a scene.
  glutDisplayFunc(PlainOldDraw);

  // Set up our basic parameters.
  // White
  //glClearColor(1.0, 1.0, 1.0, 0.0);
  // Green
  glClearColor(0.0, 1.0, 0.0, 0.0);
  glClearDepth(1.0);
  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // We need to calculate the FOV and aspect ratio. We can't use _width and
  // _height here because those have been multiplied by the scalefactor.
  double fovx =
    degrees(2 * atan(atof(config["Nx"].c_str()) / 
                     (2.0 * atof(config["fx"].c_str()))));
  //cout << "fovx: " << fovx << " fovy: " << fovy << endl;

  gluPerspective(fovx, ((double)_width)/((double)_height), 0.1, 5000.0);
  glMatrixMode(GL_MODELVIEW);
}

}  // namespace textured_localization
