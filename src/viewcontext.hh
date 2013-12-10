/*
 * viewcontext.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * We'll be wanting to render a great many images; the idea here is to
 * encapsulate the act of rendering. That way, we can render from arbitrary
 * coordinates, etc. etc. etc.
 *
 * Importantly, this is a singleton, because it keeps alive the OpenGL
 * context, etc. Therefore, you need to Init() it, and only once.
 *
 * A discussion of viewpoint geometry: OpenGL wants us to provide two things:
 * the point where we're standing, and a point to look at.
 *
 * Here's how we store this information. Let X, Y, and Z denote the (fixed)
 * world coordinate frame (which is the OpenGL frame, except rotated). Let x,
 * y, and z denote the robot's coordinate frame, where +x is the direction
 * we're looking, +y is left, and +z is up.
 *
 * We maintain the 4x4 homogeneous-coordinates transformation matrix _robot,
 * which turns coordinates in XYZ into coordinates in xyz. The place we stand
 * is then _robot * [0 0 0 1]', and the place we're looking is _robot * [1 0 0
 * 1]'.
 *
 * We repeat this process with _camera, which takes us from xyz to camera
 * coordinates (maybe x'y'z'?). The product of _robot and _camera, by the
 * power of matrices, does the whole thing at once; very convenient, that. 
 *
 * The _camera matrix is passed in in the config argument robot_cam; see the
 * implementation of Init() for details.
 */

#ifndef TEXTURED_LOCALIZATION_VIEWCONTEXT_HH_INCLUDED
#define TEXTURED_LOCALIZATION_VIEWCONTEXT_HH_INCLUDED 1

#include <vector>
#include <boost/utility.hpp>  // For noncopyable.
#include <cv.h>               // For IplImage.
#include <GL/gl.h>            // OpenGL is handy.
#include <GL/glu.h>
#include <GL/freeglut.h>
#include "TNT/tnt.h"          // Array2D<double>, et al.
#include "kvparser.hh"
#include "pose.hh"

using namespace TNT;

namespace textured_localization
{
  // Wraps ViewContext::Draw for passing into glutDisplayFunc.
  void PlainOldDraw();
  void PlainOldKeyboard(unsigned char key, int x, int y);

  class BareCell;  // See barecell.hh

  class ViewContext : boost::noncopyable
  {
    public:
      // Singleton magic.
      static ViewContext& Get();

      /*
       * It's a Singleton, which (sadly) means we have to build it in two
       * passes; the first gets you an empty, and calling this Init() function
       * makes everything work. The width and height parameters get you the
       * size of the image, argc and argv (the same ones main() got) are for
       * glutInit, and the vector of BareCells is _copied_ into the singleton. 
       *
       * This starts the viewer at (0, 0, 0, 0, 0). See SetPose if you'd
       * rather be somewhere else.
       *
       * Here are the values that config is expected to contain:
       * fx: double (focal length in the x direction)
       * fy: double (ditto, y)
       * Nx: int (number of pixels in the x direction)
       * Ny: int (ditto, y)
       * scalefactor: double [=1.0]. If set, multiply the image size by this
       * factor.
       *
       * Some values that we'll use, if config has them:
       *
       * cam_offset (4 x 1 TNT matrix, in a string): the position, in the
       * robot's coordinate frame, where the camera center is.
       *
       * cam_facing (4x1 TNT matrix, as a string): the vector along which the
       * camera is facing, still in robot coordinates.
       */
      void Init(KVParser& config,
                int argc,  char* argv[],
                const std::vector<BareCell>& cells);

      ~ViewContext();

      /*
       * The big OpenGL moment. Render a scene (to the framebuffer). See
       * Render() for getting images.
       */
      void Draw();

      /*
       * The big mover. Given the various parameters here, render an image,
       * and return it.  The caller must take responsibility for
       * cvRelease()ing the returned value, as the allocation is done
       * internally.
       */
      IplImage* Render();

      /*
       * There are two sane ways to render; one is a flythrough mode, and the
       * other is automated. These let us disable keyboard input, so a hapless
       * user doesn't break things.
       */
      void EnableKeyboard();
      void DisableKeyboard();

      /*
       * When the keyboard is enabled, each keyboard step adds a pose to an
       * internal list of poses occupied. This lets you get at that list.
       */
      std::vector<Pose> Poses();
      void AddPose(Pose p);

      /*
       * If you've put it in keyboard mode, you'll need to call Render() in a
       * loop; this flag lets you determine if the user has typed 'q', asking
       * for the keyboard to quit.
       */
      bool quit_requested() const;
      void request_quit();

      /*
       * Simple getters.
       */
      int height() const;
      int width() const;
      Array2D<double> Robot() const;  // The transformation matrix.
      Array2D<double> Camera() const;  // The other transformation matrix.

      Pose pose() const;  // Compute a Pose object, and return that.

      // Clobber _robot, and re-construct it according to the given pose,
      // which is expressed in XYZ (world) coordinates.
      void SetPose(const Pose& p);

      // Set the bottoms of cells that are above us, but below the Riegl, to a
      // sentinel color. (Make sure you've got your z-pose set right first!)
      // riegl_height is in meters.
      void Yellowize(double riegl_height);

      // Move the robot d units along the provided axis.
      void GenericTranslate(Array2D<double> axis, double d);
      // Move the robot along +x (the robot's "forward") by d, in units.
      void Translate(double d);
      
      // Move the robot along +x (the robot's "sideways"), by d, in units.
      void TranslateSideways(double d); 

      // Rotate the robot around +z (the robot's "up") by r, in radians.
      void Rotate(double r);

      // Rotate the robot around +y (the robot's "left") by r radians.
      void Tilt(double r);

      void SetPoseFromConfig(KVParser& config,
                             double x0, double y0, double z0);

      /*
       * Unlike _robot, it makes sense to be able to set this monolithically
       * (because it's not going to change during a run).
       */
      void SetCamera(const Array2D<double>& cam);

      /*
       * A getter can't hurt, right?
       */
      const std::vector<BareCell>& cells() const;

      //@Us : Create a function to return corresponding 3D points from requested 2D points
      void Get3Dfrom2D(std::vector<cv::Point2f> in_pts, std::vector<cv::Point3f> &out_pts);

    private:
      ViewContext(); // "Lots of zeros!"

      // Handy thing.
      static double norm(double a, double b, double c);

      /*
       * How big of a camera are we faking?
       */
      int _width;
      int _height;

      /*
       * The matrix that transforms XYZ (world) coordinates to xyz (robot)
       * coordinates.
       */
      Array2D<double> _robot;

      /*
       * The matrix that transforms from xyz (robot) coordinates to camera
       * coordinates.
       */
      Array2D<double> _camera;

      /*
       * The cells we're responsible for drawing. 
       */
      std::vector<BareCell> _cells;

      /*
       * This object owns, oddly enough, a FreeGLUT noun. (Context? Instance?
       * A noun, anyway). It specifically depends on FreeGLUT, because that
       * lets us break GlutMainLoop() into pieces.
       */
      // We need arg{c, v} for glutInit().
      void StartGLUTUp(int argc, char* argv[], KVParser& config);
      // Finally, the ID of the window we own.
      int _window;

      /*
       * Various extra things.
       */
      std::vector<Pose> _poses;
      bool _quit_requested;
  };
}

#endif
