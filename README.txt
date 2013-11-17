README.txt
Code for the paper "Textured Occupancy Grids for Monocular Localization
Without Features", in ICRA 2011, by Julian Mason, Susanna Ricco, and Ronald
Parr.
---------------------------------------------------------------------------

* Getting started:

The URL for all of this is
<http://www.cs.duke.edu/~parr/textured-localization>. You'll need the code
download (which you already have, as you are reading this), and the data
download.

You should start by reading the paper; that will make everything that follows
make more sense.


* Installation instructions:

This code was written and run in Ubuntu linux; any other linux will probably
work. Some of the code (but not the important parts) will compile in Mac OS X;
however, the localization algorithm itself will not. (See the discussion of
OpenGL, below.)

On a brand-new Ubuntu 10.10 install (as of March 2011), the following packages
suffice:

build-essential
scons
libcv-dev
libhighgui-dev
libcvaux-dev
libboost-all-dev
freeglut3-dev    # FreeGLUT specifically, in fact. Not some other GLUT.
exuberant-ctags

Once all of these packages are installed, cd to the directory containing the
SConstruct file (the same directory that contains this README), and run

  $ scons

which will build the code, and install the results into the bin directory.


* Running the code:

The important executable is bin/localization; the command line you almost
certainly want is:

  $ bin/localization <map> confguration.txt <dir> <particle count> 2 true

Where the <map> and <dir> parameters are the map you're using and the log
you're using, respectively. These can be had in the data download at
<http://www.cs.duke.edu/~parr/textured-localization>. Note that the "log" is
actually the name of a _directory_, not a file.

confguration.txt stores a bunch of useful configuration parameters; you
probably don't want to touch it.

The '2', by itself, gives the index of the camera you're using. Our robot has
six webcams; #2 is the one used in the paper. Some of the runs also have
webcam data from the other cameras; some don't. Picking a different index
(when you have the data) will cause the robot to localize against different
images; however, because the coordinate transformation between robot-center
and camera-center will be wrong, you'll likely get strange behavior.

The 'true', by itself, puts the program into tracking (rather than global
localiaztion) mode. The word 'true' isn't special; in fact, this just checks
the value of argc, so anything here will give tracking, and nothing here gives
global localization.

Line 272 of localization.cc includes the manually-determined initial locations
for several of the different runs; if you're doing global localization, of
course, this will be ignored.

You may also be interested in the viewmapslices program, which provides a
convenient way of visualizing our maps. 

Supposing you've downloaded the datasets into ~/data; then, the following
command will run tracking, in the hallway dataset, on the original hallway
run: 

  $ bin/localization ~/data/hallway.map configuration.txt ~/data/HALLWAY 50 2 true


* OpenGL:
We use OpenGL to render views of our textured occupancy grid; as a result,
your performance will be HIGHLY dependent on how well OpenGL works on your
machine. One interesting thing we've observed: on some machines, if the OpenGL
widow itself is entirely covered up, rendering becomes a NOP (which means that
when we read the bytes from the framebuffer, they haven't changed). Make sure
your window stays (at least partially) visible! (This problem can be easily
diagnosed: rendering starts to run much, much faster.)

Getting OpenGL working in linux remains a somewhat spotty proposition. If
you're using a brand-name GPU, Ubuntu will probably Just Work. Your mileage
may vary.


* Other Implementation Details
Implementation-wise, you're probably most interested in viewcontext.{hh,cc},
as that's where the real rendering and whatnot goes on. Your other major
points of interest are sensormodel.{hh, cc}, and sensormodels.{hh,cc}. (Yes,
having both "sensormodel" and "sensormodels" was a bad idea. Too late now!)

* Questions:
If you have questions, you should contact Julian Mason:
<http://www.cs.duke.edu/~mac>
