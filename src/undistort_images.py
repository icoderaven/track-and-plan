#!/usr/bin/env python
"""
undistort_images.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

We take a collection of images from our cameras; we need to undistort them.

This, quite frankly, is possibly the worst hack I've ever written. It's a
veritable abomination. It requires that you have matlab installed, plus
forrect.mat in pwd, and a bunch of other broken-ass things. It's a mess.

Nevertheless, this is almost certaintly the easiest way to do it. How terrible
is that?
"""
import sys
import os
import subprocess
import log

if len(sys.argv) != 3:
  print "Usage: bin/undistort_images.py <log directory> <output directory>"
  sys.exit(1)

inlog  = log.Log(sys.argv[1])
outlog = log.NewLog(sys.argv[2], True) # second arg clobbers the output!

try:
  os.stat('forrect.mat')
except:
  print "Couldn't open forrect.mat. DOOM."
  sys.exit(1)

"""
Here's the algorithm. We're going to collect up a gigantic string, containing
the sequence of matlab commands which will do all the undistorting for us.
Then, we pass that to subprocess.Popen.communicate(), and BAM.

Yowza. So, we need a list of strings for storing commands.
"""
commands = ['load forrect.mat;']

for action in inlog:
  # Needed to build the new action.
  newcams = []
  for camidx, cam in enumerate(action.cameras):
    # Figure out what our old and new files will be called.
    filename = inlog.dir + cam
    (head, tail) = os.path.splitext(filename)
    nfilename = outlog.dir + os.path.splitext(cam)[0] + '_undist' + tail
    newcams.append(nfilename)
    
    # Read in the old file
    commands.append("I = double(imread('%s'));" % filename)
    commands.append('Ir = I(:, :, 1);')
    commands.append('Ig = I(:, :, 2);')
    commands.append('Ib = I(:, :, 3);')
    # Place to write the new file
    commands.append('onesize = size(I, 1);')
    commands.append('twosize = size(I, 2);')
    commands.append('I2r = 255 * ones(onesize, twosize);')
    commands.append('I2g = 255 * ones(onesize, twosize);')
    commands.append('I2b = 255 * ones(onesize, twosize);')
    commands.append('I2 = 255 * ones(size(I));')

    # Helpful variable; note the 1-indexing.
    commands.append('onim = %d;' % (camidx + 1))
    # Do the final piece.
    L = ['I2r(ind_new{onim})', 
         '= uint8(a1_{onim}', 
         '.* Ir(ind_1{onim})', 
         '+ a2_{onim}', 
         '.* Ir(ind_2{onim})', 
         '+ a3_{onim}', 
         '.* Ir(ind_3{onim})',
         '+ a4_{onim}', 
         '.* Ir(ind_4{onim}));']
    commands.append(' '.join(L))
    L = ['I2g(ind_new{onim})', 
         '= uint8(a1_{onim}', 
         '.* Ig(ind_1{onim})', 
         '+ a2_{onim}', 
         '.* Ig(ind_2{onim})', 
         '+ a3_{onim}', 
         '.* Ig(ind_3{onim})',
         '+ a4_{onim}', 
         '.* Ig(ind_4{onim}));']
    commands.append(' '.join(L))
    L = ['I2b(ind_new{onim})', 
         '= uint8(a1_{onim}', 
         '.* Ib(ind_1{onim})', 
         '+ a2_{onim}', 
         '.* Ib(ind_2{onim})', 
         '+ a3_{onim}', 
         '.* Ib(ind_3{onim})',
         '+ a4_{onim}', 
         '.* Ib(ind_4{onim}));']
    commands.append(' '.join(L))
    # And write out the file.
    commands.append('I2(:, :, 1) = I2r;')
    commands.append('I2(:, :, 2) = I2g;')
    commands.append('I2(:, :, 3) = I2b;')
    commands.append("imwrite(uint8(I2), '%s');" % nfilename)
  # This action is done; move on to the next one.
  outlog.add(log.LogEntry(action.action, newcams, action.readings)) 
  sys.stdout.write('.')
  sys.stdout.flush()

print ""
commands.append('exit')

# This should be the whole deal. Now, we need to invoke ourselves some matlab. 
P = subprocess.Popen(['matlab', '-nojvm', '-nodesktop', '-nosplash'],
                     stdin = subprocess.PIPE,
                     stdout = subprocess.PIPE,
                     stderr = subprocess.STDOUT)
(out, err) = P.communicate('\n'.join(commands))
