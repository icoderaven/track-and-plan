#!/usr/bin/env python
"""
logfile.py
Mac Mason <mac@cs.duke.edu>
Provided under the terms of the included LICENSE.txt.
http://www.cs.duke.edu/~parr/textured-localizatio

A class representing one of our logfiles. May or may not be particularly,
y'know, smart, or anything.

But, saner than re-damn-doing-it-by-hand every time.
"""
import os
import shutil

class LogEntry:
  """
  One entry in the log. Contains:
  self.action, an (x, y, theta) triple.
  self.cameras, a list of image filenames.
  self.readings, a list of (r, phi, theta) triples.

  You probably need the directory name to add to self.cameras.
  """
  def __init__(self, action, cameras, readings):
    self.action = action
    self.cameras = [os.path.split(cam)[1] for cam in cameras]
    self.readings = readings

  def __str__(self):
    actstr = "Action: %f %f %f" % self.action
    camstr = "Camera: %d %s" % (len(self.cameras), ' '.join(self.cameras))
    def trip(RPT):
      return "%f %f %f" % RPT
    lasstr = "Laser: %s" % ' '.join(map(trip, self.readings))
    return '\n'.join([actstr, camstr, lasstr])

class Log:
  """
  Represent an entire log.
  """
  def __init__(self, dir):
    """
    Send in the path to the directory which stores the log; this takes care of
    the rest.
    """
    self.dir = dir
    if self.dir[-1] != '/':
      self.dir += '/'

    if not os.path.exists(self.dir):
      raise ValueError("Directory %s does not exist!"% self.dir)
    self.logname = self.dir + os.path.basename(self.dir[:-1]) + '.log'
    if not os.path.exists(self.logname):
      raise ValueError("Log file %s does not exist!" % self.logname)

    self.file = file(self.logname, 'r')

  def __del__(self):
    self.file.close()

  def __iter__(self):
    """
    Provide the log entries, (as LogEntry objects), one at a time.
    """
    while True:
      al = self.file.readline()
      cl = self.file.readline()
      ll = self.file.readline()
      if al == '' or cl == '' or ll == '':
        if al == '' and (cl != '' or ll != ''):
          print "CORRUPT LOGFILE!"
        break

      action = tuple(map(float, al.strip().split()[1:]))
      cams = cl.strip().split()[2:]
      las = map(float, ll.strip().split()[1:])
      rs = las[::3]
      ps = las[1::3]
      ts = las[2::3]
      yield LogEntry(action, cams, zip(rs, ps, ts))

class NewLog:
  """
  For creating new logs.
  """
  def __init__(self, dir, force = False):
    self.dir = dir
    if self.dir[-1] != '/':
      self.dir += '/'
    if os.path.exists(self.dir) and not force:
      raise ValueError("Directory %s exists!" % self.dir)
    elif os.path.exists(self.dir) and force:
      shutil.rmtree(self.dir)

    os.mkdir(self.dir)
    self.file = file(self.dir + os.path.basename(self.dir[:-1]) + '.log', 'w')

  def __del__(self):
    self.file.close()

  def add(self, action):
    """
    Add a LogEntry to the log.
    """
    for cam in action.cameras:
      cam = os.path.split(cam)[1]
    self.file.write(str(action))
    self.file.write("\n")

if __name__ == "__main__":
  L = Log('/home/mac/north_complete')
  for l in L:
    print l
