# SConstruct
# Hey hierarchical builds!
# Mac Mason <mac@cs.duke.edu>
# Provided under the terms of the included LICENSE.txt
# http://www.cs.duke.edu/~parr/textured-localization
SetOption('num_jobs', 1)

def env_with_deps(cxx_header_deps, lib_deps, env_to_copy):
  """
  Build an environment, with various dependencies. Pretty handy.
  Start by Clone()ing env_to_copy, so you can accumulate stuff.
  """
  import os
  exit = False
  E = env_to_copy.Clone()

  conf = Configure(E)
  for header in cxx_header_deps:
    if not conf.CheckCXXHeader(header):
      print "****"
      print "* Need header file '%s'!"  % header
      print "****"
      exit = True

  for lib in lib_deps:
    if not conf.CheckLib(lib):
      print "****"
      print "* Need library '%s'!" % lib
      print "****"
      exit = True
  conf.Finish()

  if exit:
    Exit(1)
  else:
    return E

def tags(sourcefiles):
  """
  Make ctags build you a tagfile.
  """

Export('env_with_deps')
SConscript(['src/SConscript'], variant_dir="build")
