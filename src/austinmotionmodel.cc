/*
 * austinmotionmodel.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See austinmotionmodel.hh
 */

#include <string>
#include <vector>
#include <boost/format.hpp>
#include "random.hh"
#include "kvparser.hh"
#include "austinmotionmodel.hh"

using namespace std;

namespace textured_localization
{

AustinMotionModel::AustinMotionModel(KVParser& config)
{
  if (config["meanD_D"] == "" || 
      config["meanD_T"] == "" || 
      config["meanC_D"] == "" || 
      config["meanC_T"] == "" || 
      config["meanT_D"] == "" || 
      config["meanT_T"] == "")
  {
    throw string("AustinMotionModel is missing a mean!");
  }

  if (config["stddevD_D"] == "" || 
      config["stddevD_T"] == "" || 
      config["stddevC_D"] == "" || 
      config["stddevC_T"] == "" || 
      config["stddevT_D"] == "" || 
      config["stddevT_T"] == "")
  {
    throw string("AustinMotionModel is missing a stddev!");
  }

  if (config["min_trans_stddev"] == "" || 
      config["min_rot_stddev"] == "")
  {
    throw string("AustinMotionModel is missing a minimum!");
  }

  // You may proceed.
  _meanD_D = atof(config["meanD_D"].c_str());
  _meanD_T = atof(config["meanD_T"].c_str());
  _meanC_D = atof(config["meanC_D"].c_str());
  _meanC_T = atof(config["meanC_T"].c_str());
  _meanT_D = atof(config["meanT_D"].c_str());
  _meanT_T = atof(config["meanT_T"].c_str());

  _stddevD_D = atof(config["stddevD_D"].c_str());
  _stddevD_T = atof(config["stddevD_T"].c_str());
  _stddevC_D = atof(config["stddevC_D"].c_str());
  _stddevC_T = atof(config["stddevC_T"].c_str());
  _stddevT_D = atof(config["stddevT_D"].c_str());
  _stddevT_T = atof(config["stddevT_T"].c_str());

  _min_trans_stddev = atof(config["min_trans_stddev"].c_str());
  _min_rot_stddev = atof(config["min_rot_stddev"].c_str());
}

AustinMotionModel::~AustinMotionModel()
{
}

vector<double> AustinMotionModel::Sample(double dx, 
                                         double dy, 
                                         double dt, 
                                         double old_facing)
{
  vector<double> res;
  /*
   * This implementation is lifted from SLAM::Filter::ApplyMotionModel, in
   * ../3dpslam. Comments there are more interesting than comments here, so go
   * look at them.
   */
  double distance = sqrt((dx * dx) + (dy * dy));
  double DCenter = (distance * _meanD_D) + 
                   (dt       * _meanD_T);
  double CCenter = (distance * _meanC_D) + 
                   (dt       * _meanC_T);
  double TCenter = (distance * _meanT_D) + 
                   (dt       * _meanT_T);

  double Dstddev = fabs(distance * _stddevD_D) + fabs(dt * _stddevD_T);
  Dstddev = max(Dstddev, _min_trans_stddev);
  double Cstddev = fabs(distance * _stddevC_D) + fabs(dt * _stddevC_T);
  Cstddev = max(Cstddev, _min_trans_stddev);
  double Tstddev = fabs(distance * _stddevT_D) + fabs(dt * _stddevT_T);
  Tstddev = max(Tstddev, _min_rot_stddev);

//  cout << "Distance: " << distance << " " 
//       << format("DCenter: %f CCenter: %f TCenter: %f")
//            % DCenter % CCenter % TCenter
//       << endl
//       << format("Dstddev: %f Cstddev: %f Tstddev: %f")
//            % Dstddev % Cstddev % Tstddev
//       << endl;

  double randomD = Random::Get()->Gaussian(DCenter, Dstddev);
  double randomC = Random::Get()->Gaussian(CCenter, Cstddev);
  double randomT = Random::Get()->Gaussian(TCenter, Tstddev);

  double moveT = ((old_facing + randomT) + old_facing) / 2.0;
  double moveC = moveT + (M_PI / 2);

  double moveX = (randomD * cos(moveT)) + (randomC * cos(moveC));
  double moveY = (randomD * sin(moveT)) + (randomC * sin(moveC));

  res.push_back(moveX);
  res.push_back(moveY);
  res.push_back(randomT);

  return res;
}

} // namespace
