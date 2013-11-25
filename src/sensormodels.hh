/*
 * l2sensormodel.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Header for an L2-based sensor model.
 */

#pragma once

#include "pose.hh"
#include "utilities.hh"
#include "viewcontext.hh"
#include "sensormodel.hh"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

namespace textured_localization {
/*
 * Do the L2 kind of thing.
 */
class L2SensorModel: public SensorModel {
public:
	/*
	 * The variance used in this L2 is the L2 norm between a 1x1 white image
	 * and a 1x1 black image, times multiplier.
	 */
	L2SensorModel(double multiplier, bool smart_about_colors = true,
			bool use_features = false);
	~L2SensorModel();

	double variance() const;
	/*
	 * Base the results on the L2 norm. If we consider each pixel to be an
	 * independent reading, and believe that pixel non-matchings should pay
	 * a quadratic cost, then this is the way to go.
	 */
	double operator()(IplImage* reference, IplImage* reading);

private:
	bool _use_features;
	double _variance;
	bool _smart;

};

/* L ONE, WHAT WHAT */
class L1SensorModel: public SensorModel {
public:
	// Send in the size of the images we'll be messing with.
	L1SensorModel(int width, int height);
	~L1SensorModel();

	/*
	 * Given an WxH image (those values are provided to the constructor), we
	 * build a WxH white image, and a WxH black image, and take the L1
	 * cvNorm of those values; call the _l1_max.
	 *
	 * This function is the 1.0 - (cvNorm(reference, reference) / _l1_max).
	 */
	double operator()(IplImage* reference, IplImage* reading);

private:
	double _l1_max;
};

class L2HueSensorModel: public SensorModel {
public:
	// Set the variance as you see fit.
	L2HueSensorModel(double variance);
	~L2HueSensorModel();

	/*
	 * The assumption here is that errors in hue are normally distributed
	 * (out to 180 degrees, anyway).
	 */
	double operator()(IplImage* reference, IplImage* reading);

	// Calculate the smallest angular difference between these two colors,
	// measured in degrees.
	static double HueError(const CvScalar& a, const CvScalar& b);
private:
	double _variance;
};

class L1HueSensorModel: public SensorModel {
public:
	// Width & height, so we can work out the normalizer (see
	// L1SensorModel).
	L1HueSensorModel(int width, int height);
	~L1HueSensorModel();
	double operator()(IplImage* reference, IplImage* reading);
private:
	double _l1_max;
};

class GrayScaleL2SensorModel: public SensorModel {
public:
	GrayScaleL2SensorModel(double multiplier, bool fixmean = true);
	~GrayScaleL2SensorModel();

	/*
	 * Convert the images to grayscale, and then do L2SensorModel.
	 */
	double operator()(IplImage* reference, IplImage* reading);

	/*
	 * We think it will be useful to make the means of various images match.
	 * (This assumes that we're dealing with grayscale images). This
	 * constructs and returns a new image, whose value is im, but with a
	 * shift applied so that its mean matches the mean of target.
	 *
	 * If del is true, this also deletes im, for a handy "in-place" sort of
	 * usage.
	 *
	 * This is available as a step in operator().
	 */
	IplImage* MatchMean(IplImage* target, IplImage* im, bool del = true);
private:
	double _variance;
	bool _fixmean;
};

/*
 * This is a fun one. This wraps another sensor model, but (rather than
 * doing a single reading) it takes reading every _stepsize degrees from the
 * current (x, y) pose of ViewContext. Then, it picks the best one
 * (according to the template parameter), and returns the cost of that one.
 * Got that?
 *
 * It also needs a way of outputting the value of that best pose, so it
 * stores that, as well. Note that each call to operator() will clobber the
 * previous value of that pose.
 */
template<typename T>
class BestOrientationSensorModel: public SensorModel {
public:
	BestOrientationSensorModel(T inner_model, int stepsize) :
			_inner_model(inner_model), _stepsize(stepsize), _best_pose() {
	}

	~BestOrientationSensorModel() {
	}

	Pose best_pose() {
		return _best_pose;
	}

	/*
	 * The fun one. This is, in fact, fairly goofy, as follows. It ignores
	 * reading completely. Instead, it takes renders images by incrementing
	 * the camera's orientation; each time, it applies the templated model
	 * to that image, and returns the value of the best fit.
	 */
	double operator()(IplImage* reference, IplImage* reading) {
		Pose orig_pose = ViewContext::Get().pose();
		int degrees_changed = 0;
		double best_prob = 0.0;
		while (degrees_changed < 360) {
			ViewContext::Get().Rotate(radians(_stepsize));
			IplImage* im = ViewContext::Get().Render();
			im = Inpaint(im, cvScalar(0, 255, 0), true, 0.9);
			if (im == NULL) {
				degrees_changed += _stepsize;
				continue;
			}
			double cost = _inner_model(reference, im);
			if (cost > best_prob) {
				_best_pose = ViewContext::Get().pose();
				best_prob = cost;
			}
			cvReleaseImage(&im);
			degrees_changed += _stepsize;
		}

		// Put it back.
		ViewContext::Get().SetPose(orig_pose);
		return best_prob;
	}
private:
	T _inner_model;
	int _stepsize;
	Pose _best_pose;
};

class ChiSquaredSensorModel: public SensorModel {
public:
	ChiSquaredSensorModel(double multiplier);
	double operator()(IplImage* reference, IplImage* reading);
private:
	double _multiplier;
};

/* Fun with giant histograms! */
class MutualInformationSensorModel: public SensorModel {
private:
	int _bucketsize;
public:
	/*
	 * bucketsize tells us how many buckets there are in one dimension of
	 * the (two-d) histogram. It ranges from 1 to 255; the extremes probably
	 * aren't very interesting.
	 */
	MutualInformationSensorModel(int bucketsize);
	double operator()(IplImage* reference, IplImage* reading);

	int bucketsize() const {
		return _bucketsize;
	}
};

/*
 * Run the standard L2 sensor model, _after_ normalizing both images to have
 * mean 0 and standard deviation 1.
 */
class NormalizedL2SensorModel: public SensorModel {
private:
	L2SensorModel _l2;
public:
	// Build one; multiplier is passed through to _l2.
	NormalizedL2SensorModel(double multiplier);
	double operator()(IplImage* reference, IplImage* reading);
};

class PerChannelNormalizedL2SensorModel: public SensorModel {
private:
	L2SensorModel _l2;
	bool _use_features;
public:
	PerChannelNormalizedL2SensorModel(double multiplier, bool use_features=false);
	double operator()(IplImage* reference, IplImage* reading);
};
}
