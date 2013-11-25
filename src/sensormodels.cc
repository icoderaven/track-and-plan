/*
 * sensormodels.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation; see sensormodels.hh.
 */

#include <cassert>
#include <cmath>
#include <iostream>
#include <boost/math/distributions/chi_squared.hpp>
#include "colorspaces.hh"
#include "sensormodels.hh"

using namespace std;

namespace textured_localization {

L2SensorModel::L2SensorModel(double multiplier, bool smart_about_colors,
		bool use_features) :
		SensorModel(), _variance(0.0), _smart(smart_about_colors), _use_features(
				use_features) {
	IplImage* black = cvCreateImage(cvSize(1, 1), IPL_DEPTH_8U, 3);
	IplImage* white = cvCloneImage(black);
	cvSet(white, cvScalarAll(255.0));
	cvSet(black, cvScalarAll(0.0));
	_variance = cvNorm(black, white, CV_L2) * multiplier;
	cvReleaseImage(&black);
	cvReleaseImage(&white);
}

L2SensorModel::~L2SensorModel() {
	// Nada!
}

double L2SensorModel::variance() const {
	return _variance;
}

double L2SensorModel::operator()(IplImage* reference, IplImage* reading) {
	/*
	 * First step; generate the mask.
	 */
	IplImage* mask = NULL;
	int valid_pixels = reading->width * reading->height;
	if (_smart) {
		mask = cvCreateImage(cvGetSize(reading), IPL_DEPTH_8U, 1);
		cvSet(mask, cvScalarAll(255.0));
		for (int i = 0; i < mask->height; ++i) {
			for (int j = 0; j < mask->width; ++j) {
				CvScalar c = cvGet2D(reading, i, j);
				if (c.val[1] == 255) {
					cvSet2D(mask, i, j, cvScalarAll(0));
					valid_pixels--;
				}
			}
		}
		if (valid_pixels < 0.9 * mask->width * mask->height) {
			cvReleaseImage(&mask);
			return 0;
		}
	}
	//@US Replace this by a SIFT/SURF match score
	if (!_use_features) {
		double cvnorm = cvNorm(reference, reading, CV_L2, mask);

		cvnorm *= (reading->width * reading->height);
		cvnorm /= valid_pixels;
		double logprob = cvnorm / (2 * _variance);
		if (_smart)
			cvReleaseImage(&mask);
		return exp(-(logprob * logprob));
	} else {
		//First get SIFT features
		//-- Step 1: Detect the keypoints using SURF Detector
		double total = 0.0;
		int minHessian = 400;

		cv::SurfFeatureDetector detector(minHessian);

		std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
//		cv::Mat ref_gray, ref_read;

		IplImage* ref_gray = cvCreateImage(
				cvSize(reference->width, reference->height), IPL_DEPTH_8U, 1);
		cvCvtColor(reference, ref_gray, CV_BGR2GRAY);
		IplImage* ref_read = cvCreateImage(
				cvSize(reading->width, reading->height), IPL_DEPTH_8U, 1);
		cvCvtColor(reading, ref_read, CV_BGR2GRAY);

//		cv::cvtColor(cv::Mat(reference), ref_gray, CV_BGR2GRAY);
//		cv::cvtColor(cv::Mat(reading), ref_read, CV_BGR2GRAY);

		detector.detect(ref_gray, keypoints_1);
		detector.detect(ref_read, keypoints_2);

		//-- Step 2: Calculate descriptors (feature vectors)
		cv::SurfDescriptorExtractor extractor;

		cv::Mat descriptors_1, descriptors_2;

		extractor.compute(ref_gray, keypoints_1, descriptors_1);
		extractor.compute(ref_read, keypoints_2, descriptors_2);

		//-- Step 3: Matching descriptor vectors using FLANN matcher
		std::cout<<keypoints_2.size()<<"\n";
		if (keypoints_1.size() != 0 && keypoints_2.size() != 0) {
			cv::FlannBasedMatcher matcher;
			std::vector<cv::DMatch> matches;
			matcher.match(descriptors_1, descriptors_2, matches);

			double max_dist = 0;
			double min_dist = 100;

			//-- Quick calculation of max and min distances between keypoints
			for (int i = 0; i < descriptors_1.rows; i++) {
				double dist = matches[i].distance;
				if (dist < min_dist)
					min_dist = dist;
				if (dist > max_dist)
					max_dist = dist;

			}

			std::vector<cv::DMatch> good_matches;

			for (int i = 0; i < descriptors_1.rows; i++) {
				double dist = matches[i].distance;
				if (dist <= 2 * min_dist) {
					good_matches.push_back(matches[i]);
					total += dist * dist;
				}
			}

			// --- Step 4: Display theees image
			cv::Mat matches_img;
			cv::drawMatches(ref_gray, keypoints_1, ref_read, keypoints_2,
					good_matches, matches_img, cv::Scalar::all(-1),
					cv::Scalar::all(-1), vector<char>(),
					cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			cv::imshow("OURAWESOMEWINDOW", matches_img);
//				cv::waitKey(1);
			double avg = total / good_matches.size();
			double logprob = sqrt(avg) / (2 * _variance);
			return exp(-(logprob * logprob));
		} else {
			return 0;
		}
	}

}

L1SensorModel::L1SensorModel(int width, int height) :
		SensorModel(), _l1_max(0.0) {
	IplImage* black = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	IplImage* white = cvCloneImage(black);
	cvSet(white, cvScalarAll(255.0));
	cvSet(black, cvScalarAll(0.0));
	_l1_max = cvNorm(black, white, CV_L1);
	cvReleaseImage(&black);
	cvReleaseImage(&white);
}

L1SensorModel::~L1SensorModel() {
	// Do nothing.
}

double L1SensorModel::operator()(IplImage* reference, IplImage* reading) {
	return 1.0 - (cvNorm(reference, reading, CV_L1) / _l1_max);
}

L2HueSensorModel::L2HueSensorModel(double variance) :
		SensorModel(), _variance(variance) {
}

L2HueSensorModel::~L2HueSensorModel() {
	// Still nothing to do.
}

double L2HueSensorModel::operator()(IplImage* reference, IplImage* reading) {
	double total = 0.0;
	assert(reference->height == reading->height);
	assert(reference->width == reading->width);

	for (int i = 0; i < reference->height; ++i) {
		for (int j = 0; j < reference->width; ++j) {
			double d = HueError(cvGet2D(reference, i, j),
					cvGet2D(reading, i, j));
			total += d * d;
		}
	}

	double logprob = sqrt(total) / (2 * _variance);
	return exp(-(logprob * logprob));
}

double L2HueSensorModel::HueError(const CvScalar& a, const CvScalar& b) {
	for (int i = 0; i < 3; ++i) {
		assert(a.val[i] >= 0.0);
		assert(a.val[i] <= 255.0);
		assert(b.val[i] >= 0.0);
		assert(b.val[i] <= 255.0);
	}
	double ahue, asat, aval, bhue, bsat, bval;
	rgb_to_hsv((int) a.val[2], (int) a.val[1], (int) a.val[0], ahue, asat,
			aval);
	rgb_to_hsv((int) b.val[2], (int) b.val[1], (int) b.val[0], bhue, bsat,
			bval);
	assert(ahue <= 360.0);
	assert(ahue >= 0.0);
	assert(bhue <= 360.0);
	assert(bhue >= 0.0);

	double diff = fabs(ahue - bhue);
	if (diff > 180.0)
		return 360.0 - diff;
	else
		return diff;
}

L1HueSensorModel::L1HueSensorModel(int width, int height) :
		SensorModel(), _l1_max(180.0 * width * height) {
// The L_1 is measured in degrees.
}

L1HueSensorModel::~L1HueSensorModel() {
}

double L1HueSensorModel::operator()(IplImage* reference, IplImage* reading) {
	double total = 0.0;
	for (int i = 0; i < reference->height; ++i) {
		for (int j = 0; j < reference->width; ++j) {
			double d = L2HueSensorModel::HueError(cvGet2D(reference, i, j),
					cvGet2D(reading, i, j));
			total += d;
		}
	}

	return 1.0 - (total / _l1_max);
}

GrayScaleL2SensorModel::GrayScaleL2SensorModel(double multiplier, bool fixmean) :
		SensorModel(), _variance(255.0 * 255.0 * multiplier), _fixmean(fixmean) {
}

GrayScaleL2SensorModel::~GrayScaleL2SensorModel() {
// Nothin'
}

double GrayScaleL2SensorModel::operator()(IplImage* reference,
		IplImage* reading) {
	IplImage* ref_gray = cvCreateImage(
			cvSize(reference->width, reference->height), IPL_DEPTH_8U, 1);
	IplImage* read_gray = cvCreateImage(
			cvSize(reference->width, reference->height), IPL_DEPTH_8U, 1);

	cvCvtColor(reference, ref_gray, CV_BGR2GRAY);
	cvCvtColor(reading, read_gray, CV_BGR2GRAY);

	if (_fixmean)
		read_gray = MatchMean(ref_gray, read_gray, true);

	double norm = cvNorm(ref_gray, read_gray, CV_L2);
	cvReleaseImage(&ref_gray);
	cvReleaseImage(&read_gray);

	double logprob = norm / (2 * _variance);
	return exp(-(logprob * logprob));
}

IplImage* GrayScaleL2SensorModel::MatchMean(IplImage* target, IplImage* im,
		bool del) {
	assert(target->nChannels == 1);
	assert(im->nChannels == 1);

	double target_mean = cvAvg(target).val[0];
	double current_mean = cvAvg(im).val[0];

	IplImage* res = cvCloneImage(im);
	cvAddS(im, cvScalar(current_mean - target_mean), res);
//cvAddS(im, cvScalar(target_mean - current_mean), res);

	if (del)
		cvReleaseImage(&im);
	return res;
}

ChiSquaredSensorModel::ChiSquaredSensorModel(double multiplier) :
		_multiplier(multiplier) {
}

double ChiSquaredSensorModel::operator()(IplImage* reference,
		IplImage* reading) {
	using namespace boost::math;
	IplImage* mask = NULL;
	int k = reading->width * reading->height;
	mask = cvCreateImage(cvGetSize(reading), IPL_DEPTH_8U, 1);
	cvSet(mask, cvScalarAll(255.0));
	for (int i = 0; i < mask->height; ++i) {
		for (int j = 0; j < mask->width; ++j) {
			CvScalar c = cvGet2D(reading, i, j);
			if (c.val[1] == 255) {
				cvSet2D(mask, i, j, cvScalarAll(0));
				k--;
			}
		}
	}

	k *= 3;  // Three channels.
	double norm = cvNorm(reference, reading, CV_L2, mask);
	cvReleaseImage(&mask);
// Make them "unit-variance-ish".
	norm *= _multiplier;
	norm *= norm;

	if (k == 0)
		return 0;

	chi_squared chi(k);

	return pdf(chi, norm);
}

MutualInformationSensorModel::MutualInformationSensorModel(int bucketsize) :
		_bucketsize(bucketsize) {
}

double MutualInformationSensorModel::operator()(IplImage* reference,
		IplImage* reading) {
// Don't be stupid.
	if ((reference->height != reading->height)
			|| (reference->width != reading->width)) {
		cout << "Size or type mismatch in MutualInformationSensorModel!"
				<< endl;
		exit(1);
	}

	int buckets = 256 / _bucketsize;
	double histogram[buckets * buckets];  // Kyle Singler approves this message.
	double ref_hist[buckets];
	double rea_hist[buckets];  // And again, he approves.

// Zero is the loneliest number. AND THE BEST.
	for (int i = 0; i < buckets * buckets; ++i)
		histogram[i] = 0.0;
	for (int i = 0; i < buckets; ++i) {
		ref_hist[i] = 0.0;
		rea_hist[i] = 0.0;
	}

	for (int i = 0; i < reading->height; ++i) {
		for (int j = 0; j < reading->width; ++j) {
			CvScalar ref = cvGet2D(reference, i, j);
			CvScalar rea = cvGet2D(reading, i, j);
			for (int k = 0; k < 3; ++k) {
				int ri = (int) ref.val[k];
				int rj = (int) rea.val[k];
				//cout << ri << " " << rj << endl;
				// 2D histogram
				//      cout << "idx: " << ((ri % buckets) * buckets) + (rj % buckets) << endl;
				histogram[((ri % buckets) * buckets) + (rj % buckets)] += 1.0;
				ref_hist[ri % buckets] += 1.0;
				rea_hist[rj % buckets] += 1.0;
			}
		}
	}

	double e1 = Entropy(ref_hist, buckets);
	double e2 = Entropy(rea_hist, buckets);
	double joint = Entropy(histogram, buckets * buckets);
	cout << "H(ref): " << e1 << " H(render): " << e2 << " Joint: " << joint
			<< endl;
	return e1 + e2 - joint;
}

NormalizedL2SensorModel::NormalizedL2SensorModel(double multiplier) :
		_l2(multiplier, false) {
}

double NormalizedL2SensorModel::operator()(IplImage* reference,
		IplImage* reading) {
// First, normalize the images.
	IplImage* ref_norm = Normalize(reference, false);
	IplImage* rea_norm = Normalize(reading, false);

// Make sure we've done the right thing.
	pair<double, double> ref_stats = MeanAndStddev(ref_norm);
	pair<double, double> rea_stats = MeanAndStddev(rea_norm);
//  cout << "Means and Sigmas: Ref: " 
//       << ref_stats.first << " " << ref_stats.second 
//       << " Rea: "
//       << rea_stats.first << " " << rea_stats.second 
//       << endl;

	double result = _l2(ref_norm, rea_norm);
// Don't waste too much memory...
	cvReleaseImage(&ref_norm);
	cvReleaseImage(&rea_norm);

	return result;
}

PerChannelNormalizedL2SensorModel::PerChannelNormalizedL2SensorModel(
		double multiplier, bool use_features) :
		_l2(multiplier, false, use_features), _use_features(use_features) {
}

double PerChannelNormalizedL2SensorModel::operator()(IplImage* reference,
		IplImage* reading) {
	double result;
	if (!_use_features) {
		IplImage* ref_norm = NormalizePerChannel(reference, false);
		IplImage* rea_norm = NormalizePerChannel(reading, false);

		result = _l2(ref_norm, rea_norm);
		cvReleaseImage(&ref_norm);
		cvReleaseImage(&rea_norm);
	} else {
		result = _l2(reference, reading);
	}
	return result;
}

} // end of namespace

