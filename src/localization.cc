/*
 * localization.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Do localization, with a real robot.
 */

#include <sys/time.h>
#include <cassert>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include <cv.h>
#include <highgui.h>
#include "barecell.hh"
#include "particle.hh"
#include "particle_visualizer.hh"
#include "kvparser.hh"
#include "pose.hh"
#include "utilities.hh"
#include "random.hh"
#include "austinmotionmodel.hh"
#include "kldmm.hh"
#include "sensormodels.hh"
#include "extended_tnt.hh"
#include "viewcontext.hh"
#include "scale.hh"
#include "sparsemap.hh"
#include "linear.hh"

using namespace std;
using namespace boost;
namespace bfs = boost::filesystem;
using namespace textured_localization;

void PrintUsageAndDie() {
	cout
			<< "Usage: bin/localization <map> <configuration> <logdir> <N> <camidx>"
			<< " [tracking]" << endl;
	exit(1);
}

/* An assortment of diagnostic tools. */
class ParticleSorter {
public:
	bool operator()(const Particle& a, const Particle& b) {
		return (a.weight() < b.weight());
	}
};

typedef pair<Particle, Particle> ParticlePair;
ParticlePair FindBestAndWorstParticles(vector<Particle> filter) {
	// The use of pass-by-value means we can sort things.
	sort(filter.begin(), filter.end(), ParticleSorter());
	reverse(filter.begin(), filter.end());
	assert(filter.front().weight() >= filter.back().weight());
	return make_pair(filter.front(), filter.back());
}

double CalculateNeff(const vector<Particle>& filter) {
	double res = 0.0;
	foreach(const Particle& p, filter)res += pow(p.weight(), 2);
	res = 1.0 / res;
	return res;
}

/* Go cat go. */
int main(int argc, char* argv[]) {
	bool _use_features = true;
	//@Us Read the camera calibration matrix that we found from the interwebz
	cv::FileStorage fs("camera.yaml", cv::FileStorage::READ);
	cv::Mat _cameraMatrix, _distCoeffs;
	fs["camera_matrix"] >> _cameraMatrix;
	fs["distortion_coefficients"] >> _distCoeffs;
	cout << _cameraMatrix;
	try {
		if (argc < 6 || argc > 7)
			PrintUsageAndDie();

		// Reset the seed to get different random behavior each run.
		struct timeval tv;
		gettimeofday(&tv, NULL);
		Random::Get()->ReSeed(tv.tv_usec);

		// Read in the map.
		double x0, y0, z0;
		vector<BareCell> map;
		int cells_read = BareCell::ParseMapFile(argv[1], map, x0, y0, z0);
		if (cells_read == -1) {
			cout << "Read or parse failure of " << argv[1] << endl;
			return 1;
		}

		// Open the configuration file.
		KVParser config(argv[2]);

		if (config["scalefactor"] == "") {
			cout << "You must provide a scalefactor!" << endl;
			PrintUsageAndDie();
		}
		double SCALEFACTOR = atof(config["scalefactor"].c_str());
		cout << "Scalefactor is: " << SCALEFACTOR << endl;

		// And get the motion model, too.
		AustinMotionModel MM(config);

		// Work out the name, etc. of the log directory.
		bfs::path logdir(argv[3]);
		if (!bfs::exists(logdir)) {
			cout << "Can't find the log directory!" << endl;
			PrintUsageAndDie();
		}
		// ...and the logfile.
		bfs::path logfile = logdir / (logdir.filename().string() + ".log");
		if (!bfs::exists(logfile)) {
			cout << "Can't find the logfile!" << endl;
			PrintUsageAndDie();
		}
		bfs::ifstream log(logfile);
		if (!log) {
			cout << "Can't open the logfile!" << endl;
			PrintUsageAndDie();
		}

		// Finally, let's get the particle count.
		int PARTICLE_COUNT = atoi(argv[4]);
		if (PARTICLE_COUNT == 0) {
			cout << "Need more than 0 particles!" << endl;
			PrintUsageAndDie();
		}

		int CAMERA_INDEX = atoi(argv[5]);

		/*
		 * Parse the SLAM logfile. We don't need the laser data, but we need the
		 * actions and the image filenames (we don't load the images here, because
		 * we only need one at a time).
		 */
		vector<Pose> actions;
		vector<bfs::path> image_names;
		string line;
		while (getline(log, line)) {
			stringstream ss(line);
			string keyword;
			ss >> keyword;
			if (keyword == "Action:") {
				double x, y, t;
				ss >> x >> y >> t;
				actions.push_back(Pose(SCALE * x, SCALE * y, 0, t));
			} else if (keyword == "Camera:") {
				vector<string> splut;
				trim(line);
				split(splut, line, is_any_of(" "));
				// There are eight entries per line; CAMERA_INDEX 0 is then 2.
				image_names.push_back(logdir / splut.at(CAMERA_INDEX + 2));
				if (!bfs::exists(image_names.back())) {
					cout << "Image " << image_names.back() << " does not exist!"
							<< endl;
					return 1;
				}
			}
		}

		// Get OpenGL up and running.
		ViewContext::Get().Init(config, argc, argv, map);
		// If we know where the floor is...
		if (config["robot_cam_height"] != "" && config["floor_z"] != "") {
			double cam_height = (atof(config["robot_cam_height"].c_str())
					* SCALE) + atof(config["floor_z"].c_str());

			z0 = cam_height;
		}

		// Do we do color conversion?
		Array2D<double> colormatrix = TNT::ident<double>(4);
		if (config["color_conversion"] != "") {
			stringstream ss(config["color_conversion"]);
			ss >> colormatrix;
			assert(colormatrix.dim1() == 4 && colormatrix.dim2() == 4);
		}
		ViewContext::Get().SetPose(Pose(x0, y0, z0, 0));
		ViewContext::Get().Yellowize(0.6);
		ViewContext::Get().DisableKeyboard();

		// Make a miniature gui.
		cvNamedWindow("Reference Image", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("All", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("Best", CV_WINDOW_AUTOSIZE);
		IplImage* reference = NULL;
		IplImage* all = NULL;
		IplImage* best = NULL;

		// Build our sensor models.
		L1SensorModel L1S(ViewContext::Get().width(),
				ViewContext::Get().height());
		L2SensorModel L2S(10, false);
		NormalizedL2SensorModel NL2S(10);
		PerChannelNormalizedL2SensorModel PCNL2S(0.5, false);
		L2HueSensorModel HS(5000);
		L1HueSensorModel L1HS(ViewContext::Get().width(),
				ViewContext::Get().height());
		GrayScaleL2SensorModel GSL2S(0.1, true);
		MutualInformationSensorModel MIS(4);

		// We're doing kidnapped, so let's scatter some particles; we start by
		// building the bounding box, and we go from there.
		//
		// TODO: Add the inside-outside test, 'cause that would be nice.
		vector<Particle> filter;
		vector<int> bbox = BareCell::BoundingBox(map);
		SparseMap sparsemap(map);
		double bbox_x_width = bbox[1] - bbox[0];
		double bbox_y_width = bbox[3] - bbox[2];
		if (argc == 6) {
			cout << "Kidnapped mode, with " << PARTICLE_COUNT << " particles."
					<< endl;
			int successes = 0;
			while (successes < PARTICLE_COUNT) {
				double xpos = bbox_x_width * Random::Get()->Uniform();
				double ypos = bbox_y_width * Random::Get()->Uniform();
				Pose temp(xpos + bbox[0], ypos + bbox[2], z0, 0);
				if (!sparsemap.InAWall(temp)
						&& sparsemap.UpDownInBoundsCheck(temp)) {
					//for (int j = 0 ; j < 360 ; j += 5)
					//{
					//  filter.push_back(Particle(Pose(xpos + bbox[0],
					//                                 ypos + bbox[2],
					//                                 z0,
					//                                 radians(j)),
					//                            1.0));
					//}
					filter.push_back(
							Particle(
									Pose(xpos + bbox[0], ypos + bbox[2], z0,
											radians(
													360
															* Random::Get()->Uniform())),
									1.0));
					successes++;
					if (successes % 100 == 0)
						cout << "." << flush;
				}
			}
			cout << endl;
		} else  // tracking mode
		{
			cout << "Tracking mode!" << endl;
			while (filter.size() < (size_t) PARTICLE_COUNT) {
				// Handmade ground truth for the lab dataset.
//        filter.push_back(
//            Particle(Pose(4102.94, 4133.88, 4072.03, radians(249)), 1.0));
				// Handmade ground truth for the "old" hallway dataset.
				filter.push_back(
						Particle(Pose(3955.69, 4366.25, 4072.03, 2.11185),
								1.0));
				// Handmade ground truth for the "new" hallway dataset.
//          filter.push_back(
//              Particle(Pose(4257.74, 4315.17, 4071.96, 2.09438), 1.0));
				// DWING  23 Jan "Dark" pose
//          filter.push_back(
//              Particle(Pose(4304.57, 4249, 4071.96, 2.07693), 1.0));
			}
		}
		Particle::Normalize(filter);
		Pose p = ViewContext::Get().pose();
		//DrawAndSave((format("%d_all.png") % 0).str().c_str(), filter, map, &p);
		if (all != NULL)
			cvReleaseImage(&all);
		all = Draw(filter, map, &p);
		cvSaveImage(str(format("%d_all.png") % 0).c_str(), all);
		cvShowImage("All", all);
		cvWaitKey(30);

		/*
		 * Finally, we start iterating through our actions and images.
		 */
		IplImage* img = NULL;
		assert(actions.size() == image_names.size());
		for (size_t i = 0; i < actions.size(); ++i) {
			// Clean out the old images, if necessary.
			if (reference != NULL) {
				cvReleaseImage(&reference);
				reference = NULL;
			}
			if (all != NULL) {
				cvReleaseImage(&all);
				all = NULL;
			}
			if (best != NULL) {
				cvReleaseImage(&best);
				best = NULL;
			}
			int floored = 0;
			cout << "Step " << i << endl;
			/* Step 1: get the real image and scale it. */
			IplImage* reference_big = cvLoadImage(
					image_names.at(i).string().c_str());
			reference = cvCreateImage(
					cvSize((int) (reference_big->width * SCALEFACTOR),
							(int) (reference_big->height * SCALEFACTOR)),
					reference_big->depth, reference_big->nChannels);
			cvResize(reference_big, reference);
			cvReleaseImage(&reference_big);
			// Re-color it.
			//reference = GSL2S.ColorConversion(reference, ident(4), true);
			// Exposure-compensate it.
			exposure_compensation(reference, 2.0);

			// Show our reference image.
			cvShowImage("Reference Image", reference);
			cvWaitKey(30);

			int pidx = 0;
			int best_idx = 0;
			double best_prob = 0;
			IplImage* ref_gray = cvCreateImage(
					cvSize(reference->width, reference->height), IPL_DEPTH_8U,
					1);
			cvCvtColor(reference, ref_gray, CV_BGR2GRAY);
			CvScalar ref_mean, ref_stddev;
			cvAvgSdv(ref_gray, &ref_mean, &ref_stddev);
			cout
					<< format("Reference: mu: %f sigma: %f") % ref_mean.val[0]
							% ref_stddev.val[0] << endl;

			foreach(Particle& p, filter){
			//CvScalar img_mean, img_stddev;
			/* Step 2: perturbation. */
			bool keep_going = true;
			int attempts = 0;
			while (keep_going && attempts < 100)
			{
				Pose oldpose = p.pose();
				vector<double> sample = MM.Sample(actions.at(i).x(),
						actions.at(i).y(),
						actions.at(i).theta(),
						p.pose().theta());
				p.Move(sample[0], sample[1], sample[2]);
				if (sparsemap.InAWall(p.pose(), 0.4))
				{
					p.SetPose(oldpose);
				}
				else
				{
					keep_going = false;
				}
				attempts++;
			}
		}
			cout << "Before 378: filter has size " << filter.size() << endl;
			if (all != NULL)
				cvReleaseImage(&all);
			all = Draw(filter, map);
			cvShowImage("All", all);
			foreach(Particle& p, filter){
			if (pidx % 25 == 0)
			cvWaitKey(5);
			/* Step 3: Generate the rendered image and weight it.*/
			double prob = 0.0;
			if (sparsemap.InAWall(p.pose(), 0.4))
			prob = 0.0;
			else
			{
				ViewContext::Get().SetPose(p.pose());
				if (img != NULL)
				cvReleaseImage(&img);
				img = ViewContext::Get().Render();
				bool uncertain = false;
//          cvSaveImage(str(format("%d_%d_img_nopaint.png") % i % pidx).c_str(), 
//                      img);
				img = GSL2S.Inpaint(img, cvScalar(0, 255, 0), &uncertain);
				if (img == NULL)
				{
					cout << "Particle " << pidx << " NULLified!" << endl;
					prob = 0.0;
				}
				else
				{
					//prob = L1S(reference, img);
					//prob = L2S(reference, img);
					prob = PCNL2S(reference, img);
					//prob = NL2S(reference, img);
					//prob = MIS(reference, img);
//            if (img != NULL)
//              cvSaveImage(str((format("%d_%d_img.png") % i % pidx)).c_str(), img);
					// Compute variance & stddev of our images.
					//IplImage* img_gray =
					//  cvCreateImage(cvSize(reference->width, reference->height),
					//                IPL_DEPTH_8U, 1);
					//cvCvtColor(img, img_gray, CV_BGR2GRAY);
					//cvAvgSdv(img_gray, &img_mean, &img_stddev);
					//cvReleaseImage(&img_gray);
					//if (img_stddev.val[0] < 0 * ref_stddev.val[0]) // was 0.75 *||
					//    //img_stddev.val[0] > 1.25 * ref_stddev.val[0])
					//{
					//  prob = 0.0;
					//  cout << "Particle " << pidx
					//       << " killed by sigma of " << img_stddev.val[0]
					//       << " (wanted " << ref_stddev.val[0] << ")" << endl;
					//}
					if (prob != 0.0 && uncertain)
					{
						cout << "Particle " << pidx << " floored!" << endl;
						floored++;
					}

					if (std::isnan(prob))
					{
						cout << "Particle " << pidx << " is NAN!" << endl;
						prob = 0.0;
					}
				}

				if (prob > best_prob)
				{
//            cout << format("New best [%d]: mu: %f sigma: %f P: %f")
//                      % pidx % img_mean.val[0] % img_stddev.val[0] % prob
//                 << endl;
					cout << format("New best [%d]: P: %f") % pidx % prob << endl;
					cvShowImage("Best", img);
					cout << format("Step %d new best: ") % i
					<< p.pose() << endl;
					cvWaitKey(30);
					best_prob = prob;
					best_idx = pidx;
				}
			}
			p.set_weight(prob);
			if (p.weight() < 0.0000000001)
			{
				p.set_weight(0.0000000001);
			}
			cout << format("Particle %d: %0.10f") % pidx % p.weight() << endl;
			pidx++;
		}
			cvReleaseImage(&ref_gray);
			cvSaveImage((format("%d_ref.png") % i).str().c_str(), reference);
			Pose p = ViewContext::Get().pose();
			all = Draw(filter, map, &p);
			cvSaveImage((format("%d_all.png") % i).str().c_str(), all);
			cvShowImage("All", all);
			cvWaitKey(30);
			Particle::Normalize(filter);

			// Save the best image.
			ParticlePair extremes = FindBestAndWorstParticles(filter);
			ViewContext::Get().SetPose(extremes.first.pose());
			best = ViewContext::Get().Render();
			//@US - Inject ePnP here
			/** a. Find correspondences between the reference image and the best image
			 * b. Back project best image 2D coordinates to 3D
			 * c. use ePnP on matching reference pixels and these 3D points
			 */
			if (_use_features) {
				//Find correspondences - lift code from sensormodels.cc
				//-- Step 1: Detect the keypoints using SURF Detector
				double total = 0.0;
				int minHessian = 400;

				cv::SurfFeatureDetector detector(minHessian);

				std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
				//		cv::Mat ref_gray2, ref_read;
				IplImage* ref_gray2 = cvCreateImage(
						cvSize(reference->width, reference->height),
						IPL_DEPTH_8U, 1);
				IplImage* ref_read = cvCreateImage(
						cvSize(best->width, best->height), IPL_DEPTH_8U, 1);
				cvCvtColor(best, ref_read, CV_BGR2GRAY);
				cvCvtColor(reference, ref_gray2, CV_BGR2GRAY);

				//		cv::cvtColor(cv::Mat(reference), ref_gray2, CV_BGR2GRAY);
				//		cv::cvtColor(cv::Mat(best), ref_read, CV_BGR2GRAY);

				detector.detect(ref_gray2, keypoints_1);
				detector.detect(ref_read, keypoints_2);

				//-- Step 2: Calculate descriptors (feature vectors)
				cv::SurfDescriptorExtractor extractor;

				cv::Mat descriptors_1, descriptors_2;

				extractor.compute(ref_gray2, keypoints_1, descriptors_1);
				extractor.compute(ref_read, keypoints_2, descriptors_2);

				//-- Step 3: Matching descriptor vectors using FLANN matcher
				std::cout << keypoints_1.size() << keypoints_2.size() << "\n";
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
					cv::drawMatches(ref_gray2, keypoints_1, ref_read,
							keypoints_2, good_matches, matches_img,
							cv::Scalar::all(-1), cv::Scalar::all(-1),
							vector<char>(),
							cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
					cv::imshow("OURAWESOMEWINDOW", matches_img);

					//Now find the 3D points corresponding to the rendered image
					//a. Pull out the corresponding 2D points in the reference image and rendered image
					std::vector<cv::Point2f> ref_pts, ren_pts;
					std::vector<cv::Point2f> all_ref, all_ren;
					//store the point matches in the keypoints struct in these vectors
					cv::KeyPoint::convert(keypoints_1, all_ref);
					cv::KeyPoint::convert(keypoints_2, all_ren);
					for (std::vector<cv::DMatch>::iterator it =
							good_matches.begin(); it != good_matches.end();
							++it) {
						ref_pts.push_back(all_ref[it->trainIdx]);
						ren_pts.push_back(all_ren[it->queryIdx]);
					}

					cout << "\nNumber of matches :: " << ref_pts.size() << endl;
					if (ref_pts.size() >= 4) {

						//b. Get corresponding 3D points from viewcontext
						std::vector<cv::Point3f> map_pts;
						//c. Call solvepnp
						cv::Mat rvec;
						//@todo use current pose estimate to speed things up/optimize
						//cv::Mat cam = "0.9795 0.0940 0.1780 85.8394 -0.0097 0.9053 -0.4246 -33.9370 -0.2010 0.4141 0.8877 138.1021 0 0 0 1";
						std::vector<cv::Point2f> indices;
						for (int i = 0; i < ref_gray2->height; i++) {
							for (int j = 0; j < ref_gray2->width; j++) {
								indices.push_back(cv::Point2f(j, i));
							}
						}
						Array2D<double> _camera = ViewContext::Get().Camera();
//						Array2D<double> _camera(4, 4);
//						stringstream camstream(config["robot_cam"]);
//						camstream >> _camera;
						Array2D<double> world_to_robot =
								ViewContext::Get().Robot();
						Array2D<double> robot_to_camera =
								ViewContext::Get().Camera();
						Array2D<double> world_to_camera_init = matmult(
								robot_to_camera, world_to_robot);

						cout << "\n ROBOT TO CAMERA :: " << robot_to_camera;
						cout << "\n WORLD TO ROBOT :: " << world_to_robot;
//						cout<<"\n WORLD TO CAMERA :: "<<world_to_camera_init;
						//cv::Mat init_T = cv::Mat(4,4,CV_64F);

						cout << "World to camera init" << world_to_camera_init
								<< endl;
						//cout<<init_T;
						cv::Mat R_init(3, 3, CV_64F);
						cv::Mat tvec(3, 1, CV_64F);
						for (int i = 0; i < 3; i++) {
							for (int j = 0; j < 3; j++) {
								R_init.at<double>(i, j) =
										world_to_camera_init[i][j];
							}
						}

						for (int i = 0; i < 3; i++) {
							tvec.at<double>(0, i) = world_to_camera_init[i][3];
						}

						//init_T.copyTo(R_init(cv::Rect(1,1,1,1)));
						cout << "R_init " << R_init << endl;
						//init_T(cv::Rect(0,3,1,3)).copyTo(tvec);
						cout << "tvec " << tvec << endl;
						cout << "\n\n\n\n";
						cv::Rodrigues(R_init, rvec);

						ViewContext::Get().Get3Dfrom2D(indices, map_pts);

						_cameraMatrix.at<double>(2,2) = _cameraMatrix.at<double>(2,2)/SCALEFACTOR;

						_cameraMatrix = _cameraMatrix/_cameraMatrix.at<double>(2,2);
						cout<<_cameraMatrix;
						cv::solvePnP(map_pts, indices, _cameraMatrix,
								_distCoeffs, rvec, tvec, true, CV_EPNP);

						cv::Mat R;
						cv::Rodrigues(rvec, R);
						cv::Mat T = cv::Mat::zeros(4, 4, R.type());
						T(cv::Range(0, 3), cv::Range(0, 3)) = R * 1; // copies R into T
						T(cv::Range(0, 3), cv::Range(3, 4)) = tvec * 1; // copies tvec into T

						cout << "\n EPNP WORLD TO CAMERA :: " << T;
						cout << R << "\n\n" << tvec;
						Array2D<double> world_to_camera(4, 4, (double*) T.data);
						world_to_camera[3][3] = 1;
						cout << world_to_camera << "\n\n"
								<< invert(world_to_camera);
						//This is a transformation from the world to camera. We need to chain it with camera to robot to get world to robot

						//This is from robot to camera
						Array2D<double> world_transform;
//						world_transform = (matmult(invert(world_to_camera),
//								(_camera)));
//					Pose rectified_pose(world_transform[0][3],
//							world_transform[1][3], world_transform[2][3],
//							atan2(world_transform[0][1],
//									world_transform[1][1]));
						stringstream s;
//						s<< "4 4 0.2059   -0.7564   -0.6208 0  -0.2706    0.5657   -0.7790 0  0.9405    0.3283   -0.0882 0  0 0 0 1";
						s<<"4 4 -0.2011   -0.9796    0.0097 0  0.4142   -0.0940   -0.9054 0  0.8878   -0.1780    0.4246 0 0 0 0 1";
						Array2D<double> reverseMagicRotation;
						s >> reverseMagicRotation;
						world_transform = matmult(invert(world_to_camera), reverseMagicRotation);
						Array2D<double> O = Origin<double>();
						Array2D<double> X = PlusX<double>();
						O = matmult(world_transform, O);
						X = matmult(world_transform, X);
						X = X - O;
						Pose rectified_pose = Pose(O[0][0], O[1][0], O[2][0],
								atan2(X[1][0], X[0][0]));

						//Diagnostic - print camera pose wrt world
						Array2D<double> best_transform =
								ViewContext::Get().Robot();
						Array2D<double> camera_transform = matmult(
								best_transform, _camera);
						cout << " Raw pose: " << world_transform
								<< "\n Camera_transform " << camera_transform;
						//Whew, print this out

						cout << "Our pose: " << rectified_pose << endl;

						//Let's look at this pose

						ViewContext::Get().SetPose(rectified_pose);
						IplImage* new_view = NULL;
						new_view = ViewContext::Get().Render();
						imshow("walllla", cv::Mat(new_view));

//					cout << "\n3D points" << map_pts;
						//Test to see if get 3D from 2D works. It does.
//					std::vector<cv::Point2f> indices;
//					for (int i = 0; i < ref_gray2->height; i++) {
//						for (int j = 0; j < ref_gray2->width; j++) {
//							indices.push_back(cv::Point2f(j, i));
//						}
//					}
//					ViewContext::Get().Get3Dfrom2D(indices, map_pts);
//					ofstream outfile("point.txt", ios::out);
//					for(vector<cv::Point3f>::const_iterator i = map_pts.begin(); i != map_pts.end(); ++i) {
//					    outfile << *i << '\n';
//					}
//					outfile.close();

					}
				}
			}

			//best = GSL2S.Inpaint(best, cvScalar(0, 255, 0), true);
			if (best != NULL)
				cvSaveImage((format("%d_best.png") % i).str().c_str(), best);
			else {
				cout << "BEST IS NULL" << endl;
				exit(1);
			}

			/*
			 * Now we should output some debugging information.
			 */
			cout << "Best pose: " << extremes.first.pose() << endl;
			cv::waitKey(-1);
			cout
					<< format(
							"Weights: %05.6f (best) %05.6f (worst) %05.6f (ratio)")
							% extremes.first.weight() % extremes.second.weight()
							% (extremes.first.weight()
									/ extremes.second.weight()) << endl;
			cout
					<< format("Neff: %f (%d particles)") % CalculateNeff(filter)
							% filter.size() << endl;
			cout << "Floored " << floored << " particles." << endl;

			/* Finally, resample, and then we're done. */
			if (floored < 0.5 * filter.size()) {
				cout << "Resampling because of Neff!" << endl;
				vector<Particle> new_filter;
				for (size_t i = 0; i < filter.size(); ++i) {
					double sample = Random::Get()->Uniform();
					double total = 0.0;
					for (size_t j = 0; j < filter.size(); ++j) {
						total += filter.at(j).weight();
						if (total >= sample) {
							new_filter.push_back(filter.at(j));
							break;
						}
					}
				}
				filter.swap(new_filter);
			}
		} // end of action-processing loop.

		ParticlePair extremes = FindBestAndWorstParticles(filter);
		foreach(Pose p, extremes.first.poses()){
		cout << p << endl;
	}

		cvDestroyWindow("Reference Image");
		cvDestroyWindow("All");
		cvDestroyWindow("Best");
		if (reference != NULL)
			cvReleaseImage(&reference);
		if (all != NULL)
			cvReleaseImage(&all);
		if (best != NULL)
			cvReleaseImage(&best);
	} catch (string e) {
		cout << "Main caught: " << e << endl;
	}
}
