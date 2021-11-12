#include "road.h"
#include <stdlib.h>    
#include <numeric>
#include <iostream>
#include <fstream>
#include <functional>



road::road(const Mat& input)
{
	//-- angle
	Ka = 3; cov0_a = 500; alpha = 0.004; p = 1; cov_low_a = 1; cov_hi_a = 100;

	mode_angle = new Gaussian[input.rows * input.cols * Ka];
	mode_a_num = new int[input.rows * input.cols];

	for (int i = 0; i < input.rows * input.cols; i++) {
		mode_a_num[i] = 0;
	}

	for (int i = 0; i < input.rows * input.cols * Ka; i++) {
		mode_angle[i].sigma = 0.0;
		mode_angle[i].mu = 0.0;
		mode_angle[i].weight = 0.0;
		mode_angle[i].n = 0;
	}

	//-- flow
	Kf = 3; cov0_f = 500; alpha_f = 0.004; p_f = 1; cov_low_f = 1; cov_hi_f = 100;

	mode_flow = new Gaussian[input.rows * input.cols * Kf * channel];
	mode_f_num = new int[input.rows * input.cols];

	for (int i = 0; i < input.rows * input.cols; i++) {
		mode_f_num[i] = 0;
	}

	for (int i = 0; i < input.rows * input.cols * Kf * channel; i++) {
		mode_flow[i].sigma = 0.0;
		mode_flow[i].mu = 0.0;
		mode_flow[i].weight = 0.0;
		mode_flow[i].n = 0;
	}


	//-- Mat
	downDir = Mat::zeros(input.rows, input.cols, CV_8UC1);
	upDir = Mat::zeros(input.rows, input.cols, CV_8UC1);
	leftDir = Mat::zeros(input.rows, input.cols, CV_8UC1);
	rightDir = Mat::zeros(input.rows, input.cols, CV_8UC1);
	wholeRoad = Mat::zeros(input.rows, input.cols, CV_8UC1);
	wholeRoad2 = Mat::zeros(input.rows, input.cols, CV_32FC1);
	wholeRoad3 = Mat::zeros(input.rows, input.cols, CV_8UC1);

	output_roi = Mat::zeros(input.rows, input.cols, CV_8UC3);

	acc_traj = Mat::zeros(input.rows, input.cols, CV_8UC1);
	degrees = Mat::zeros(input.rows, input.cols, CV_32FC1); // avg of angles
	numbers = Mat::zeros(input.rows, input.cols, CV_32FC1); // number of times each pixel has been foreground
	angles = Mat::zeros(input.rows, input.cols, CV_32FC1); // gmm of angles
	flows = Mat::zeros(input.rows, input.cols, CV_32FC2); // gmm of flows
	flow_angles = Mat::zeros(input.rows, input.cols, CV_32FC1); 
	flow_angles_median = Mat::zeros(input.rows, input.cols, CV_32FC1); 
	flow_mags = Mat::zeros(input.rows, input.cols, CV_32FC1); 

	flood_seeds = Mat::zeros(input.rows, input.cols, CV_8UC1);
	flood_mask = Mat::zeros(input.rows, input.cols, CV_8UC1);

	merged_binary = Mat::zeros(input.rows, input.cols, CV_8UC1);
	roadProbEst = Mat::zeros(input.rows, input.cols, CV_8UC1);
	accForegroundMask = Mat::zeros(input.rows, input.cols, CV_8UC1);
	totalMotionMask = Mat::zeros(input.rows, input.cols, CV_8UC1);
	accFloodMat = Mat::zeros(input.rows, input.cols, CV_32FC1); // number of times each pixel has been in flood
	P_flood = Mat::zeros(input.rows, input.cols, CV_8UC1); // flood of each frame

	//-------------------------------------------------------
	//-- just representation
	//-------------------------------------------------------
	arrows2 = Mat::zeros(input.rows, input.cols, CV_8UC3);
	test = Mat::zeros(input.rows, input.cols, CV_8UC3);
	flood_fill = Mat::zeros(input.rows, input.cols, CV_8UC3);
	show_flow = Mat::zeros(input.rows, input.cols, CV_8UC3);
	arrows_drawn = Mat::zeros(input.rows, input.cols, CV_8UC1);

	ginv = Mat::zeros(input.rows, input.cols, CV_8UC1);
	cinv = Mat::zeros(input.rows, input.cols, CV_8UC1);
	hinv = Mat::zeros(input.rows, input.cols, CV_8UC1);
	allinv = Mat::zeros(input.rows, input.cols, CV_8UC1);
	pg = Mat::zeros(input.rows, input.cols, CV_8UC1);
	pc = Mat::zeros(input.rows, input.cols, CV_8UC1);
	ph = Mat::zeros(input.rows, input.cols, CV_8UC1);
	ps = Mat::zeros(input.rows, input.cols, CV_8UC1);
	pf = Mat::zeros(input.rows, input.cols, CV_8UC1);
	pall = Mat::zeros(input.rows, input.cols, CV_8UC1);
	pgbhist = Mat::zeros(input.rows, input.cols, CV_8UC1);
	pgrayhist = Mat::zeros(input.rows, input.cols, CV_8UC1);
	merged = Mat::zeros(input.rows, input.cols, CV_8UC1);
}

road::~road()
{
	delete[] mode_angle;
	delete[] mode_a_num;
}




//================================================================================
// road process
//================================================================================

Mat downRoad = imread("D:/Project/traffic/results/downDir.jpg",IMREAD_GRAYSCALE);
Mat upRoad = imread("D:/Project/traffic/results/upDir.jpg", IMREAD_GRAYSCALE);

//Mat downRoad = imread("C:/folder/project/05072020_accident/results/downDir.jpg", IMREAD_GRAYSCALE);
//Mat upRoad = imread("C:/folder/project/05072020_accident/results/upDir.jpg", IMREAD_GRAYSCALE);

void road::roadProcess(Mat& frame, Mat& mask, Mat& background, Mat& initial_map, cvb::CvTracks tracks, int frame_num, int frame_rate, deque<Mat> frameSeq,  int move_length, int update_period) {

	cv::Mat smooth_curve = Mat::zeros(frame.rows, frame.cols, CV_8UC3);
	cv::Mat down_smooth_curve, up_smooth_curve;
	cv::Mat edges;
	
	//--separate directions
	twoSideRoadDetection(frame, background, mask, tracks, move_length, frame_num, frameSeq);
	imshow("whole_convex", whole_convex);

	if (frame_num > 3000) {
		cv::medianBlur(flow_angles, flow_angles_median, kernel_size);

		//-- show flow_angles_median
		frame.copyTo(flow_angle_m_arrows);
		for (int y = 0; y < flow_angle_m_arrows.rows; y += 30)
			for (int x = 0; x < flow_angle_m_arrows.cols; x += 30) {
				if (flow_angles_median.at<float>(y, x) != 0.0) {
					//-- get the angle from y, x position
					const float angleatxy = flow_angles_median.at<float>(y, x);

					//-- draw line at flow direction
					arrowedLine(flow_angle_m_arrows, Point(x, y), Point(x + 20 * cos(angleatxy), y + 20 * sin(angleatxy)), Scalar(0, 0, 255), 3, CV_AA, 0, 0.35);
					arrowedLine(flow_angle_m_arrows, Point(x, y), Point(x + 20 * cos(angleatxy), y + 20 * sin(angleatxy)), Scalar(0, 255, 255), 1, CV_AA, 0, 0.35);
					//circle(flow_arrows, Point(x, y), 1, Scalar(0, 255, 255), -1);
				}
			}
		if (!flow_angle_m_arrows.empty()) imshow("flow_angle_m_arrows", flow_angle_m_arrows);
	}
	


	//-- show flow magnitudes
	background.copyTo(flow_mag_map);
	flow_mag_map.convertTo(flow_mag_map, CV_32FC3);
	Mat Bands[3];
	split(flow_mag_map, Bands);
	cv::add(Bands[1], 10 * flow_mags, Bands[1], Mat(flow_mags > 0));
	cv::add(Bands[2], 10 * flow_mags, Bands[2], Mat(flow_mags > 0));
	vector<Mat> channels = { Bands[0],Bands[1],Bands[2] };
	merge(channels, flow_mag_map);
	flow_mag_map.convertTo(flow_mag_map, CV_8UC3);
	if (!flow_mag_map.empty()) imshow("flow_mag_map", flow_mag_map);


	//-- segmentation
	//floodFillFromSeedMask(background, flood_seeds, 255, 255, 4);
	//roadEstimate(frame, background, mask, frame_num, tracks);
	//acc_mask(mask, tracks, 5);


	//short max_w = 100;
	//Mat G_inv_colored, frame_flood;
	//G_inv_colored = background.clone();
	////floodFillFromSeedMask(G_inv_colored, mask, &frame_flood);
	////floodFillFromSeedMask(G_inv_colored, flood_seeds, &frame_flood);
	//floodFillFromSeedMask(G_inv_colored, wholeRoad2, &frame_flood);
	//P_flood += ((w_flood * P_flood) + frame_flood) / (w_flood + 1);
	//if (w_flood <= max_w) w_flood++;
	//else w_flood = max_w;
	//Mat P_F;
	//P_flood.copyTo(P_F);
	////if(!whole_convex.empty()) P_F(Mat(whole_convex<200)) = 0;
	//bitwise_and(P_F, whole_convex, P_F);
	//if (!P_F.empty()) imshow("P_F", P_F);
}




//================================================================================
// motion-based functions
//================================================================================

void road::twoSideRoadDetection(Mat& frame, Mat& background, cv::Mat& mask, cvb::CvTracks& tracks, const short& move_length, int frame_num, deque<Mat> frameSeq)
{
	cv::Mat down_convex, up_convex;
	vector<cv::Point> down_convex_points, up_convex_points, whole_convex_points;
	float update_factor = 100;

	Mat flow;
	frame.copyTo(flow);
	//h_road->calculateOpticalFlowFarne(frame, frameSeq, 3, &flow, &show_flow);
	//h_road->calculateOpticalFlowLucas(frame, mask, frameSeq, 4, &flow, &show_flow);
	
	for (std::map<cvb::CvID, cvb::CvTrack*>::iterator it = tracks.begin(); it != tracks.end(); it++)
	{
		cvb::CvID id = (*it).first;
		cvb::CvTrack* track = (*it).second;
		if (track->width <= 0 || track->height <= 0)
			continue;

		//-- track blob
		Mat track_mask = Mat::zeros(mask.rows, mask.cols, CV_8UC1);
		Rect roi(track->minx, track->miny, track->width, track->height);
		mask(roi).copyTo(track_mask(roi));
		//dilate(track_mask, track_mask, Mat(), Point(-1, -1), 2, 1, 1);
		cv::add(track->motion_mask, track_mask, track->motion_mask, track_mask);

		//-- track angle
		track->angleMat.setTo(0.0, track_mask);
		cv::add(track->angleMat, track->angle, track->angleMat, track_mask);

		//-- average of moving angles
		//avgDegree(frame, track_mask, track->angle);

		//-- track flow
		//track->flowMat.setTo(0.0, track_mask);
		//cv::add(track->flowMat, flow, track->flowMat, track_mask);


		//Mat diff;
		//cv::subtract(flow_angles, track->angleMat, diff, track_mask);
		//flow_angles.setTo(track->angle, Mat(abs(diff) > 2));

		//-- increase the weight of pixels that belong to longer tracks
		float track_movement = norm(track->trajectory.back() - track->trajectory[0]);

		//-- update cells
		//for (int i = 0; i < leftCells.size(); i++)
		//{
		//	updateCell(frame, downRoad, leftCells[i], track);
		//	if (debug) {
		//		Point2d dirPoint = Point2d(leftCells[i].xcenter + 3 * leftCells[i].avgSpeed * cos(leftCells[i].avgDir), leftCells[i].ycenter + 3 * leftCells[i].avgSpeed * sin(leftCells[i].avgDir));
		//		arrowedLine(frame, Point(leftCells[i].xcenter, leftCells[i].ycenter), dirPoint, Scalar(255, 255, 255), 1, CV_AA, 0, 0.25);
		//	}
		//}

		//for (int i = 0; i < rightCells.size(); i++)
		//{
		//	updateCell(frame, upRoad, rightCells[i], track);
		//	if (debug) {
		//		Point2d dirPoint = Point2d(rightCells[i].xcenter + 3 * rightCells[i].avgSpeed * cos(rightCells[i].avgDir), rightCells[i].ycenter + 3 * rightCells[i].avgSpeed * sin(rightCells[i].avgDir));
		//		arrowedLine(frame, Point(rightCells[i].xcenter, rightCells[i].ycenter), dirPoint, Scalar(255, 255, 255), 1, CV_AA, 0, 0.25);
		//	}
		//}

		//-- valid tracks
		if (track->trajectory.size() >= move_length && track->inactive >= move_length && track_movement >= move_length) {
		//if (track_movement >= move_length) {

			//update_factor = track->trajectory.size();

			//-- generate flood fill seed points
			//generateFloodSeeds(frame, track, move_length);

			//-- gmm model of moving angles
			//degreeModel(frame, track->angleMat);

			//-- fix flow using track
			//float general_ang = atan2(track->trajectory.back().y - track->trajectory[0].y, track->trajectory.back().x - track->trajectory[0].x);
			//frame.setTo(Scalar(255, 255, 255), track->motion_mask);
			//circle(frame, track->trajectory[track->trajectory.size() / 2], 2, Scalar(0, 255, 255), CV_FILLED);
			//arrowedLine(frame, track->trajectory[track->trajectory.size() / 2], Point(track->trajectory[track->trajectory.size() / 2].x + 20 * cos(general_ang), track->trajectory[track->trajectory.size() / 2].y + 20 * sin(general_ang)), Scalar(0, 0, 255), 2, CV_AA, 0, 0.35);
			
			//Mat diff;
			//cv::subtract(flow_angles, general_ang, diff, track->motion_mask);
			//diff = cv::abs(diff);
			//flow_angles.setTo(general_ang, Mat(diff > .1));


			//-- flow angles
			//background.copyTo(flow_angle_arrows);
			//for (int y = 0; y < flow_angle_arrows.rows; y += 40)
			//	for (int x = 0; x < flow_angle_arrows.cols; x += 40) {
			//		if (flow_angles.at<float>(y, x) != 0.0) {
			//			//-- get the angle from y, x position
			//			const float angleatxy = flow_angles.at<float>(y, x);

			//			//-- draw line at flow direction
			//			arrowedLine(flow_angle_arrows, Point(x + 2, y + 2), Point(x + 30 * cos(angleatxy), y + 30 * sin(angleatxy)), Scalar(255, 255, 255), 6, CV_AA, 0, 0.35);
			//			arrowedLine(flow_angle_arrows, Point(x, y), Point(x + 30 * cos(angleatxy), y + 30 * sin(angleatxy)), Scalar(0, 255, 255), 2, CV_AA, 0, 0.35);
			//			//circle(flow_angle_arrows, Point(x, y), 3, Scalar(0, 255, 255), CV_FILLED);
			//		}
			//	}
			//if (!flow_angle_arrows.empty())
			//	imshow("gmm_angles_flow", flow_angle_arrows);


				//-- increase the weight of pixels that belong to longer tracks
			/*float track_movement = norm(track->trajectory.back() - track->trajectory[0]);
			cv::add(flow_weights, track_movement, flow_weights, track->motion_mask);
			imshow("flow_weights", flow_weights);*/

			//flowModel(frame, track->flowMat, track_movement);


			//-- whole motion area
			cv::add(wholeRoad, update_factor * Scalar(1), wholeRoad, track->motion_mask);

			//-- separate upwards and downwards
			if (track->trajectory.back().y < track->trajectory[0].y) { // going up
				cv::add(upDir, update_factor * Scalar(1), upDir, track->motion_mask);
				cv::subtract(downDir, Scalar(1), downDir, track->motion_mask);
				cv::subtract(upDir, downDir, upDir);
			}

			else { // going down
				cv::add(downDir, update_factor * Scalar(1), downDir, track->motion_mask);
				cv::subtract(upDir, Scalar(1), upDir, track->motion_mask);
				cv::subtract(downDir, upDir, downDir);
			}

			//if (debug) {
			cv::imshow("downDir", downDir );
			cv::imshow("upDir", upDir);
			cv::imshow("wholeRoad", wholeRoad);

			//Mat down = downDir * 1000;
			//Mat up = upDir * 1000;
			//imwrite("D:/Project/traffic/results/downDir.jpg", down);
			//imwrite("D:/Project/traffic/results/upDir.jpg", up);

			//imwrite("C:/folder/project/05072020_accident/results/downDir.jpg", down);
			//imwrite("C:/folder/project/05072020_accident/results/upDir.jpg", up);
			//}
		}
	}


	/*--only keep biggest areas--*/
	down_contour = findBiggestContour(downDir, 5);
	up_contour = findBiggestContour(upDir, 5);
	whole_contours = findBiggestContour(wholeRoad, 15);

	/*--remove small areas with different direction from two major areas--*/
	if (up_contour.size() > 1)
		removeIrrelevantContours(up_contour, 0.7);
	if (down_contour.size() > 1)
		removeIrrelevantContours(down_contour, 0.7);

	/*--covex hull of all contours at each side as final roi--*/
	convex(frame, whole_contours, &whole_convex, whole_convex_points);
	convex(frame, down_contour, &down_convex, down_final_counter);
	convex(frame, up_contour, &up_convex, up_final_counter);

	//-- remove intersection
	if (down_final_counter.size() > 5 && up_final_counter.size() > 5) {
		removeContourIntersection(frame, down_final_counter, up_final_counter, downDir, upDir);
	}

	//-- gmm model of optical flows
	//flowModel(frame, flow);

	//-- show output
	//down_convex.setTo(cv::Scalar(255, 0, 0), down_convex);
	//up_convex.setTo(cv::Scalar(0, 255, 0), up_convex);
	frame.copyTo(output_roi);
	cv::add(output_roi, Scalar(70, 0, 0), output_roi, upDir);
	//cv::add(output_roi, Scalar(10, 0, 0), output_roi, up_convex);
	cv::add(output_roi, Scalar(0, 70, 0), output_roi, downDir);
	//cv::add(output_roi, Scalar(0, 10, 0), output_roi, down_convex);

	//output_roi = frame
		//+ 0.5 * down_temp
		//+ 0.5 * up_temp
		//+ down_convex
		//+ up_convex
		//+ whole_convex
		//+ down_smooth_curve 
		//+ up_smooth_curve
		//;
	//cv::polylines(output_roi, up_final_counter, true, Scalar(255, 0, 0), 2, CV_AA);
	//cv::polylines(output_roi, down_final_counter, true, Scalar(0, 255, 0), 2, CV_AA);

	//cv::imshow("output_roi", output_roi);
}


void road::fourSideRoadDetection(cv::Mat& mask, cvb::CvTracks& tracks, const short& move_length)
{
	Mat direc = Mat::zeros(mask.rows, mask.cols, CV_8UC3);

	for (std::map<cvb::CvID, cvb::CvTrack*>::iterator it = tracks.begin(); it != tracks.end(); it++)
	{
		cvb::CvID id = (*it).first;
		cvb::CvTrack* track = (*it).second;
		if (track->width <= 0 || track->height <= 0)
			continue;


		//============= blob =====================
		Mat temp;
		Mat track_mask = Mat::zeros(mask.rows, mask.cols, CV_8UC1);
		Rect roi(track->minx, track->miny, track->width, track->height);
		mask(roi).copyTo(track_mask(roi));
		track_mask.convertTo(temp, CV_32FC1);

		//============= whole roi =====================

		if (track->lifetime > 10) {
			wholeRoad += 0.01 * temp;
		}

		//============= four sides =====================


		if (track->lifetime > 10
			&& track->trajectory.size() > move_length
			&& norm(track->trajectory.back() - track->trajectory[track->trajectory.size() - move_length]) >= move_length - 1
			)
		{




			if ((track->angle > CV_PI * .25 && track->angle < CV_PI * .75)) { //down
				downDir += 0.01 * temp;
				upDir = cv::max(0, upDir - 0.01 * temp);
				rightDir = cv::max(0, rightDir - 0.01 * temp);
				leftDir = cv::max(0, leftDir - 0.01 * temp);
				downDir = cv::max(0, downDir - upDir);
				downDir = cv::max(0, downDir - leftDir);
				downDir = cv::max(0, downDir - rightDir);
			}

			else if (track->angle > -CV_PI * .25 && track->angle < CV_PI * .25) { //right
				rightDir += 0.01 * temp;
				leftDir = cv::max(0, leftDir - 0.01 * temp);
				upDir = cv::max(0, upDir - 0.01 * temp);
				downDir = cv::max(0, downDir - 0.01 * temp);
				rightDir = cv::max(0, rightDir - leftDir);
				rightDir = cv::max(0, rightDir - upDir);
				rightDir = cv::max(0, rightDir - downDir);
			}

			else if (track->angle > -CV_PI * .75 && track->angle < -CV_PI * .25) { //up
				upDir += 0.01 * temp;
				downDir = cv::max(0, downDir - 0.01 * temp);
				leftDir = cv::max(0, leftDir - 0.01 * temp);
				rightDir = cv::max(0, rightDir - 0.01 * temp);
				upDir = cv::max(0, upDir - downDir);
				upDir = cv::max(0, upDir - leftDir);
				upDir = cv::max(0, upDir - rightDir);
			}

			else { //left
				leftDir += 0.01 * temp;
				rightDir = cv::max(0, rightDir - 0.01 * temp);
				downDir = cv::max(0, downDir - 0.01 * temp);
				upDir = cv::max(0, upDir - 0.01 * temp);
				leftDir = cv::max(0, leftDir - rightDir);
				leftDir = cv::max(0, leftDir - upDir);
				leftDir = cv::max(0, leftDir - downDir);
			}



			//============= outputs =====================

			//if (debug) {
				cv::imshow("downDir", downDir*100);
				cv::imshow("upDir", upDir*100);
				cv::imshow("leftDir", leftDir*100);
				cv::imshow("rightDir", rightDir*100);
				cv::imshow("wholeRoad", wholeRoad);
			//}
		}
	}
}




//**
// convex
//*

void road::convex(Mat& frame, std::vector<std::vector<cv::Point>> contours, cv::Mat* output_convex, vector<cv::Point>& output_convex_points) {

	RNG rng(12345);
	*output_convex = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
	//vector<Point> ConvexHullPoints;
	vector<Point> points;
	vector<vector<Point>> pts;

	//--------------------------------------------------
	// convex hull of separate contours
	//--------------------------------------------------
	//for (size_t i = 0; i < contours.size(); i++) {
	//	for (size_t j = 0; j < contours[i].size(); j++) {
	//		points.push_back(contours[i][j]);
	//	}
	//	pts.push_back(points);
	//	convexHull(pts[i], ConvexHullPoints);

	//	Scalar color = Scalar(rng.uniform(0, 10), rng.uniform(150, 255), rng.uniform(150, 255));
	//	
	//	polylines(*output_convex, ConvexHullPoints, false, color, 2);
	//	points.clear();

	//	imshow("convex", *output_convex);
	//}
	//--------------------------------------------------


	//--------------------------------------------------
	// convex hull of all contours at once
	//--------------------------------------------------
	vector<vector<Point>> hull;
	for (size_t i = 0; i < contours.size(); i++) {
		for (size_t j = 0; j < contours[i].size(); j++) {
			points.push_back(contours[i][j]);
		}
	}

	if (points.size() > 0) {
		convexHull(points, output_convex_points);
		hull.push_back(output_convex_points);
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255), -1);
		//cv::polylines(*output_convex, output_convex_points, true, color, 2);
		drawContours(*output_convex, hull, 0, Scalar(255), -1);
		points.clear();
		hull.clear();
	}
	//--------------------------------------------------
}




//**
// removeContourIntersection
//*

void road::removeContourIntersection(cv::Mat& frame, std::vector<cv::Point>& down_convex, std::vector<cv::Point>& up_convex, cv::Mat& downDir, cv::Mat& upDir) {
	cv::Mat down_convex_fill = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
	cv::Mat up_convex_fill = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
	cv::Mat down_fill, up_fill;

	cv::fillConvexPoly(down_convex_fill, down_convex, cv::Scalar(255), 8);
	cv::fillConvexPoly(up_convex_fill, up_convex, cv::Scalar(255), 8);

	up_convex_fill -= Mat(downDir>=20);
	down_convex_fill -= Mat(upDir>=20);
	
	vector<vector<Point>> up_contours , down_contours;
	if(!up_convex_fill.empty()) up_contours = findBiggestContour(up_convex_fill, 1);
	if(!down_convex_fill.empty()) down_contours = findBiggestContour(down_convex_fill, 1);

	if (!up_contours.empty()) up_convex.assign(up_contours[0].begin(), up_contours[0].end());
	if (!down_contours.empty()) down_convex.assign(down_contours[0].begin(), down_contours[0].end());
}




//**
// findBiggestContour
//*

std::vector<std::vector<cv::Point>> road::findBiggestContour(cv::Mat& img, int numOfContours) {

	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point>> biggest_contours;
	std::vector<cv::Vec4i > hierarchy;
	double minArea = 0.0;

	//-- crop sides of image
	const short crop_size = 3;
	img(Range(0, crop_size), Range(0, img.cols)) = 0; // crop top 10 pixels
	img(Range(img.rows - crop_size, img.rows), Range(0, img.cols)) = 0; // crop bottom 10 pixels
	img(Range(0, img.rows), Range(0, crop_size)) = 0; // crop down 10 pixels
	img(Range(0, img.rows), Range(img.cols - crop_size, img.cols)) = 0; // crop up 10 pixels

	Mat thresh_img;
	//threshold(img, thresh_img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	threshold(img, thresh_img, 20, 255, CV_THRESH_BINARY);
	//imshow("thresh_img", thresh_img);

	findContours(thresh_img.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1, Point());

	if(contours.size() > 0){

		vector<double> areas;
		for (int i = 0; i < contours.size(); i++)
			areas.push_back(contourArea(contours[i])); // contours areas

		sort(areas.begin(), areas.end(), greater<double>());
	
		if (contours.size() < numOfContours) {
			for (int i = 0; i < contours.size(); i++) {
				if (contourArea(contours[i]) > areas[0] / 5)
				{
					biggest_contours.push_back(contours[i]);
				}
			}
		}
		else {
			for (int i = 0; i < contours.size(); i++) {
				if (contourArea(contours[i]) > areas[numOfContours] && contourArea(contours[i]) > areas[0] / 5)
				{
					biggest_contours.push_back(contours[i]);
				}
			}
		}
	}

	contours.clear();
	hierarchy.clear();

	return biggest_contours;
}




//**
// average degree
//*

void road::avgDegree(Mat& frame, Mat& track_mask, float& deg) {
	Mat maskneg = Mat::zeros(track_mask.rows, track_mask.cols, CV_8UC1);
	bitwise_not(track_mask, maskneg);
	cv::add(numbers, 1, numbers, track_mask);
	Mat temp2 = Mat::zeros(track_mask.rows, track_mask.cols, CV_32FC1);
	Mat temp3 = Mat::zeros(track_mask.rows, track_mask.cols, CV_32FC1);
	cv::add(temp2, deg, temp2, track_mask);
	cv::subtract(temp2, degrees, temp2, track_mask);
	cv::divide(temp2, numbers, temp3, 1.0);
	temp2.copyTo(temp3, maskneg);
	cv::add(degrees, temp3, degrees, track_mask);

	/*-- draw degrees --*/
	frame.copyTo(arrows);
	for (int i = 0; i < arrows.cols; i += 30) {
		for (int j = 0; j < arrows.rows; j += 30) {
			if (degrees.at<float>(j, i) != 0) {
				Point2d dirPoint = Point2d(i + 20 * cos(degrees.at<float>(j, i)), j + 20 * sin(degrees.at<float>(j, i)));
				arrowedLine(arrows, Point2d(i, j), dirPoint, Scalar(0, 0, 255), 3, CV_AA, 0, 0.35);
				arrowedLine(arrows, Point2d(i, j), dirPoint, Scalar(0, 255, 255), 1, CV_AA, 0, 0.35);
			}
		}
	}
	if (!arrows.empty()) imshow("average_angles_track", arrows);
}



void road::degreeModel(Mat& frame, Mat& frame_angles) {

	for (int i = 0; i < frame_angles.rows - 1; i++) {
		float* i_ptr = frame_angles.ptr<float>(i);
		float* ang_ptr = angles.ptr<float>(i);

		for (int j = 0; j < frame_angles.cols - 1; j++) {

			if (*i_ptr != 0) {
				float totalWeight = 0.0;
				int pos = i * frame_angles.cols + j;

				//-- when no angle info
				if (mode_a_num[pos] == 0) {
					mode_angle[pos * Ka].sigma = cov0_a;
					mode_angle[pos * Ka].mu = *i_ptr;
					mode_angle[pos * Ka].n = 1;
					mode_angle[pos * Ka].weight = 1.0;
					if (*i_ptr != 0) mode_a_num[pos]++;
				}

				else {
					double p_b = 0.0;
					double max_p_v = 0.0;
					double max_sig_v = 0.0;
					int max_s = 0;
					double s_b = 0.0, w_b;

					//use the most significant componet as angle
					float p_c = -(mode_angle[pos * Ka].mu - *i_ptr) * (mode_angle[pos * Ka].mu - *i_ptr) / mode_angle[pos * Ka].sigma;
					p_b += p_c;
					s_b += mode_angle[pos * Ka].sigma;
					w_b = mode_angle[pos * Ka].weight;

					bool isFindMatch = false;
					float totalWeight = 0.0;

					//-- find match component
					for (int k = 0; k < mode_a_num[pos]; k++) {
						if (isFindMatch == false) {
							float var = 0.0;
							float dist = 0.0;
							var += mode_angle[pos * Ka + k].sigma;
							dist += (mode_angle[pos * Ka + k].mu - *i_ptr) * (mode_angle[pos * Ka + k].mu - *i_ptr);

							if (dist < p * var) {
								isFindMatch = true;

								mode_angle[pos * Ka + k].mu = mode_angle[pos * Ka + k].mu - alpha * (mode_angle[pos * Ka + k].mu - *i_ptr);
								float var_temp = mode_angle[pos * Ka + k].sigma + alpha * ((mode_angle[pos * Ka + k].mu - *i_ptr) * (mode_angle[pos * Ka + k].mu - *i_ptr) - mode_angle[pos * Ka + k].sigma);
								mode_angle[pos * Ka + k].sigma = var_temp < cov_low_a ? cov_low_a : (var_temp > cov_hi_a ? cov_hi_a : var_temp);

								mode_angle[pos * Ka + k].n++;

							}
						}
					}

					//if matching component is not found
					if (isFindMatch == false) {
						if (mode_a_num[pos] != Ka) {
							mode_angle[pos * Ka + mode_a_num[pos]].sigma = cov0_a;
							mode_angle[pos * Ka + mode_a_num[pos]].mu = *i_ptr;
							mode_angle[pos * Ka + mode_a_num[pos]].n = 1;
							if (*i_ptr != 0) mode_a_num[pos]++;
						}
						else {
							mode_angle[pos * Ka + (mode_a_num[pos] - 1)].sigma = cov0_a;
							mode_angle[pos * Ka + (mode_a_num[pos] - 1)].mu = *i_ptr;
							mode_angle[pos * Ka + (mode_a_num[pos] - 1)].n = 1;
						}


					}
					for (int t = 0; t < mode_a_num[pos]; t++) {
						totalWeight += mode_angle[pos * Ka + t].n;
					}
					if (totalWeight >= maxn - 1) {
						totalWeight /= 2;
						for (int t = 0; t < mode_a_num[pos]; t++) {
							mode_angle[pos * Ka + t].n /= 2;
							if (mode_angle[pos * Ka + t].n == 0) {
								mode_angle[pos * Ka + t].n++;
								totalWeight++;
							}
							mode_angle[pos * Ka + t].weight = mode_angle[pos * Ka + t].n / totalWeight;
						}
					}
					else {
						for (int t = 0; t < mode_a_num[pos]; t++) {
							mode_angle[pos * Ka + t].weight = mode_angle[pos * Ka + t].n / totalWeight;
						}
					}
					// adjust position
					for (int l = mode_a_num[pos] - 1; l > 0; l--) {
						if (mode_angle[pos * Ka + l].weight > mode_angle[pos * Ka + (l - 1)].weight) {
							Gaussian temp = mode_angle[pos * Ka + l];
							mode_angle[pos * Ka + l] = mode_angle[pos * Ka + (l - 1)];
							mode_angle[pos * Ka + (l - 1)] = temp;
						}
					}
				}

				float val = mode_angle[pos * Ka].mu;
				*ang_ptr = val;
			}

			ang_ptr++;
			i_ptr++;
		}
	}


	//-- show output
	frame.copyTo(angle_arrows);
	for (int i = 0; i < angle_arrows.cols; i += 30) {
		for (int j = 0; j < angle_arrows.rows; j += 30) {
			if (angles.at<float>(j, i) != 0) {
				Point2d dirPoint = Point2d(i + 20 * cos(angles.at<float>(j, i)), j + 20 * sin(angles.at<float>(j, i)));
				arrowedLine(angle_arrows, Point2d(i, j), dirPoint, Scalar(0, 0, 255), 3, CV_AA, 0, 0.35);
				arrowedLine(angle_arrows, Point2d(i, j), dirPoint, Scalar(0, 255, 255), 1, CV_AA, 0, 0.35);
				circle(flow_arrows, Point(i, j), 1, Scalar(0, 255, 255), -1);
			}
		}
	}
	if (!angle_arrows.empty()) imshow("gmm_track_angles", angle_arrows);
}


void road::flowModel(Mat& frame, Mat& frame_flows, float track_movement) {
	
	for (int i = 0; i < frame_flows.rows - 1; i++) {
		float* i_ptr = frame_flows.ptr<float>(i);
		float* f_ptr = flows.ptr<float>(i);
		float* fa_ptr = flow_angles.ptr<float>(i);
		float* fm_ptr = flow_mags.ptr<float>(i);

		for (int j = 0; j < frame_flows.cols - 1; j++) {

			if (*i_ptr != 0) {
				float totalWeight = 0.0;
				int pos = i * frame_flows.cols + j;

				//-- when no angle info
				if (mode_a_num[pos] == 0) {
					for (int c = 0; c < channel; c++) {
						mode_flow[pos * Kf * channel + c].sigma = cov0_f;
						mode_flow[pos * Kf * channel + c].mu = (float) * (i_ptr + c);
						mode_flow[pos * Kf * channel + c].n = 10;
						mode_flow[pos * Kf * channel + c].weight = 1.0;
					}
					mode_a_num[pos]++;
				}

				else {
					double p_b = 0.0;
					double max_p_v = 0.0;
					double max_sig_v = 0.0;
					int max_s = 0;
					double s_b = 0.0, w_b;

					//use the most significant componet as flow
					for (int c = 0; c < channel; c++) {
						float p_c = -(mode_flow[pos * Kf * channel + c].mu - (float) * (i_ptr + c)) * (mode_flow[pos * Kf * channel + c].mu - (float) * (i_ptr + c)) / mode_flow[pos * Kf * channel + c].sigma;
						p_b += p_c;
						s_b += mode_flow[pos * Kf * channel + c].sigma;
					}
					w_b = mode_angle[pos * Ka].weight;

					//-- update flow
					bool isFindMatch = false;
					float totalWeight = 0.0;

						//-- find match component
					for (int k = 0; k < mode_a_num[pos]; k++) {
						if (isFindMatch == false) {
							float var = 0.0;
							float dist = 0.0;
							for (int c = 0; c < channel; c++) {
								var += mode_flow[pos * Kf * channel + k * channel + c].sigma;
								dist += (mode_flow[pos * Kf * channel + k * channel + c].mu - (float) * (i_ptr + c)) * (mode_flow[pos * Kf * channel + k * channel + c].mu - (float) * (i_ptr + c));
							}
							if (dist < p * var) {
								isFindMatch = true;

								for (int c = 0; c < channel; c++) {
									mode_flow[pos * Kf * channel + k * channel + c].mu = mode_flow[pos * Kf * channel + k * channel + c].mu - alpha * (mode_flow[pos * Kf * channel + k * channel + c].mu - (float) * (i_ptr + c));
									float var_temp = mode_flow[pos * Kf * channel + k * channel + c].sigma + alpha * ((mode_flow[pos * Kf * channel + k * channel + c].mu - (float) * (i_ptr + c)) * (mode_flow[pos * Kf * channel + k * channel + c].mu - (float) * (i_ptr + c)) - mode_flow[pos * Kf * channel + k * channel + c].sigma);
									mode_flow[pos * Kf * channel + k * channel + c].sigma = var_temp < cov_low_f ? cov_low_f : (var_temp > cov_hi_f ? cov_hi_f : var_temp);

									mode_flow[pos * Kf * channel + k * channel + c].n++;
								}
								break;
							}
						}
					}

						//-- if matching component is not found
					if (isFindMatch == false) {
						if (mode_a_num[pos] != Ka) {
							for (int c = 0; c < channel; c++) {
								mode_flow[pos * Kf * channel + mode_f_num[pos] * channel + c].sigma = cov0_f;
								mode_flow[pos * Kf * channel + mode_f_num[pos] * channel + c].mu = (float) * (i_ptr + c);
								mode_flow[pos * Kf * channel + mode_f_num[pos] * channel + c].n = 1;
							}
							mode_a_num[pos]++;
						}
						else {
							for (int c = 0; c < channel; c++) {
								mode_flow[pos * Kf * channel + (mode_f_num[pos] - 1) * channel + c].sigma = cov0_f;
								mode_flow[pos * Kf * channel + (mode_f_num[pos] - 1) * channel + c].mu = (float) * (i_ptr + c);
								mode_flow[pos * Kf * channel + (mode_f_num[pos] - 1) * channel + c].n = 1;
							}
						}


					}
					for (int t = 0; t < mode_f_num[pos]; t++) {
						totalWeight += mode_flow[pos * Kf * channel + t * channel].n;
					}
					if (totalWeight >= maxn - 1) {
						totalWeight /= 2;
						for (int t = 0; t < mode_f_num[pos] * channel; t++) {
							mode_flow[pos * Kf * channel + t].n /= 2;
							if (mode_flow[pos * Kf * channel + t].n == 0) {
								mode_flow[pos * Kf * channel + t].n++;
								totalWeight++;
							}
							mode_flow[pos * Kf * channel + t].weight = mode_flow[pos * Kf * channel + t].n / totalWeight;
						}
					}
					else {
						for (int t = 0; t < mode_f_num[pos] * channel; t++) {
							mode_flow[pos * Kf * channel + t].weight = mode_flow[pos * Kf * channel + t].n / totalWeight;
						}
					}
						//-- adjust position (switch to the component with the heighest weight)
					for (int l = mode_f_num[pos] - 1; l > 0; l--) {
						if (mode_flow[pos * Kf * channel + l * channel].weight > mode_flow[pos * Kf * channel + (l - 1) * channel].weight) {
							for (int c = 0; c < channel; c++) {
								Gaussian temp = mode_flow[pos * Kf * channel + l * channel + c];
								mode_flow[pos * Kf * channel + l * channel + c] = mode_flow[pos * Kf * channel + (l - 1) * channel + c];
								mode_flow[pos * Kf * channel + (l - 1) * channel + c] = temp;
							}
						}
					}
				}

				float val1 = mode_flow[pos * Kf * channel].mu;
				float val2 = mode_flow[pos * Kf * channel + 1].mu;

				*f_ptr = val1; // flow_x
				*(f_ptr + 1) = val2; // flow_y
				*fa_ptr = atan2(val2, val1);
				*fm_ptr = sqrt(pow(val1, 2) + pow(val2, 2));

				////-- take median
				//vector<float> median_vec;
				//if (i > kernel_size / 2 && j > kernel_size / 2 && i < frame_flows.rows - kernel_size / 2 && j < frame_flows.cols - kernel_size / 2)
				//	for (int kx = -kernel_size / 2; kx <= kernel_size / 2; kx++)
				//		for (int ky = -kernel_size / 2; ky <= kernel_size / 2; ky++)
				//			if (flow_angles.at<float>(j + ky, i + kx) != 0)
				//				median_vec.push_back(flow_angles.at<float>(j + ky, i + kx));

				//*fa_ptr = h_road->median(median_vec);
				//median_vec.clear();
			}

			f_ptr += channel;;
			i_ptr += channel;
			fa_ptr++;
			fm_ptr++;
		}
	}


	//-- show output
		//-- flows
	//frame.copyTo(flow_arrows);
	//for (int y = 0; y < flow_arrows.rows; y += 10) 
	//	for (int x = 0; x < flow_arrows.cols; x += 10) {
	//		if (flows.at<float>(y, x) != 0.0) {
	//			//-- get the flow from y, x position * 10 for better visibility
	//			const Point2f flowatxy = flows.at<Point2f>(y, x);

	//			//-- draw line at flow direction
	//			arrowedLine(flow_arrows, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255, 255, 255), 1, CV_AA, 0, 0.25);
	//			//circle(flow_arrows, Point(x, y), 1, Scalar(0, 255, 255), -1);
	//		}
	//	}
	//if (!flow_arrows.empty()) 
	//	imshow("flow_arrows", flow_arrows);

		//-- flow angles
	//frame.copyTo(flow_angle_arrows);
	//for (int y = 0; y < flow_angle_arrows.rows; y += 5)
	//	for (int x = 0; x < flow_angle_arrows.cols; x += 5) {
	//		if (flow_angles.at<float>(y, x) != 0.0) {
	//			//-- get the angle from y, x position
	//			const float angleatxy = flow_angles.at<float>(y, x);

	//			//-- draw line at flow direction
	//			arrowedLine(flow_angle_arrows, Point(x, y), Point(x + 20 * cos(angleatxy), y + 20 * sin(angleatxy)), Scalar(0, 0, 255), 3, CV_AA, 0, 0.35);
	//			arrowedLine(flow_angle_arrows, Point(x, y), Point(x + 20*cos(angleatxy), y + 20*sin(angleatxy)), Scalar(0, 255, 255), 1, CV_AA, 0, 0.35);
	//			circle(flow_arrows, Point(x, y), 1, Scalar(0, 255, 255), -1);

	//			arrows_drawn.at<uchar>(y, x) = 1;
	//		}
	//	}
	//if (!flow_angle_arrows.empty()) 
	//	imshow("gmm_angles_flow", flow_angle_arrows);
}


//==============================================================================================
//                                        grid cells
//==============================================================================================

//**
// update cells
//*

void road::updateCell(Mat frame, Mat roadMask, road::Cell& c, cvb::CvTrack* t)
{
	int pixelValue = roadMask.at<uchar>(t->centroid.y, t->centroid.x);
	if ((pixelValue > 0) && (t->centroid.y > c.ymin) && (t->centroid.y < c.ymax) && (t->centroid.x > c.xmin) && (t->centroid.x < c.xmax) && (t->magnitude > 2))
	{
		c.number++;
		c.avgDir += (t->angle - c.avgDir) / (c.number + 1);
		c.avgSpeed += (t->magnitude - c.avgSpeed) / (c.number + 1);
	}
}


//**
// draw cells
//*


void road::drawRoiGrid(Mat frame, Mat roiFill, vector<road::Cell>& gridCells)
{
	vector< vector <Point> > contours;
	vector<Rect> rects;

	imshow("roiFill", roiFill);

	contours = findBiggestContour(roiFill, 1);

	int box_w = 5; // Define box width here
	int box_h = 15; // Define box height here
	int threshold_perc = 25; //perceantage value for eliminating the box according to pixel count inside the box
	int threshold = (box_w * box_h * threshold_perc) / 100;

	for (int i = 0; i < contours.size(); i++) {
		Rect r = boundingRect(contours[i]); // Find bounding rect
		rectangle(frame, r, Scalar(0, 0, 255), 0.5, 8, 0);

		//-- Scan the image within bounding box  
		int c = 0;
		for (int j = r.x; j < r.x + r.width; j = j + box_w) {
			for (int k = r.y; k < r.y + r.height; k = k + box_h) {
				Rect roi_rect(j, k, box_w, box_h);
				Mat roi = roiFill(roi_rect & r);
				int count = countNonZero(roi);
				if (count > threshold) {
					rects.push_back(roi_rect);
				}
			}
		}

		for (int i = 0; i < rects.size(); i++) {
			for (int j = 0; j < rects.size(); j++) {
				if (rects[i].y == rects[j].y && rects[i].x < rects[j].x) {
					rects[i].width = rects[i].width + rects[j].width;
					rects.erase(rects.begin() + j);
				}
			}
		}

		for (int i = 0; i < rects.size(); i++) {
			for (int j = 0; j < rects.size(); j++) {
				if (rects[i].y == rects[j].y && rects[i].x < rects[j].x) {
					rects[i].width = rects[i].width + rects[j].width;
					rects.erase(rects.begin() + j);
				}
			}
		}

		for (int i = 0; i < rects.size(); i++) {
			for (int j = 0; j < rects.size(); j++) {
				if (rects[i].y == rects[j].y && rects[i].x < rects[j].x) {
					rects[i].width = rects[i].width + rects[j].width;
					rects.erase(rects.begin() + j);
				}
			}
		}

		if (rects.size() > 1) {
			rects[rects.size() - 2].width = rects[rects.size() - 2].width + rects[rects.size() - 1].width;
			rects.erase(rects.end() - 1);
		}

		for (int i = 0; i < rects.size(); i++) {
			Cell ce = { 0 };
			gridCells.push_back(ce);
			gridCells[c].xmin = rects[i].x;
			gridCells[c].xmax = rects[i].x + rects[i].width;
			gridCells[c].ymin = rects[i].y;
			gridCells[c].ymax = rects[i].y + rects[i].height;
			gridCells[c].width = rects[i].width;
			gridCells[c].height = rects[i].height;
			gridCells[c].xcenter = gridCells[c].xmin + (gridCells[c].xmax - gridCells[c].xmin) / 2;
			gridCells[c].ycenter = gridCells[c].ymin + (gridCells[c].ymax - gridCells[c].ymin) / 2;
			if (debug) {
				rectangle(frame, rects[i], Scalar(255, 255, 255), 0.5, 8, 0);
			}
			c++;
		}
	}
}



//**
// remove irrelevant Contours
//*

void road::removeIrrelevantContours(vector<vector<Point>>& contours, float thresh) {
	vector<double> mean_degrees; //mean degree of each contour
	for (int i = 0; i < contours.size(); i++) {

		Mat mask = Mat::zeros(wholeRoad.rows, wholeRoad.cols, CV_8UC1);
		if (!contours[i].empty()) cv::fillConvexPoly(mask, contours[i], Scalar(255), CV_AA);

		Mat countour_degrees;
		degrees.copyTo(countour_degrees, mask);
		mean_degrees.push_back(cv::sum(countour_degrees)[0] / cv::countNonZero(countour_degrees));
		//cout << "mean of " << i << "=" << mean_degrees[i] << endl;
	}

	if (contours.size() == 2 && abs(mean_degrees[0] - mean_degrees[1]) > thresh) {
		if (contourArea(contours[0]) > contourArea(contours[1]))
			contours.erase(contours.begin() + 1);
		else
			contours.erase(contours.begin());
	}

	else { //more than 2 contours
		double maj = findMajority(mean_degrees, thresh);
		for (int i = 0; i < contours.size(); i++)
			if (abs(mean_degrees[i] - maj) > thresh)
				contours.erase(contours.begin() + i);
	}
}

//**
// find majority
//*
double road::findMajority(vector<double> a, float thresh)
{
	/*-- Function to find the candidate for Majority --*/
	int maj_index = 0, count = 1;
	for (int i = 1; i < a.size(); i++)
	{
		if (abs(a[maj_index] - a[i]) < thresh)
			count++;
		else
			count--;
		if (count == 0)
		{
			maj_index = i;
			count = 1;
		}
	}


	/*-- check if the candidate occurs more than n / 2 times --*/
	int count1 = 0;
	for (int i = 0; i < a.size(); i++)

		if (abs(a[i] - a[maj_index]) < thresh)
			count1++;

	if (count1 > a.size() / 2) {
		//cout << " " << a[maj_index] << " ";
		return a[maj_index];
	}

	else {
		//cout << "No Majority Element";
		return 0.0;
	}
}


//==============================================================================================
//                                        collective roads
//==============================================================================================

void road::generateFloodSeeds(const cv::Mat& frame, cvb::CvTrack* track, const int& move_length) {
	if (track->centroid.y > .4 * frame.rows) {
		track->angle /= CV_PI;
		if ((track->angle > .5) && (track->angle < 1)) { // vehicle going down-left
			flood_seeds.at<uchar>(track->maxy, track->maxx) = 255;
			//circle(frame, Point(track->maxx, track->maxy), 2, Scalar(0, 255, 124), 2, CV_AA);
			//putText(frame, cv::format("%1.1f", track->angle), Point(track->minx, track->miny), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 124), 1, CV_AA);
			//rectangle(frame, Rect(track->minx, track->miny, track->width, track->height), Scalar(0, 200, 124), 1, CV_AA);
		}
		else if ((track->angle > -.5) && (track->angle < 0)) { // vehicle going up-right
			flood_seeds.at<uchar>(track->miny, track->minx) = 255;
			//circle(frame, Point(track->minx, track->miny), 2, Scalar(0, 255, 124), 2, CV_AA);
			//putText(frame, cv::format("%1.1f", track->angle), Point(track->minx, track->miny), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 124), 1, CV_AA);
			//rectangle(frame, Rect(track->minx, track->miny, track->width, track->height), Scalar(0, 200, 124), 1, CV_AA);
		}
		else if ((track->angle > 0) && (track->angle < .5)) { // vehicle going down-right
			flood_seeds.at<uchar>(track->miny, track->maxx) = 255;
			//circle(frame, Point(track->maxx, track->miny), 2, Scalar(0, 255, 124), 2, CV_AA);
			//putText(frame, cv::format("%1.1f", track->angle), Point(track->minx, track->miny), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 124), 1, CV_AA);
			//rectangle(frame, Rect(track->minx, track->miny, track->width, track->height), Scalar(0, 200, 124), 1, CV_AA);
		}
		else { // vehicle going up-left
			flood_seeds.at<uchar>(track->maxy, track->minx) = 255;
			//circle(frame, Point(track->minx, track->maxy), 2, Scalar(0, 255, 124), 2, CV_AA);
			//putText(frame, cv::format("%1.1f", track->angle), Point(track->minx, track->miny), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 124), 1, CV_AA);
			//rectangle(frame, Rect(track->minx, track->miny, track->width, track->height), Scalar(0, 200, 124), 1, CV_AA);
		}
	}
}

void road::cropMask(Mat& frame, Mat& mask, cvb::CvTracks& tracks, const int& move_length, Mat* croppedMask) {
	*croppedMask = Mat::zeros(mask.rows, mask.cols, CV_8UC1);

	//--------------------------------------
	//-- just representation
	//--------------------------------------
	//-- show cropped mask on frame
	frame.copyTo(cropped);
	//--------------------------------------



	for (std::map<cvb::CvID, cvb::CvTrack*>::iterator it = tracks.begin(); it != tracks.end(); it++)
	{
		cvb::CvTrack* track = (*it).second;
		if (track->width <= 0 || track->height <= 0)
			continue;

		Mat track_mask = Mat::zeros(mask.rows, mask.cols, CV_8UC1);
		//Rect roi(track->minx, track->miny + (track->height / 2), track->width, track->height / 2);
		Rect roi(track->minx, track->miny, track->width, track->height);
		mask(roi).copyTo(track_mask(roi));

		//-- estimate the average size of the vehicles (scaled size) and horizon line
		float scale_size = roi.area() / track->centroid.y / track->centroid.y;

		if (roi.area() > 0)
			if (size_count < 10000) 
				ave_y = 1.0 * (ave_y * size_count + roi.y) / (size_count + 1);
			else
				ave_y = ave_y - (ave_y - roi.y) * 0.01;

		horizonLine = ave_y / 3;



		//-- crop road part of the mask
			//-- add bottom half of track blob
		//cv::add(*croppedMask, Scalar(255), *croppedMask, track_mask);

			//-- add road half of track blob
		float track_move_length = norm(track->trajectory.back() - track->trajectory[0]);

		//-- valid tracks
		if (track->trajectory.size() >= move_length && track_move_length >= move_length && track->centroid.y > horizonLine) {

			//-- estimate horizon line
			if (size_count < 10000) {
				ave_size = 1.0 * (ave_size * size_count + scale_size) / (size_count + 1);
				size_count++;
			}
			else {
				if (scale_size > 0.5 * ave_size && scale_size < 2 * ave_size)
					ave_size = ave_size - (ave_size - scale_size) * 0.01;
			}


			//-- generate flood fill seed points
				generateFloodSeeds(frame, track, move_length);


			//-- generate road sample mask
				//-- tack middle line
			Point a = Point(track->centroid.x - (2 * track->height) * cos(track->angle), track->centroid.y - (2 * track->height) * sin(track->angle));
			Point b = Point(track->centroid.x + (2 * track->height) * cos(track->angle), track->centroid.y + (2 * track->height) * sin(track->angle));

			track->angle /= CV_PI;
			for (int i = 0; i < mask.cols; i++)
				for (int j = 0; j < mask.rows; j++) {
					if (track_mask.at<uchar>(j, i) > 0) {
						Point c = Point(i, j);
						short sign = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); // if positive point is on the right side of the line
						if ((track->angle > -.5) && (track->angle < 0) && (sign > 0)) // vehicle going up-right
							croppedMask->at<uchar>(j, i) = 255;
						else if ((track->angle > 0) && (track->angle < .5) && (sign > 0)) // vehicle going down-right
							croppedMask->at<uchar>(j, i) = 255;
						else if ((track->angle > .5) && (track->angle < 1) && (sign < 0)) // vehicle going down-left
							croppedMask->at<uchar>(j, i) = 255;
						else if ((track->angle < -.5) && (track->angle > -1) && (sign < 0)) // vehicle going up-left
							croppedMask->at<uchar>(j, i) = 255;
					}
				}

			//-- just representation
			arrowedLine(cropped, a, b, Scalar(20,200,20), 2, CV_AA, 0, .2); 
		}
	}

	//-- just representation
	dilate(*croppedMask, *croppedMask, Mat(), Point(-1, -1), 2, 1, 1);
	cv::add(cropped, Scalar(10, 10, 250), cropped, *croppedMask);
	//imshow("croppedFrame", cropped);
}


//**
// detect optimal canny thresholds
//*
void road::canny_automatic_thresh(const Mat& input, double* lower_thresh_val, double* high_thresh_val) {

	//-- method 1 (otsu)
	//Mat otsu_output;
	//double otsu_thresh_val = cv::threshold(input, otsu_output, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	//high_thresh_val = otsu_thresh_val;
	//lower_thresh_val = otsu_thresh_val * 0.5;

	//-- method 2 (median)
	Mat mask = Mat::ones(input.size(), CV_8UC1);
	int nVals = 256; //2^8
	double med = imgMedian(input, mask);
	*lower_thresh_val = 0.66 * med;
	*high_thresh_val = 1.33 * med;
}

//**
// flood fill
//*
void road::floodFillFromSeedMask(const Mat& image, const Mat& seeds, Mat* outputMask)
{
	bool horizon = false;
	Mat flood, gray, mask, mask_no_edge, red_mask, white_mask, rg_chrome;
	red_mask = Mat::zeros(image.rows, image.cols, CV_8UC3);
	white_mask = Mat::zeros(image.rows, image.cols, CV_8UC1);

	image.copyTo(flood);

	//-- calculate lofill and upfill
	Scalar mean, dev;
	//meanStdDev(image, mean, dev, wholeRoad);
	meanStdDev(image, mean, dev);
	//cout << "mean: " << mean << "\tdev: " << dev << endl;
	int upfill = max(1, (int)(dev[0] / 20));
	int lofill = max(1, (int)(dev[0] / 20));
	//cout << "upfill: " << upfill << endl;

	//int upfill = 6;
	//int lofill = 6;

	//-- Create a mask from edges in the original image
	cvtColor(flood, gray, CV_BGR2GRAY);

	double low_th = 0.0, high_th = 0.0;
	canny_automatic_thresh(gray, &low_th, &high_th);
	cv::Canny(gray, mask, low_th, high_th);
	imwrite("./results/canny_edge.png", mask);
	//cv::Canny(gray, mask, 100, 200);
	
	if(horizon) line(mask, cv::Point(0, horizonLine), cv::Point(mask.size().width, horizonLine), cv::Scalar(255), 3);

	cv::copyMakeBorder(mask, mask, 1, 1, 1, 1, cv::BORDER_REPLICATE);
	auto sz = gray.size();

	//-- Fill mask with value 128
	uchar fillValue = 128;
	//for (int y = horizonLine; y < seeds.rows; y++) {
	for (int y = 0; y < seeds.rows; y++) {
		const uchar* seeds_ptr = seeds.ptr<uchar>(y);
		for (int x = 0; x < seeds.cols; x++) {
			if (*seeds_ptr > 0) {
				cv::floodFill(flood, mask, Point(x, y), cv::Scalar(255), NULL, cv::Scalar(lofill, lofill, lofill), cv::Scalar(upfill, upfill, upfill), 4 | cv::FLOODFILL_MASK_ONLY | (fillValue << 8));
				//cv::floodFill(flood, mask, Point(x, y), cv::Scalar(255), NULL, cv::Scalar(), cv::Scalar(), 4 | cv::FLOODFILL_MASK_ONLY | (fillValue << 8));
			}			
			seeds_ptr++;
		}
	}

	imshow("flood_edge_mask", mask);
	
	 
	white_mask.setTo(Scalar(255), Mat(mask==128, Rect(Point(1, 1), sz)));
	//imshow("white_mask", white_mask);
	*outputMask = white_mask;

	//if (!red_mask.empty() && !flood.empty()) {
	//	flood += .5 * red_mask;
	//	imshow("flood", flood);
	//}

	//Mat inter = Mat::zeros(image.rows, image.cols, CV_8UC1);;
	//bitwise_and(flood_mask, wholeRoad, inter);
	//if (!white_mask.empty()) {
	//	//flood_mask += .01 * white_mask - .01 * green;
	//	imshow("flood_mask", flood_mask * 100);
	//	imshow("inter", inter);

	//}

	//red_mask.setTo(Scalar(0, 0, 255), flood_mask);

	//flood_fill = image + .5 * red_mask;
	//imshow("flood_fill", flood_fill);
}





//=================================================================
//=================================================================
//=================================================================
//=================================================================

/**
	roadEstimate
*/
void road::roadEstimate(Mat& frame, Mat& background, Mat& mask, int& frame_num, cvb::CvTracks& tracks) {
	Mat intr, rg_chrome;
	Mat merged_prob = Mat::zeros(background.rows, background.cols, CV_32FC1);

	//-- select the correct part of the foreground mask
	Mat croppedMask;
	if (tracks.empty())
		accForegroundMask += mask;
	else {
		cropMask(frame, mask, tracks, move_length, &croppedMask);
		accForegroundMask += croppedMask;
	}
	//imshow("accForegroundMask", accForegroundMask);

	totalMotionMask += mask;

	//-- generate road probablity map
	probabiltyMapGenerator(background, accForegroundMask, &merged_prob);
	threshold(merged_prob, merged_binary, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	//imshow("merged_binary", merged_binary);
	//imwrite("./results/merged_binary.png", merged_binary);

	//-- add P_all with P_F (only in case of using P_all)
	Mat added, pf_filtered, merged_filtered = Mat::zeros(pf.size(), CV_8UC1);
	imshow("pf", pf);
	vector<vector<Point>> contours = findBiggestContour(merged_binary, 1);
	if (contours.size()) {
		drawContours(merged_filtered, contours, 0, Scalar(255), CV_FILLED);
		dilate(merged_filtered, merged_filtered, Mat(), Point(-1, -1), 2, 1, 1);
		imshow("merged_filtered", merged_filtered);
	}
	bitwise_and(merged_filtered, pf, pf_filtered);
	//imshow("pf_filtered", pf_filtered);

	add(merged_binary, pf, added);

	//-- correct the probability maps by foreground mask for final ouput
	Mat P_M = Mat::zeros(background.rows, background.cols, CV_8UC1);;
	accForegroundMask.copyTo(P_M);
	//totalMotionMask.copyTo(P_M);
	int dilation_size = 15;
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * dilation_size + 1, 2 * dilation_size + 1), Point(dilation_size, dilation_size));
	dilate(P_M, P_M, element);
	if (!P_M.empty()) imshow("P_M", P_M);
	Mat temp;
	bitwise_and(merged_binary, P_M, temp);
	float intersection_coef = countNonZero(temp) / (float)countNonZero(merged_binary);
	if (intersection_coef < .8) // filter the road map only after a while
		//merged_binary.copyTo(roadProbEst);
		added.copyTo(roadProbEst);
	else 
		bitwise_and(added, P_M, roadProbEst);

	//	


	////--------------------------------------
	////-- performance measure
	////--------------------------------------

	////Mat gt = imread("C:/Users/Hadi/Desktop/road/figure/expr/illumination/gt6.png",IMREAD_GRAYSCALE);
	//Mat gt = imread("C:/Users/Hadi/Desktop/road/figure/expr/regular/gt1.png",IMREAD_GRAYSCALE);
	//resize(gt, gt, roadProbEst.size());
	//performance(roadProbEst, gt, frame_num);

	////--------------------------------------
	////-- just representation
	////--------------------------------------
	imwrite("./results/roadProbEst.png", roadProbEst);
	//-- show detected road
	background.copyTo(road_represent);
	add(road_represent, Scalar(0, 0, 120), road_represent, roadProbEst);
	imshow("road_represent", road_represent);
}



/**
	probabiltyMapGenerator
	(
		based on papers:
		(1) Road detection based on illuminant invariance and quadratic estimation
		(2) Road extraction algorithm based on intrinsic image and vanishing point for unstructured road image
		(3) Road Detection Based on Illuminant Invariance
		(4) Color Model-Based Real-Time Learning for Road Following
	)
*/
void road::probabiltyMapGenerator(Mat& input, Mat& mask, Mat* merged_prob) {

	Mat inputGray, inputBlurred, inputHsv, inputGrayBlurred, R, G, B, BBlur, GBlur, RBlur, H, HBlur, S, SBlur, V, all, allBlur;
	cvtColor(input, inputGray, CV_BGR2GRAY);
	cvtColor(input, inputHsv, CV_BGR2HSV);

	//-- split bg
	vector<Mat> channels(3), channels1(3), chs;
	split(input, channels);
	B = channels[0];
	G = channels[1];
	R = channels[2];
	split(inputHsv, channels1);
	H = channels1[0];
	S = channels1[1];
	V = channels1[2];

	chs.push_back(B);
	chs.push_back(G);
	chs.push_back(inputGray);
	chs.push_back(H);
	merge(chs, all);

	//-- blur the images
	GaussianBlur(inputGray, inputGrayBlurred, Size(3, 3), 0, 0);
	GaussianBlur(input, inputBlurred, Size(3, 3), 0, 0);
	GaussianBlur(B, BBlur, Size(3, 3), 0, 0);
	GaussianBlur(G, GBlur, Size(3, 3), 0, 0);
	GaussianBlur(R, RBlur, Size(3, 3), 0, 0);
	GaussianBlur(H, HBlur, Size(9, 9), 0, 0);
	GaussianBlur(S, SBlur, Size(3, 3), 0, 0);
	GaussianBlur(all, allBlur, Size(3, 3), 0, 0);

	//-- take average and stddev of color values in input at the location of the mask
	Scalar tempVal, tempstd;
	meanStdDev(inputGrayBlurred, tempVal, tempstd, mask);
	float inputGrayBlurMean = tempVal.val[0];
	float inputGrayBlurStd = tempstd.val[0];
	meanStdDev(inputBlurred, tempVal, tempstd, mask);
	float inputBlurMean = tempVal.val[0];
	float inputBlurStd = tempstd.val[0];
	meanStdDev(BBlur, tempVal, tempstd, mask);
	float BBlurMean = tempVal.val[0];
	float BBlurStd = tempstd.val[0];
	meanStdDev(GBlur, tempVal, tempstd, mask);
	float GBlurMean = tempVal.val[0];
	float GBlurStd = tempstd.val[0];
	meanStdDev(RBlur, tempVal, tempstd, mask);
	float RBlurMean = tempVal.val[0];
	float RBlurStd = tempstd.val[0];
	meanStdDev(HBlur, tempVal, tempstd, mask);
	float HBlurMean = tempVal.val[0];
	float HBlurStd = tempstd.val[0];
	meanStdDev(SBlur, tempVal, tempstd, mask);
	float SBlurMean = tempVal.val[0];
	float SBlurStd = tempstd.val[0];
	meanStdDev(allBlur, tempVal, tempstd, mask);
	float sum1 = 0.0, sum2 = 0.0;
	for (int i = 0; i < tempVal.rows; i++) {
		sum1 += tempVal.val[i];
		sum2 += tempstd.val[i];
	}
	float allBlurMean = sum1 / tempVal.rows;
	float allBlurStd = sum2 / tempstd.rows;

	//cout << "std of gray: " << inputGrayBlurStd << ", rgb: " << inputBlurStd << ", b: " << BBlurStd << ", g: " << GBlurStd << ", r: " << RBlurStd << ", h: " << HBlurStd << ", all: " << allBlurStd << endl;

	//-- calculate G_inv in (1) which is the same as D_tri in (2)
	Mat G_inv = Mat::zeros(input.rows, input.cols, CV_32FC1);
	absdiff(inputGrayBlurred, inputGrayBlurMean, G_inv);
	//imshow("G_inv", G_inv);

	//-- calculate C_inv
	Mat Bdiff, Gdiff, Rdiff;
	absdiff(BBlur, BBlurMean, Bdiff);
	absdiff(GBlur, GBlurMean, Gdiff);
	absdiff(RBlur, RBlurMean, Rdiff);
	Mat C_inv = Mat::zeros(input.rows, input.cols, CV_32FC1);
	C_inv = Bdiff + Gdiff + Rdiff;
	//imshow("C_inv", C_inv);

	//-- calculate H_inv
	Mat H_inv, Hdiff, HdiffOp;
	absdiff(HBlur, HBlurMean, Hdiff);
	HdiffOp = 360 - Hdiff;
	H_inv = Mat::zeros(input.rows, input.cols, CV_32FC1);
	Hdiff.copyTo(H_inv, Mat(Hdiff <= 180));
	HdiffOp.copyTo(H_inv, Mat(Hdiff > 180));
	//imshow("H_inv", H_inv);

	//-- calculate all_inv
	Mat all_inv = Mat::zeros(input.rows, input.cols, CV_32FC1);
	all_inv = (Bdiff/4) + (Gdiff/4) + (Hdiff/4) + (G_inv/4);
	//imshow("all_inv", all_inv);


	Mat Bdiff2, Gdiff2, Hdiff2, G_inv2;
	Bdiff.convertTo(Bdiff2, CV_32F);
	Gdiff.convertTo(Gdiff2, CV_32F);
	Hdiff.convertTo(Hdiff2, CV_32F);
	G_inv.convertTo(G_inv2, CV_32F);
	Mat BdiffPow = Mat::zeros(input.rows, input.cols, CV_32FC1);
	Mat GdiffPow = Mat::zeros(input.rows, input.cols, CV_32FC1);
	Mat HdiffPow = Mat::zeros(input.rows, input.cols, CV_32FC1);
	Mat G_invfPow = Mat::zeros(input.rows, input.cols, CV_32FC1);
	Mat sumPow = Mat::zeros(input.rows, input.cols, CV_32FC1);
	Mat eucDis = Mat::zeros(input.rows, input.cols, CV_32FC1);
	pow(Bdiff2, 2, BdiffPow);
	pow(Gdiff2, 2, GdiffPow);
	pow(Hdiff2, 2, HdiffPow);
	pow(G_inv2, 2, G_invfPow);
	//sumPow = BdiffPow + GdiffPow + HdiffPow + G_invfPow;
	sumPow = (BdiffPow/ BBlurStd* BBlurStd) + (GdiffPow/ GBlurStd* GBlurStd) + (HdiffPow/ HBlurStd* HBlurStd) + (G_invfPow/ inputGrayBlurStd* inputGrayBlurStd);
	sqrt(sumPow, eucDis);

	Mat all_inv_euc;
	eucDis.convertTo(all_inv_euc, CV_8U);
	//imshow("all_inv_euc", all_inv_euc);

	//---------------------------------------------------------------
	//-- calculate probability maps
	//---------------------------------------------------------------
		//-- probability maps in (1)
	double minVal, maxVal;
	minMaxLoc(G_inv, &minVal, &maxVal);
	Mat max_mat = Mat::zeros(input.rows, input.cols, CV_8UC1);
	max_mat.setTo(maxVal, Mat(G_inv > 0));
	Mat P_G = 256 * (1 - G_inv / maxVal);
	minMaxLoc(C_inv, &minVal, &maxVal);
	Mat P_C = 256 * (1 - C_inv / maxVal);
	minMaxLoc(H_inv, &minVal, &maxVal);
	Mat P_H = 256 * (1 - H_inv / maxVal);
	minMaxLoc(all_inv, &minVal, &maxVal);
	//Mat P_all = 256 * (1 - all_inv / maxVal);
	equalizeHist(P_G, P_G);
	equalizeHist(P_C, P_C);
	equalizeHist(P_H, P_H);
	//equalizeHist(P_all, P_all);
	//imshow("P_G", P_G);
	//imshow("P_C", P_C);
	//imshow("P_H", P_H);
	//imshow("P_all", P_all);

	//---------------------------------------

	//-- gray-scale difference probability maps in (2)
	Mat P_S = Mat::zeros(input.rows, input.cols, CV_8UC1);

	Mat tmp1, tmp2, tmp3, tmp4, mask1, mask2, mask3;
	tmp1 = 255 * (1.5 - (0.5 * G_inv) / inputGrayBlurStd);
	tmp2 = 255 * (1.1 - (0.3 * G_inv) / inputGrayBlurStd);
	tmp3 = .2 * (max_mat - G_inv) / (maxVal - 3 * inputGrayBlurStd);
	multiply(tmp3, G_inv, tmp4, 255);
	inRange(G_inv, inputGrayBlurStd, 2 * inputGrayBlurStd, mask1);
	inRange(G_inv, 2 * inputGrayBlurStd, 3 * inputGrayBlurStd, mask2);
	inRange(G_inv, 3 * inputGrayBlurStd, 255, mask3);

	P_S.setTo(255, Mat(G_inv <= inputGrayBlurStd));
	tmp1.copyTo(P_S, mask1);
	tmp2.copyTo(P_S, mask2);
	tmp3.copyTo(P_S, mask3);
	imshow("P_S", P_S);


	//-- 4D feature (all) difference probability map
	Mat P_all = Mat::zeros(input.rows, input.cols, CV_8UC1);

	//Mat tmp1, tmp2, tmp3, tmp4, mask1, mask2, mask3;
	tmp1 = 255 * (1.5 - (0.5 * all_inv_euc) / allBlurStd);
	tmp2 = 255 * (1.1 - (0.3 * all_inv_euc) / allBlurStd);
	tmp3 = .2 * (max_mat - all_inv_euc) / (maxVal - 3 * allBlurStd);
	multiply(tmp3, G_inv, tmp4, 255);
	inRange(all_inv_euc, allBlurStd, 2 * allBlurStd, mask1);
	inRange(all_inv_euc, 2 * allBlurStd, 3 * allBlurStd, mask2);
	inRange(all_inv_euc, 3 * allBlurStd, 255, mask3);

	P_all.setTo(255, Mat(all_inv_euc <= allBlurStd));
	tmp1.copyTo(P_all, mask1);
	tmp2.copyTo(P_all, mask2);
	tmp3.copyTo(P_all, mask3);
	imshow("P_all", P_all);

	
	//---------------------------------------

	//-- histogram maps in (3) and (4)
	int histSize = 256; // number of bins
	float range[] = { 0, 256 };
	const float* histRange = { range };
	bool uniform = true, accumulate = false;
	Mat b_hist, g_hist, r_hist, gray_hist, h_hist;
	Mat bn_hist, gn_hist, rn_hist, grayn_hist;
	Mat P_bhist = Mat::zeros(input.rows, input.cols, CV_8UC1);
	Mat P_ghist = Mat::zeros(input.rows, input.cols, CV_8UC1);
	Mat P_rhist = Mat::zeros(input.rows, input.cols, CV_8UC1);
	Mat P_hhist = Mat::zeros(input.rows, input.cols, CV_8UC1);
	Mat P_gbhist = Mat::zeros(input.rows, input.cols, CV_8UC1);
	Mat P_grayhist = Mat::zeros(input.rows, input.cols, CV_8UC1);

	calcHist(&B, 1, 0, mask, b_hist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&G, 1, 0, mask, g_hist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&R, 1, 0, mask, r_hist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&H, 1, 0, mask, h_hist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&inputGray, 1, 0, mask, gray_hist, 1, &histSize, &histRange, uniform, accumulate);

	//-- method of (4)
	if (!merged_binary.empty()) {
		Mat nonRoad, nonForeground;
		bitwise_not(accForegroundMask, nonForeground);
		bitwise_not(merged_binary, nonRoad, nonForeground);

		calcHist(&B, 1, 0, nonRoad, bn_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&G, 1, 0, nonRoad, gn_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&R, 1, 0, nonRoad, rn_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&inputGray, 1, 0, nonRoad, grayn_hist, 1, &histSize, &histRange, uniform, accumulate);

		int lambda = 80, bin = 0; // percent
		for (int i = 0; i < input.cols; i++) {
			for (int j = 0; j < input.rows; j++) {
				bin = static_cast<int>(B.at<uchar>(j, i));
				//cout << b_hist.at<float>(bin) << "\t" << bn_hist.at<float>(bin) << endl;
				P_bhist.at<uchar>(j, i) = 255* ((b_hist.at<float>(bin)) / ((b_hist.at<float>(bin)) + (bn_hist.at<float>(bin))));
				bin = static_cast<int>(G.at<uchar>(j, i));
				P_ghist.at<uchar>(j, i) = 255 * ((g_hist.at<float>(bin)) / ((g_hist.at<float>(bin)) + (gn_hist.at<float>(bin))));
				bin = static_cast<int>(R.at<uchar>(j, i));
				P_rhist.at<uchar>(j, i) = 255 * ((r_hist.at<float>(bin)) / ((r_hist.at<float>(bin)) + (rn_hist.at<float>(bin))));
				bin = static_cast<int>(inputGray.at<uchar>(j, i));
				P_grayhist.at<uchar>(j, i) = 255 * ((gray_hist.at<float>(bin)) / ((gray_hist.at<float>(bin)) + (grayn_hist.at<float>(bin))));
			}
		}

		P_gbhist = P_bhist + P_ghist;

		//imshow("P_bhist", P_bhist);
		//imshow("P_ghist", P_ghist);
		//imshow("P_rhist", P_rhist);
		//imshow("added", P_gbhist);
		//imshow("P_grayhist", P_grayhist);
	}
	
	//---------------------------------------

	//-- flood fill
		//-- update flood based on paper: Color Model-Based Real-Time Learning for Road Following - part (C)
	short max_w = 100;
	Mat G_inv_colored, frame_flood;
	cvtColor(G_inv, G_inv_colored, CV_GRAY2BGR);
	//floodFillFromSeedMask(G_inv_colored, mask, &frame_flood);
	//floodFillFromSeedMask(G_inv_colored, flood_seeds, &frame_flood);
	floodFillFromSeedMask(G_inv_colored, totalMotionMask, &frame_flood);
	P_flood += ((w_flood * P_flood) + frame_flood) / (w_flood + 1);
	if (w_flood <= max_w) w_flood++;
	else w_flood = max_w;
	Mat P_F;
	P_flood.copyTo(P_F);
	bitwise_and(P_F, P_S, P_F);
	if(!P_F.empty()) imshow("P_F", P_F);




	//---------------------------------------------------------------
	//-- merge probability maps (based on paper (1) section 3-2)
	//---------------------------------------------------------------
		//-- list of weights
	vector<int> listWeights;
	int W_G = 1, W_C = 1, W_H = 1, W_D = 2, W_F = 3, W_gbhist = 1, W_grayhist = 1;
	listWeights.push_back(W_G);
	listWeights.push_back(W_C);
	listWeights.push_back(W_H);
	listWeights.push_back(W_D);
	listWeights.push_back(W_F);
	if (!P_grayhist.empty()) {
		listWeights.push_back(W_gbhist);
		listWeights.push_back(W_grayhist);
	}
	int total_sum = std::accumulate(listWeights.begin(), listWeights.end(), 0);

	//-- just representation
	ginv = G_inv*3;
	cinv = C_inv*3;
	hinv = H_inv*3;
	allinv = all_inv_euc.clone();
	pg = P_G.clone();
	pc = P_C.clone();
	ph = P_H.clone();
	ps = P_S.clone();
	pf = P_flood.clone();
	pgbhist = P_gbhist.clone();
	pgrayhist = P_grayhist.clone();
	pall = P_all.clone();

	
	vector<Mat> listImages;
	listImages.push_back(P_G);
	listImages.push_back(P_C);
	listImages.push_back(P_H);
	listImages.push_back(P_S);
	listImages.push_back(P_F);
	listImages.push_back(P_gbhist);
	listImages.push_back(P_grayhist);

		//-- calculate the mean of all prob maps
	Mat allMean = Mat::zeros(input.rows, input.cols, CV_8UC1);
	for (int i = 0; i < listImages.size(); i++) // weighted average
		allMean += (listWeights[i] * (listImages[i] / total_sum));

		//-- calculate the median of all prob maps
	Mat tmp;
	for (int i = 0; i < listImages.size(); i++) { // sort the images pixel-wise
		for (int j = i + 1; j < listImages.size(); j++) {
			listImages[i].copyTo(tmp);
			min(listImages[i], listImages[j], listImages[i]);
			max(listImages[j], tmp, listImages[j]);
		}
	}
	Mat medianProb;
	if(listImages.size() % 2 != 0)
		medianProb = listImages[listImages.size() / 2];
	else
		medianProb = 0.5 * (listImages[listImages.size() / 2] + listImages[(listImages.size() / 2) + 1]);
		
		//-- calculate merged probability
	Mat highMean, lowMean, temp1;
	int half_size = ((listImages.size() / 2) + 1);
	temp1 = medianProb / half_size;
	temp1.copyTo(highMean);
	temp1.copyTo(lowMean);
	for (int i = (listImages.size() % 2) + (listImages.size() / 2); i < listImages.size(); i++)
		highMean += listImages[i] / half_size;
	for(int i = 0; i < (listImages.size() / 2); i++)
		lowMean += listImages[i] / half_size;

		//-- equation 2 of paper (1)
	int t1 = 9, t2 = 6, t3 = 9;
	allMean.copyTo(*merged_prob);
	highMean.copyTo(*merged_prob, Mat(medianProb > (t1*255/10)));
	Mat tmp5, tmp6, tmp7;
	lowMean.copyTo(tmp5, Mat(listImages[listImages.size()-1] < (t3*255/10)));
	lowMean.copyTo(tmp6, Mat(allMean < (t2*255/10)));
	bitwise_and(tmp5, tmp6, tmp7);
	lowMean.copyTo(*merged_prob, tmp7);
	medianBlur(*merged_prob, *merged_prob, 3);
	//imshow("merged", *merged_prob);



	//==========================================================================

	//-- if want to use P_all instead of merged
	//P_all.copyTo(*merged_prob);
	//Mat drawing = Mat::zeros(input.rows,input.cols,CV_8UC3);

	//vector<vector<Point>> contours = findBiggestContour(*merged_prob, 1);
	//if (contours.size() > 0) {
	//	drawContours(drawing, contours, 0, Scalar(255, 0, 0), CV_FILLED);
	//	imshow("drawing", drawing);
	//}
	

	//int morph_size = 3;
	//Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
	//morphologyEx(*merged_prob,  *merged_prob, MORPH_OPEN, element);
	//morph_size = 5;
	//element = getStructuringElement(MORPH_ELLIPSE, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
	//morphologyEx(*merged_prob, *merged_prob, MORPH_CLOSE, element);


	//------------------------------------

	//Mat bin;
	//threshold(*merged_prob, bin, 50, 255, THRESH_BINARY | THRESH_OTSU);

	//// A image with size greater than the present object is created, it is needed from floodFill()
	//cv::Mat msk = cv::Mat::zeros(input.rows + 2, input.cols + 2, CV_8U);

	//cv::floodFill(bin, msk, cv::Point(0, 0), 255, 0, cv::Scalar(), cv::Scalar(), 4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
	////NOTE Since the mask is larger than the filled image, a pixel  (x, y) in image corresponds to the pixel (x+1, y+1) in the mask .

	////remove the extra rows/cols added earlier in the initialization of the mask, if you want of course it is just "optional"
	//Mat dst;
	//msk(Range(1, msk.rows - 1), Range(1, msk.cols - 1)).copyTo(dst);

	//imshow("dst", dst);

	//------------------------------------

	//int const smooth_kernel = 11;
	//Mat srcSmooth, srcThresh;

	//for (int i = 1; i < smooth_kernel; i = i + 2)
	//{
	//	medianBlur(*merged_prob, srcSmooth, i);
	//}

	//threshold(srcSmooth, srcThresh, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

	////retrieve only external contours (also you can retrieve list and use hierarchy)
	//vector<vector<Point>> contours;
	//findContours(srcThresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	//Mat result(input.rows, input.cols, CV_8UC1, Scalar(0));
	//drawContours(result, contours, 0, Scalar(255), -1);     //draw with filling

	//imshow("Final", result);

	//==========================================================================



	
	

	//-------------------------------------------------------
	//-- just representation
	//-------------------------------------------------------
	merged = *merged_prob;


	//-- plot histograms
	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound((double)hist_w / histSize);
	int bin_h = cvRound((double)hist_h / histSize);

	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(255, 255, 255));

	normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	normalize(h_hist, h_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	normalize(gray_hist, gray_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

		//-- draw one way
	for (int i = 1; i < histSize; i++)
	{
		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
			Scalar(255, 0, 0), 2, 8, 0);
		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
			Scalar(0, 255, 0), 2, 8, 0);
		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
			Scalar(0, 0, 255), 2, 8, 0);
		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(gray_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(gray_hist.at<float>(i))),
			Scalar(100, 100, 100), 2, 8, 0);
		//line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(h_hist.at<float>(i - 1))),
		//	Point(bin_w * (i), hist_h - cvRound(h_hist.at<float>(i))),
		//	Scalar(250, 0, 250), 2, 8, 0);

		//--  draw grid
		if (i % 30 == 0) {
			line(histImage, Point(i * bin_w, 0), Point(i * bin_w, hist_h), Scalar(200, 200, 200), 1, 8, 0);
			line(histImage, Point(0, i * bin_w), Point(hist_w, i * bin_w), Scalar(200, 200, 200), 1, 8, 0);
		}
	}

		//-- draw another way
	//for (int i = 0; i < histSize; i++) {
	//	line(histImage, Point(bin_w * (i), hist_h), Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))), Scalar(255, 0, 0), bin_w, 8, 0);
	//	line(histImage, Point(bin_w * (i), hist_h), Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))), Scalar(0, 255, 0), bin_w, 8, 0);
	//	line(histImage, Point(bin_w * (i), hist_h), Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))), Scalar(0, 0, 255), bin_w, 8, 0);
	//	line(histImage, Point(bin_w * (i), hist_h), Point(bin_w * (i), hist_h - cvRound(gray_hist.at<float>(i))), Scalar(100, 100, 100), bin_w, 8, 0);
	//}
	

	//namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE);
	//imshow("calcHist Demo", histImage);
	//imwrite("./results/histogram_plot.png", histImage);
}


/**
// image median
*/
double road::imgMedian(const cv::Mat& img, const Mat& mask)
{
	//double m = (img.rows * img.cols) / 2;
	double m = countNonZero(mask) / 2;
	int bin = 0;
	double med = -1.0;

	int histSize = 256;
	float range[] = { 0, 256 };
	const float* histRange = { range };
	bool uniform = true;
	bool accumulate = false;
	cv::Mat hist;
	cv::calcHist(&img, 1, 0, mask, hist, 1, &histSize, &histRange, uniform, accumulate);

	for (int i = 0; i < histSize && med < 0.0; ++i)
	{
		bin += cvRound(hist.at< float >(i));
		if (bin > m && med < 0.0)
			med = i;
	}

	return med;
}


/**
	accumulative forground mask
*/
void road::acc_mask(Mat& mask, cvb::CvTracks& tracks, const short& move_length) {
	for (std::map<cvb::CvID, cvb::CvTrack*>::iterator it = tracks.begin(); it != tracks.end(); it++)
	{
		cvb::CvID id = (*it).first;
		cvb::CvTrack* track = (*it).second;
		if (track->width <= 0 || track->height <= 0)
			continue;

		//-- track blob
		Mat track_mask = Mat::zeros(mask.rows, mask.cols, CV_8UC1);
		Rect roi(track->minx, track->miny, track->width, track->height);
		mask(roi).copyTo(track_mask(roi));
		//dilate(track_mask, track_mask, Mat(), Point(-1, -1), 2, 1, 1);
		cv::add(track->motion_mask, track_mask, track->motion_mask, track_mask);

		//-- track move size
		float track_movement = norm(track->trajectory.back() - track->trajectory[0]);

		//-- valid tracks
		if (track->trajectory.size() >= move_length && track->inactive >= move_length && track_movement >= move_length) {
			//-- whole motion area
			cv::add(wholeRoad2, track_movement, wholeRoad2, track->motion_mask);
		}
	}

	//-- normalize
	double minVal, maxVal;
	minMaxLoc(wholeRoad2, &minVal, &maxVal);
	Mat msk = Mat::zeros(mask.rows, mask.cols, CV_8UC1);
	wholeRoad2.convertTo(msk, CV_8U);
	double med = imgMedian(wholeRoad2, msk);
	//cout << med << " " << maxVal << endl;
	Mat res;
	res = wholeRoad2.clone();
	res = res / maxVal;
	//res = res / med;

	res.convertTo(wholeRoad3, CV_8U, 255);

	//-- just representation
	Mat rep;
	//add(wholeRoad3, Scalar(120), rep, Mat(wholeRoad3 > 0));
	multiply(wholeRoad3, Scalar(10), rep);
	imshow("whole", rep);
	imwrite("./results/accMask.png", rep);

	int dilation_size = 15;
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * dilation_size + 1, 2 * dilation_size + 1), Point(dilation_size, dilation_size));
	dilate(rep, rep, element);
	imshow("whole_dilate", rep);
	imwrite("./results/accMask_dilate.png", rep);
}



/**
	performance metrics
	result and ground_truth should be both CV_8UC1 masks
*/
void road::performance(Mat& result, Mat& ground_truth, int& frame_num) {
	unsigned long long int fp, tp, fn, tn;
	float fpr, pre, rec, acc, f1, mcc;

	Mat intersection, inverse_result, inverse_gt, inverse_intersection;
	bitwise_and(result, ground_truth, intersection);
	bitwise_not(result, inverse_result);
	bitwise_not(ground_truth, inverse_gt);
	bitwise_and(inverse_result, inverse_gt, inverse_intersection);

	fp = countNonZero(result - ground_truth);
	tp = countNonZero(intersection);
	fn = countNonZero(ground_truth - result);
	tn = countNonZero(inverse_intersection);

	//-----------------------------------------------------
	//	metrics
	//-----------------------------------------------------
	fpr = fp / (float)(fp + tn); // false positive rate
	pre = tp / (float)(tp + fp); // precision
	rec = tp / (float)(tp + fn); // recall
	acc = (tp + tn) / (float)(tp + tn + fp + fn); // accuracy
	f1 = 2 * (pre * rec) / (float)(pre + rec); // f-measure
	mcc = ((unsigned long long int)(tp * tn) - (unsigned long long int)(fp * fn)) / (long double)(sqrt((tp + fp)*(tp + fn)*(tn + fp)*(tn + fn)));

	//-----------------------------------------------------
	//	print metrics
	//-----------------------------------------------------
	//cout << "fpr= " << fpr << "\tpre= " << pre << "\trec= " << rec << "\tacc= " << acc << "\tf1= " << f1 << "\tmcc= " << mcc << endl;

	//-----------------------------------------------------
	//	write metrics to file
	//-----------------------------------------------------
	performance_output.open("./results/performance.txt", std::ios::app);
	performance_output << frame_num << " " << fpr << " " << pre << " " << rec << " " << acc << " " << f1 << " " << mcc << endl;
	performance_output.close();
}


























































































































































//==============================================================================================
//                                        curve functions
//==============================================================================================

/* 1st and 2nd derivative of 1D gaussian
*/
void road::getGaussianDerivs(double sigma, int M, vector<double>& gaussian, vector<double>& dg, vector<double>& d2g) {
	int L = (M - 1) / 2;
	double sigma_sq = sigma * sigma;
	double sigma_quad = sigma_sq * sigma_sq;
	dg.resize(M); d2g.resize(M); gaussian.resize(M);

	Mat_<double> g = getGaussianKernel(M, sigma, CV_64F);
	for (double i = -L; i < L + 1.0; i += 1.0) {
		int idx = (int)(i + L);
		gaussian[idx] = g(idx);
		// from http://www.cedar.buffalo.edu/~srihari/CSE555/Normal2.pdf
		dg[idx] = (-i / sigma_sq) * g(idx);
		d2g[idx] = (-sigma_sq + i * i) / sigma_quad * g(idx);
	}
}

//--------------------------------------------------------------------------------

/* 1st and 2nd derivative of smoothed curve point */
void road::getdX(vector<double> x, // curve
	int n, // index of point on the curve
	double sigma,
	double& gx,
	double& dgx,
	double& d2gx,
	vector<double> g,
	vector<double> dg,
	vector<double> d2g,
	bool isOpen = false)
{
	int L = (g.size() - 1) / 2;

	gx = dgx = d2gx = 0.0;
	//  cout << "Point " << n << ": ";
	for (int k = -L; k < L + 1; k++) {
		double x_n_k;
		if (n - k < 0) {
			if (isOpen) {
				//open curve - mirror values on border
				x_n_k = x[-(n - k)];
			}
			else {
				//closed curve - take values from end of curve
				x_n_k = x[x.size() + (n - k)];
			}
		}
		else if (n - k > x.size() - 1) {
			if (isOpen) {
				//mirror value on border
				x_n_k = x[n + k];
			}
			else {
				x_n_k = x[(n - k) - (x.size())];
			}
		}
		else {
			//          cout << n-k;
			x_n_k = x[n - k];
		}
		//      cout << "* g[" << g[k + L] << "], ";

		gx += x_n_k * g[k + L]; //gaussians go [0 -> M-1]
		dgx += x_n_k * dg[k + L];
		d2gx += x_n_k * d2g[k + L];
	}
	//  cout << endl;
}

//--------------------------------------------------------------------------------

/* 0th, 1st and 2nd derivatives of whole smoothed curve */
void road::getdXcurve(vector<double> x,
	double sigma,
	vector<double>& gx,
	vector<double>& dx,
	vector<double>& d2x,
	vector<double> g,
	vector<double> dg,
	vector<double> d2g,
	bool isOpen = false)
{
	gx.resize(x.size());
	dx.resize(x.size());
	d2x.resize(x.size());
	for (int i = 0; i < x.size(); i++) {
		double gausx, dgx, d2gx;
		getdX(x, i, sigma, gausx, dgx, d2gx, g, dg, d2g, isOpen);
		gx[i] = gausx;
		dx[i] = dgx;
		d2x[i] = d2gx;
	}
}

//--------------------------------------------------------------------------------

template<typename T, typename V>
void road::PolyLineSplit(const vector<Point_<T> >& pl, vector<V>& contourx, vector<V>& contoury) {
	contourx.resize(pl.size());
	contoury.resize(pl.size());

	for (int j = 0; j < pl.size(); j++)
	{
		contourx[j] = (V)(pl[j].x);
		contoury[j] = (V)(pl[j].y);
	}
}

//--------------------------------------------------------------------------------

template<typename T, typename V>
void road::PolyLineMerge(vector<Point_<T> >& pl, const vector<V>& contourx, const vector<V>& contoury) {
	assert(contourx.size() == contoury.size());
	pl.resize(contourx.size());
	for (int j = 0; j < contourx.size(); j++) {
		pl[j].x = (T)(contourx[j]);
		pl[j].y = (T)(contoury[j]);
	}
}

//--------------------------------------------------------------------------------

void road::smoothen(Mat& frame, Mat& contourImg, std::vector<std::vector<cv::Point>> biggest_contours, cv::Mat* smoothed_curve) {

	*smoothed_curve = Mat::zeros(frame.rows, frame.cols, CV_8UC3);
	vector<Point> curve;

	for (size_t i = 0; i < biggest_contours.size(); i++) {
		for (size_t j = 0; j < biggest_contours[i].size(); j++) {
			curve.push_back(biggest_contours[i][j]);
		}

		double sigma = 3.0;
		int M = round((10.0 * sigma + 1.0) / 2.0) * 2 - 1;
		assert(M % 2 == 1); //M is an odd number

		//create kernels
		vector<double> g, dg, d2g;

		getGaussianDerivs(sigma, M, g, dg, d2g);

		vector<double> curvex, curvey, smoothx, smoothy;
		PolyLineSplit(curve, curvex, curvey);

		vector<double> X, XX, Y, YY;
		getdXcurve(curvex, sigma, smoothx, X, XX, g, dg, d2g, isOpen);
		getdXcurve(curvey, sigma, smoothy, Y, YY, g, dg, d2g, isOpen);

		PolyLineMerge(curve, smoothx, smoothy);

		cv::polylines(*smoothed_curve, curve, isOpen, Scalar(0, 100, 255), 2);

		curve.clear();
	}

}

//--------------------------------------------------------------------------------

void road::ResampleCurve(const vector<double>& curvex, const vector<double>& curvey,
	vector<double>& resampleX, vector<double>& resampleY,
	int N,
	bool isOpen
) {
	assert(curvex.size() > 0 && curvey.size() > 0 && curvex.size() == curvey.size());

	vector<Point2d> resamplepl(N); resamplepl[0].x = curvex[0]; resamplepl[0].y = curvey[0];
	vector<Point2i> pl;

	PolyLineMerge(pl, curvex, curvey);

	double pl_length = arcLength(pl, false);
	double resample_size = pl_length / (double)N;
	int curr = 0;
	double dist = 0.0;
	for (int i = 1; i < N; ) {
		assert(curr < pl.size() - 1);
		double last_dist = norm(pl[curr] - pl[curr + 1]);
		dist += last_dist;
		//      cout << curr << " and " << curr+1 << "\t\t" << last_dist << " ("<<dist<<")"<<endl;
		if (dist >= resample_size) {
			//put a point on line
			double _d = last_dist - (dist - resample_size);
			Point2d cp(pl[curr].x, pl[curr].y), cp1(pl[curr + 1].x, pl[curr + 1].y);
			Point2d dirv = cp1 - cp; dirv = dirv * (1.0 / norm(dirv));
			//          cout << "point " << i << " between " << curr << " and " << curr+1 << " remaining " << dist << endl;
			assert(i < resamplepl.size());
			resamplepl[i] = cp + dirv * _d;
			i++;

			dist = last_dist - _d; //remaining dist         

			//if remaining dist to next point needs more sampling... (within some epsilon)
			while (dist - resample_size > 1e-3) {
				//              cout << "point " << i << " between " << curr << " and " << curr+1 << " remaining " << dist << endl;
				assert(i < resamplepl.size());
				resamplepl[i] = resamplepl[i - 1] + dirv * resample_size;
				dist -= resample_size;
				i++;
			}
		}

		curr++;
	}

	PolyLineSplit(resamplepl, resampleX, resampleY);
}



//--------------------------------------------------------------------------------


void road::polyfit(const Mat& src, const vector<double>& x, const vector<double>& y, vector<Point>& final_curve) {

	int N = x.size();
	const int n = 2;							// degree
	int i, j, k, m;
	//cout.precision(4);                        //set precision in output
	//cout.setf(ios::fixed);
	std::vector<double> est_x, est_y;

	double X[2 * n + 1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	for (i = 0; i < 2 * n + 1; i++)
	{
		X[i] = 0;
		for (j = 0; j < N; j++)
			X[i] = X[i] + pow(x[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	}
	double B[n + 1][n + 2], a[n + 1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
	for (i = 0; i <= n; i++)
		for (j = 0; j <= n; j++)
			B[i][j] = X[i + j];            //Build the Normal matrix by storing the corresponding coefficients at the up positions except the last column of the matrix
	double Y[n + 1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	for (i = 0; i < n + 1; i++)
	{
		Y[i] = 0;
		for (j = 0; j < N; j++)
			Y[i] = Y[i] + pow(x[j], i) * y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	}
	for (i = 0; i <= n; i++)
		B[i][n + 1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
	m = n + 1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
	//cout << "\nThe Normal(Augmented Matrix) is as follows:\n";
	//for (i = 0; i < m; i++)            //print the Normal-augmented matrix
	//{
	//	for (j = 0; j <= m; j++)
	//		cout << B[i][j] << setw(16);
	//	cout << "\n";
	//}
	for (i = 0; i < m; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
		for (k = i + 1; k < m; k++)
			if (B[i][i] < B[k][i])
				for (j = 0; j <= m; j++)
				{
					double temp = B[i][j];
					B[i][j] = B[k][j];
					B[k][j] = temp;
				}

	for (i = 0; i < m - 1; i++)            //loop to perform the gauss elimination
		for (k = i + 1; k < m; k++)
		{
			double t = B[k][i] / B[i][i];
			for (j = 0; j <= m; j++)
				B[k][j] = B[k][j] - t * B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
		}
	for (i = m - 1; i >= 0; i--)                //back-substitution
	{                        //x is an array whose values correspond to the values of x,y,z..
		a[i] = B[i][m];                //make the variable to be calculated equal to the rhs of the last equation
		for (j = 0; j < m; j++)
			if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value is being calculated
				a[i] = a[i] - B[i][j] * a[j];
		a[i] = a[i] / B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
	}
	//cout << "\nThe values of the coefficients are as follows:\n";
	//for (i = 0; i < m; i++)
	//	cout << "x^" << i << "=" << a[i] << endl;            // Print the values of x^0,x^1,x^2,x^3,....    
	//cout << "\nHence the fitted Polynomial is given by:\ny=";
	//for (i = 0; i < m; i++)
	//	cout << " + (" << a[i] << ")" << "x^" << i;
	//cout << "\n" << "=========================================================" << endl;




	//cout << "x\ty\test" << endl;
	for (int j = 0; j < N; j++) {
		double est = 0.0;
		//cout << x[j] << "\t" << y[j] << "\t";
		for (i = 0; i < m; i++) {
			est += a[i] * pow(x[j], i);
		}
		//cout << est << endl;
	}


	//cout << "x\test" << endl;
	for (int j = 0; j < src.cols; j++) {
		//cout << j << "\t";
		double est = 0.0;
		for (i = 0; i < m; i++) {
			est += a[i] * pow(j, i);
		}
		//cout << est << endl;
		final_curve.push_back(Point(j, est));
	}

}




//--------------------------------------------------------------------------------

void road::curve_fit(const Mat& frame, const Mat& background, Mat& contourImg, std::vector<std::vector<cv::Point>> contours, cv::Mat* output_boundaries) {

	//*output_boundaries = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);
	background.copyTo(*output_boundaries);
	//-----------------------
	/// cut surrounding of contours
	//-----------------------

	int cut_width = 5;
	int cut_height = 15;
	if (contours.size() > 0) {


		int minx = 0, miny = 0, maxx = 0, maxy = 0;
		vector<int> minValx, maxValx, minValy, maxValy;
		for (size_t i = 0; i < contours.size(); i++) {
			cv::Rect rect = cv::boundingRect(contours[i]);

			// tl() directly equals to the desired min. values
			minValx.push_back(rect.tl().x);
			minValy.push_back(rect.tl().y);

			// br() is exclusive so we need to subtract 1 to get the max. values
			//maxVal.push_back(rect.br() - cv::Point(1, 1));
			maxValx.push_back(rect.br().x - 1);
			maxValy.push_back(rect.br().y - 1);
		}

		minx = *min_element(minValx.begin(), minValx.end());
		miny = *min_element(minValy.begin(), minValy.end());
		maxx = *max_element(maxValx.begin(), maxValx.end());
		maxy = *max_element(maxValy.begin(), maxValy.end());

		Mat cut_contours = Mat::zeros(contourImg.rows, contourImg.cols, CV_8UC3);
		cut_contours(Range(miny + cut_height, maxy - cut_height), Range(minx + cut_width, maxx - cut_width)) += contourImg(Range(miny + cut_height, maxy - cut_height), Range(minx + cut_width, maxx - cut_width));
		//cv::imshow("cut_contours", cut_contours);
		//cv::imwrite("./results/cut_contours.png", cut_contours);


		//-----------------------
		/// clustering
		//-----------------------
		Mat gray_contours;
		cv::cvtColor(cut_contours, gray_contours, CV_BGR2GRAY);

		// Get all non black points
		vector<Point> contour_pts;
		findNonZero(gray_contours, contour_pts);

		// Define the radius tolerance
		int th_distance = 5; // radius tolerance

		// Apply partition 
		// All pixels within the radius tolerance distance will belong to the same class (same label)
		vector<int> labels;

		// With functor
		//int n_labels = partition(contour_pts, labels, EuclideanDistanceFunctor(th_distance));

		// With lambda function (require C++11)
		int th2 = th_distance * th_distance;
		int n_labels = partition(contour_pts, labels, [th2](const Point& lhs, const Point& rhs) {
			return ((lhs.x - rhs.x) * (lhs.x - rhs.x) + (lhs.y - rhs.y) * (lhs.y - rhs.y)) < th2;
			});

		// You can save all points in the same class in a vector (one for each class), just like findContours
		vector<vector<Point>> contour_lines(n_labels);
		for (int i = 0; i < contour_pts.size(); ++i)
		{
			contour_lines[labels[i]].push_back(contour_pts[i]);
		}

		// Draw results

		// Build a vector of random color, one for each class (label)
		vector<Vec3b> colors;
		for (int i = 0; i < n_labels; ++i)
		{
			colors.push_back(Vec3b(rand() & 255, rand() & 255, rand() & 255));
		}

		// Draw the labels
		Mat3b lbl(frame.size(), Vec3b(0, 0, 0));
		for (int i = 0; i < contour_pts.size(); ++i)
		{
			lbl(contour_pts[i]) = colors[labels[i]];
		}

		cv::imshow("Labels", lbl);
		cv::imwrite("./results/clusters.png", lbl);


		//-----------------------
		/// resampling and curve fitting
		//-----------------------
		Mat resample, pca;
		RNG rng(12345);
		background.copyTo(resample);
		background.copyTo(pca);
		Mat curve_drawing2 = Mat::zeros(contourImg.size(), CV_8UC3);
		vector<Point> resample_curve, estimate_curve, final_curve;
		//vector<double> estimatedX, estimatedY;
		vector<int> estimatedX, estimatedY;
		for (size_t i = 0; i < contour_lines.size(); i++) {
			vector<double> curvex, curvey, resampleX, resampleY;
			PolyLineSplit(contour_lines[i], curvex, curvey);

			if (curvex.size() < 20) {
				resampleX = curvex;
				resampleY = curvey;
			}
			else {
				//ResampleCurve(curvex, curvey, resampleX, resampleY, arcLength(contour_lines[labels[i]], isOpen) / 100, isOpen);
				ResampleCurve(curvex, curvey, resampleX, resampleY, 20, !isOpen);
			}

			Scalar color = Scalar(rng.uniform(50, 220), rng.uniform(180, 255), rng.uniform(220, 255));
			for (int i = 0; i < resampleX.size(); i++) {
				cv::circle(resample, cv::Point(resampleX[i], resampleY[i]), 3, color, CV_FILLED, 8);
			}

			//=======================
			// pca
			//=======================
			vector<Point> border_points;
			PolyLineMerge(border_points, resampleX, resampleY);
			pca_curve(border_points, pca, final_curve);
			//-----------------------

			//=======================
			// non-pca
			//=======================
			//polyfit(bg, resampleX, resampleY, final_curve);
			//-----------------------

			cv::polylines(*output_boundaries, final_curve, isOpen, color, 3);

			resample_curve.clear();
			final_curve.clear();
		}

		cv::imshow("curve_drawing", *output_boundaries);
		cv::imshow("resample", resample);
		cv::imwrite("./results/resample.png", resample);
	}
}


//==============================================================================================
//                                    pca functions
// =============================================================================================

/**
	* @function drawAxis
	*/
void road::drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
	//! [visualization1]
	double angle = atan2((double)p.y - q.y, (double)p.x - q.x); // angle in radians
	double hypotenuse = sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));

	// Here we lengthen the arrow by a factor of scale
	q.x = (int)(p.x - scale * hypotenuse * cos(angle));
	q.y = (int)(p.y - scale * hypotenuse * sin(angle));
	line(img, p, q, colour, 1, LINE_AA);

	// create the arrow hooks
	p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
	p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
	line(img, p, q, colour, 1, LINE_AA);

	p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
	p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
	line(img, p, q, colour, 1, LINE_AA);
	//! [visualization1]
}

//--------------------------------------------------------------------------------

/**
	* @function getOrientation
*/
void road::pca_curve(const vector<Point>& pts, Mat& img, vector<Point>& output_curve)
{
	//! [pca]
	//Construct a buffer used by the pca analysis
	int sz = static_cast<int>(pts.size());
	Mat data_pts = Mat(sz, 2, CV_64F);
	for (int i = 0; i < data_pts.rows; i++)
	{
		data_pts.at<double>(i, 0) = pts[i].x;
		data_pts.at<double>(i, 1) = pts[i].y;
	}

	//Perform PCA analysis
	PCA pca_analysis(data_pts, Mat(), PCA::DATA_AS_ROW);

	//Store the center of the object
	Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
		static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

	//Store the eigenvalues and eigenvectors
	vector<Point2d> eigen_vecs(2);
	vector<double> eigen_val(2);
	for (int i = 0; i < 2; i++)
	{
		eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
			pca_analysis.eigenvectors.at<double>(i, 1));

		eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
	}
	//! [pca]

	//! [visualization]
	// Draw the principal components
	circle(img, cntr, 3, Scalar(255, 0, 255), 2);
	Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
	Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
	drawAxis(img, cntr, p1, Scalar(0, 255, 0), 2);
	drawAxis(img, cntr, p2, Scalar(255, 255, 0), 10);
	imshow("pca", img);
	imwrite("./results/pca.png", img);

	double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
	//! [visualization]


	//--------------------------
	// rotate point around center
	//--------------------------
	vector<Point> rotatedPoints;
	Point tmpPoint;
	for (int i = 0; i < pts.size(); i++)
	{
		tmpPoint = rotatePoint(pts[i], cntr, -angle);
		rotatedPoints.push_back(tmpPoint);
	}

	//-----------------------

	Mat rotated_curve_img;
	img.copyTo(rotated_curve_img);
	vector<Point> final_curve;
	vector<double> curvex, curvey;
	PolyLineSplit(rotatedPoints, curvex, curvey);
	polyfit(img, curvex, curvey, final_curve);

	//vector<Point> rotatedPoints1;
	Point tmpPoint1;
	for (int i = 0; i < final_curve.size(); i++)
	{
		tmpPoint1 = rotatePoint(final_curve[i], cntr, angle);
		output_curve.push_back(tmpPoint1);
	}

	//polylines(rotated_curve_img, output_curve, false, Scalar(255,255,255), 3);
	//imshow("rotated_curve_img", rotated_curve_img);
}


//--------------------------------------------------------------------------------

cv::Point2f road::rotate2d(const cv::Point2f& inPoint, const double& angRad)
{
	cv::Point2f outPoint;
	//CW rotation
	outPoint.x = std::cos(angRad) * inPoint.x - std::sin(angRad) * inPoint.y;
	outPoint.y = std::sin(angRad) * inPoint.x + std::cos(angRad) * inPoint.y;
	return outPoint;
}

cv::Point2f road::rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad)
{
	return rotate2d(inPoint - center, angRad) + center;
}




//==============================================================================================
//                                    load config
// =============================================================================================

void road::loadConfig(string filename){
	CvFileStorage* fs = cvOpenFileStorage(filename.c_str(), 0, CV_STORAGE_READ);

	number_of_color = cvReadIntByName(fs, 0, "NumOfColor", 1);
	road_dilate_size = cvReadIntByName(fs, 0, "roadDilate", 2);
	road_blur = cvReadIntByName(fs, 0, "roadBlur", 5);
	rd_block_size = cvReadIntByName(fs, 0, "rdBlockSize", 1);
	block_size = cvReadIntByName(fs, 0, "blockSize", 1);

	debug = cvReadIntByName(fs, 0, "debug", 0);
	cvReleaseFileStorage(&fs);
}

