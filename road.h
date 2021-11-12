#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <string>
#include <queue>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "./package_tracking/cvblob/cvblob.h"
#include "package_tracking/BlobTracking.h"

using namespace cv;
using namespace std;


class road
{

public:

	road(const Mat& input);
	~road();

	struct Gaussian{
		float sigma;
		float mu;
		float weight;
		int n;
	};

	struct Cell {
		int ymin{ 0 }, ymax{ 0 }, xmin{ 0 }, xmax{ 0 };
		int number{ 0 };
		int height{ 0 }, width{ 0 };
		float avgSpeed{ 0.0 };
		float avgDir{ 0.0 };
		int numOfCars{ 0 };
		int xcenter{ 0 }, ycenter{ 0 };
	};

	Gaussian* mode_angle;
	Gaussian* mode_flow;
	int* mode_a_num;
	int* mode_f_num;
	int Ka, Kf, maxn, bias, colorSpace;
	int rd_block_size = 1;
	Mat road_map, stop_ROI, bg_sdv, bg_sdv_gray, bg_sdv_gray_roi, green, non_green, initial_fg, nr_in_ROI, no_road_region, gfm_road;

	Mat downDir, upDir, leftDir, rightDir, wholeRoad, wholeRoad2, wholeRoad3, whole_convex;

	Mat acc_traj;
	Mat degrees, numbers, angles, flows, flow_angles, flow_angles_median, flow_mags;

	float ave_size = 0.0;
	float ave_y = 0.0;
	int size_count = 0;
	int horizonLine = 0;
	Mat flood_seeds;
	Mat flood_mask;

	Mat ginv, cinv, hinv, allinv, pg, pc, ph, ps, pf, pall, pgbhist, pgrayhist, merged, cropped;
	Mat merged_binary, roadProbEst, accForegroundMask, accFloodMat, P_flood, totalMotionMask;


	//------------------------
	// just representation
	//------------------------
	Mat arrows2, test, track_orthog, flood_fill, angle_arrows, arrows, flow_arrows, flow_angle_arrows, flow_angle_m_arrows, flow_mag_map, show_flow, arrows_drawn;
	Mat output_roi;
	Mat road_represent;
	int rep_counter = 0;

	ofstream performance_output;

	void road::performance(Mat& result, Mat& ground_truth, int& frame_num);
	//------------------------


	vector<cv::Point> down_final_counter, up_final_counter;
	vector<vector<Point> > down_contour, up_contour, whole_contours;


	void initialization(Mat& input);
	void finalRoad(const Mat& frame, Mat& mask, Mat& background, cvb::CvTracks& tracks, Mat final_road, Mat acc_mask);
	void loadConfig(string filename);

	void generateFloodSeeds(const cv::Mat& frame, cvb::CvTrack* track, const int& move_length);
	void road::cropMask(Mat& frame, Mat& mask, cvb::CvTracks& tracks, const int& move_length, Mat* croppedMask);
	void floodFillFromSeedMask(const Mat& image, const Mat& seeds, Mat* outputMask);
	void density(Mat& inputMat, Mat* edge_density);
	void road::intrinsicImage(const Mat& input, Mat* intrinsic);
	void probabiltyMapGenerator(Mat& input, Mat& mask, Mat* road_prob);
	double road::pca_angle(const vector<Point>& pts, Mat& img);
	double road::imgMedian(const cv::Mat& img, const Mat& mask);
	void road::roadEstimate(Mat& frame, Mat& background, Mat& mask, int& frame_num, cvb::CvTracks& tracks);

	
	void road::acc_mask(Mat& mask, cvb::CvTracks& tracks, const short& move_length);
	void convex(Mat& frame, std::vector<std::vector<cv::Point>> contours, cv::Mat* output_convex, vector<cv::Point>& output_convex_points);
	void twoSideRoadDetection(Mat& frame, Mat& background, cv::Mat& mask, cvb::CvTracks& tracks, const short& move_length, int frame_num, deque<Mat> frameSeq);
	void fourSideRoadDetection(cv::Mat& mask, cvb::CvTracks& tracks, const short& move_length);
	void roadProcess(Mat& frame, Mat& mask, Mat& background, Mat& initial_map, cvb::CvTracks tracks, int frame_num, int frame_rate, deque<Mat> frameSeq, int move_length, int update_period);
	std::vector<std::vector<cv::Point>> findBiggestContour(cv::Mat& img, int numOfContours);
	void removeIrrelevantContours(vector<vector<Point>>& contour, float thresh);
	void smoothen(Mat& frame, Mat& contourImg, std::vector<std::vector<cv::Point>> contours, cv::Mat* smoothed_curve);
	void removeContourIntersection(cv::Mat& frame, std::vector<cv::Point>& down_convex, std::vector<cv::Point>& up_convex, cv::Mat& down_region, cv::Mat& up_region);
	void curve_fit(const Mat& frame, const Mat& background, Mat& contourImg, std::vector<std::vector<cv::Point>> contours, cv::Mat* output_boundaries);
	void laneDetection(cv::Mat& frame, cvb::CvTracks& tracks);
	void boundaryDetection(cv::Mat& frame, cv::Mat& background, cv::Mat* edge_output);

	void avgDegree(Mat& frame, Mat& track_mask, float& deg);
	void degreeModel(Mat& frame, Mat& frame_angles);
	void flowModel(Mat& frame, Mat& frame_flows, float track_movement);

	void updateCell(Mat frame, Mat roadMask, Cell& c, cvb::CvTrack* t);
	void drawRoiGrid(Mat frame, Mat roiFill, vector<Cell>& cells);
	double findMajority(vector<double> a, float thresh);


	void motionSegment(const cv::Mat& frame, const cv::Mat& mask, cvb::CvTracks tracks, int move_length);
	void motionSegment2(const cv::Mat& frame, const cv::Mat& mask, cvb::CvTracks tracks, int move_length);

	const int channel = 2;

private:
	bool debug = true;

	int number_of_color = 1;

	int block_size = 1;
	int road_dilate_size = 1;
	int road_blur = 5;

	int w_flood = 1;

	int kernel_size = 3; // odd number
	//vector<float> median_vec;

	float cov0_a, alpha, p, cov_low_a, cov_hi_a;
	float cov0_f, alpha_f, p_f, cov_low_f, cov_hi_f;

	bool smooth = true;
	bool isOpen = false;
	bool first_track;

	bool gridDraw = false;
	int a = 1;
	int gridRows = 15;
	int move_length = 15;

	void getGaussianDerivs(double sigma, int M, vector<double>& gaussian, vector<double>& dg, vector<double>& d2g);
	void getdX(vector<double> x, int n, double sigma, double& gx, double& dgx, double& d2gx, vector<double> g, vector<double> dg, vector<double> d2g, bool isOpen);
	void getdXcurve(vector<double> x, double sigma, vector<double>& gx, vector<double>& dx, vector<double>& d2x, vector<double> g, vector<double> dg, vector<double> d2g, bool isOpen);

	template<typename T, typename V>
	void PolyLineSplit(const vector<Point_<T> >& pl, vector<V>& contourx, vector<V>& contoury);

	template<typename T, typename V>
	void PolyLineMerge(vector<Point_<T> >& pl, const vector<V>& contourx, const vector<V>& contoury);

	void ResampleCurve(const vector<double>& curvex, const vector<double>& curvey, vector<double>& resampleX, vector<double>& resampleY, int N, bool isOpen);
	void polyfit(const Mat& src, const vector<double>& x, const vector<double>& y, vector<Point>& final_curve);
	void drawAxis(Mat&, Point, Point, Scalar, const float);
	void pca_curve(const vector<Point>&, Mat&, vector<Point>&);
	cv::Point2f rotate2d(const cv::Point2f& inPoint, const double& angRad);
	cv::Point2f rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad);

	void canny_automatic_thresh(const Mat& input, double* lower_thresh_val, double* high_thresh_val);
};

#endif // !ROAD_H
