#include <iostream>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include "road.h"

using namespace cv;
using namespace std;

//----------------------------------------------------------
//-- parameters
//----------------------------------------------------------
#define SCALE false
#define SCALE_RATE .5


int main(int argc, char* argv[])
{
	//----------------------------------------------------------
	//-- paths
	//----------------------------------------------------------
	string videoPath; //-- path to video

	if (argc == 1)
		videoPath = "";
	else if (argc == 2)
		videoPath = argv[1];
	else {
		cerr << "Usage: roadSurveillance.exe videoPath";
		return 0;
	}

	cout << "video path: " << videoPath << endl;
	cout << "-----------------------\n";

	//----------------------------------------------------------
	//-- variables
	//----------------------------------------------------------
	Mat frame, fgMask, bg, img_blob;
	int numFrame = 0;
	deque<Mat> frameSeq;

	BlobTracking* blobtracking = new BlobTracking;
	blobtracking->loadConfig("./config/BlobTracking.xml");

	//-- create Background Subtractor objects
	Ptr<BackgroundSubtractor> pBackSub;
	pBackSub = createBackgroundSubtractorMOG2(500, 400, false);

	VideoCapture capture(videoPath);
	if (!capture.isOpened()) {
		cerr << "Unable to open: " << videoPath << endl;
		return 0;
	}

	int frameRate = capture.get(CAP_PROP_FPS);
	
	//----------------------------------------------------------
	//-- read first frame
	//----------------------------------------------------------
	capture >> frame;
#if SCALE
	resize(frame, frame, Size(), SCALE_RATE, SCALE_RATE);
#endif

	Mat final_road = Mat(frame.rows, frame.cols, CV_32FC1, cv::Scalar(0));
	road* roadDetect = new road(frame);

	//----------------------------------------------------------
	//-- read video
	//----------------------------------------------------------

	while (true) {
		capture >> frame;
		if (frame.empty())
			break;
		numFrame++;

#if SCALE
		resize(frame, frame, Size(), SCALE_RATE, SCALE_RATE);
#endif

		frameSeq.push_back(frame.clone());
		if (frameSeq.size() > 10)
			frameSeq.pop_front();

		//-- update the background model
		pBackSub->apply(frame, fgMask);
		pBackSub->getBackgroundImage(bg);

		//-- show the current frame and the fg masks
		imshow("Frame", frame);
		imshow("FG Mask", fgMask);
		imshow("Background", bg);

		//-- blob tracking
		blobtracking->process(frame, fgMask, img_blob);
		cvb::CvTracks tracks = blobtracking->getTracks();

		roadDetect->roadProcess(frame, fgMask, bg, final_road, tracks, numFrame, frameRate, frameSeq, 14, 20);

		//-- get the input from the keyboard
		int keyboard = waitKey(30);
		if (keyboard == 'q' || keyboard == 27)
			break;
	}

	system("Pause");
	return 0;
}

