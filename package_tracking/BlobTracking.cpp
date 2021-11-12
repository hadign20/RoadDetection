#include "BlobTracking.h"

BlobTracking::BlobTracking(){
	std::cout << "BlobTracking()" << std::endl;
}

BlobTracking::~BlobTracking(){
	std::cout << "~BlobTracking()" << std::endl;
}

const cvb::CvTracks BlobTracking::getTracks(){
	return tracks;
}

//const cvb::CvBlobs BlobTracking::getBlobs() {
//	return blobs;
//}

void BlobTracking::process(const cv::Mat &img_input, const cv::Mat img_mask, cv::Mat &img_output){
	if (img_input.empty() || img_mask.empty()){
		return;
	}

	//loadConfig();

	IplImage* frame = new IplImage(img_input);
	cvConvertScale(frame, frame, 1, 0);
	Mat mask;
	img_mask.copyTo(mask);

	if (erosion_size != 0){
		morphKernel_erosion = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size));
		erode(mask, mask, morphKernel_erosion);
	}

	if (dilation_size != 0){
		morphKernel_dilation = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));
		dilate(mask, mask, morphKernel_dilation);
	}

	IplImage* segmentated = new IplImage(mask);



	
	if(showBlobMask)
		cvShowImage("Blob Mask", segmentated);

	IplImage* labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL, 1);

	cvb::CvBlobs blobs;
	unsigned int result = cvb::cvLabel(segmentated, labelImg, blobs);
	cvb::cvFilterByArea(blobs, minArea, maxArea);
  
	if(debugBlob)
		cvb::cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX|CV_BLOB_RENDER_CENTROID|CV_BLOB_RENDER_ANGLE|CV_BLOB_RENDER_TO_STD);
	//else
		//cvb::cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX);
		//cvb::cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX|CV_BLOB_RENDER_CENTROID|CV_BLOB_RENDER_ANGLE);
	
	cvb::cvUpdateTracks(img_input, blobs, tracks, distance, inactiveTime);
  
	if(debugTrack)
		cvb::cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX|CV_TRACK_RENDER_TO_STD);
	//else
		//cvb::cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_BOUNDING_BOX);
		//cvb::cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);
  

	if(showOutput)
		cvShowImage("Blob Tracking", frame);

	cv::Mat tmpimg = cv::cvarrToMat(frame);
	cv::Mat img_result(tmpimg);
	img_result.copyTo(img_output);

	//cvReleaseImage(&frame);
	//cvReleaseImage(&segmentated);
	cvReleaseImage(&labelImg);
	delete frame;
	delete segmentated;
	cvReleaseBlobs(blobs);
}




void BlobTracking::loadConfig(string filename){
	CvFileStorage* fs = cvOpenFileStorage(filename.c_str(), 0, CV_STORAGE_READ);
  
	minArea = cvReadIntByName(fs, 0, "minArea", 10);
	maxArea = cvReadIntByName(fs, 0, "maxArea", 3000);
	distance = cvReadIntByName(fs, 0, "distance", 10);
	inactiveTime = cvReadIntByName(fs, 0, "inactiveTime", 15);
	erosion_size = cvReadIntByName(fs, 0, "erosion", 1);
	dilation_size = cvReadIntByName(fs, 0, "dilation", 1);

	debugTrack = cvReadIntByName(fs, 0, "debugTrack", 1);
	debugBlob = cvReadIntByName(fs, 0, "debugBlob", 1);
	showBlobMask = cvReadIntByName(fs, 0, "showBlobMask", 1);
	showOutput = cvReadIntByName(fs, 0, "showOutput", 1);

	cvReleaseFileStorage(&fs);
}
