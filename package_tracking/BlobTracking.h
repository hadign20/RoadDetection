#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "cvblob/cvblob.h"

using namespace cv;
using namespace std;

class BlobTracking
{
public:
  bool firstTime;
  int minArea;
  int maxArea;
  int distance, inactiveTime;
  int erosion_size, dilation_size;
  
  bool debugTrack;
  bool debugBlob;
  bool showBlobMask;
  bool showOutput;

  cvb::CvTracks tracks;
  //cvb::CvBlobs blobs;
  void saveConfig();
  void loadConfig(string filename);


  BlobTracking();
  ~BlobTracking();

  void process(const cv::Mat &img_input, const cv::Mat img_mask, cv::Mat &img_output);
  const cvb::CvTracks getTracks();
  //const cvb::CvBlobs getBlobs();

  cv::Mat morphKernel_dilation;
  cv::Mat morphKernel_erosion;
};

