#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "dataStructures.h"



double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef, std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType);

double descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType);

double detKeypoints_SHITOMASI(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis);

double Harris_keypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis);

double detKeypoints_Method(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis);

void Performance_Evaluation(std::vector<int>& Container_Images_in_RectKeys, std::vector<int>& Container_Images_All_Keys, std::vector<double>& Time_Det_All, std::vector<double>& Time_Ext_ROI_K, std::vector<double>& Time_Ext_Desc,  std::string detectorType,  std::string descriptorType, int& N_Imgs);


#endif /* matching2D_hpp */
