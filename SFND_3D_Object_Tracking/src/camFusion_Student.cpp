
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
     
	vector<cv::DMatch>Matches{0};
	float Sum{0}, Thres{0}, Mean{0};
	
	for(auto it = kptMatches.begin(); it != kptMatches.end(); it++)
	{	
		if( boundingBox.roi.contains(kptsCurr[it->trainIdx].pt) == true ) Matches.push_back(*it);	
	}
	
	if(Matches.size()!= 0)
    {
      for(auto Match : Matches) 
      {
        Sum += norm(kptsPrev[Match.queryIdx].pt-kptsCurr[Match.trainIdx].pt);	
        
      }
      Mean = (Sum)/(Matches.size());
      for(auto Match : Matches)
      {
        if(norm(kptsPrev[Match.queryIdx].pt-kptsCurr[Match.trainIdx].pt) < Mean ) boundingBox.kptMatches.push_back(Match);        
      }
      
  }		
	
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    double Curr_Dist, Prev_Dist;
    vector<double> R_DistRatio;
    double dt = 1/frameRate;
    
	for(auto it_1 = kptMatches.begin(); it_1 != kptMatches.end()-1; it_1++)
	{
		for(auto it_2 = kptMatches.begin()+1; it_2 != kptMatches.end(); it_2++)
		{
			 Curr_Dist =  abs(cv::norm((kptsCurr[it_1->trainIdx].pt)-(kptsCurr[it_2->trainIdx].pt)));
			 Prev_Dist =  abs(cv::norm((kptsPrev[it_1->queryIdx].pt)-(kptsPrev[it_2->queryIdx].pt)));
			 if(Prev_Dist > 0 && Curr_Dist > 100) R_DistRatio.push_back(Curr_Dist/Prev_Dist);
				 	
		}
		
	}
	
	if(R_DistRatio.size() == 0) TTC = NAN;
	else
    {
	sort(R_DistRatio.begin(),R_DistRatio.end());
	long medIndex = floor(R_DistRatio.size() / 2.0);
	double medDistRatio = R_DistRatio.size() % 2 == 0 ? (R_DistRatio[medIndex - 1] + R_DistRatio[medIndex]) / 2.0 : R_DistRatio[medIndex];
	TTC = -dt/(1-medDistRatio);
    cout<<"Camera time Col"<<" "<<TTC<<endl;
}
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dt = 1/frameRate, Sum{0};
    
    int Min_Y{2};
    
    vector<double>Prev_Points, Curr_Points;
    
    double Med_Curr_X, Med_Prev_X;
	
    for(auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); it++)
    {
      if(abs(it->y) <= Min_Y)
      {
      Sum+=it->x;
      Prev_Points.push_back(it->x);
      }
    }
  
    Med_Prev_X = Sum/Prev_Points.size() ; 
    Sum = 0;
     
    for(auto it2 = lidarPointsCurr.begin(); it2 != lidarPointsCurr.end(); it2++)
    {
      if(abs(it2->y) <= Min_Y)
      {
      Sum+=it2->x;
      Curr_Points.push_back(it2->x);
      }
    }
  
    Med_Curr_X = Sum/Curr_Points.size() ;
         
    TTC =( (Med_Curr_X * dt) / (Med_Prev_X-Med_Curr_X) );
        
    cout<<"Lidar time Col"<<" "<<TTC<<endl;
  }



void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
  
      int array[prevFrame.boundingBoxes.size()]           [currFrame.boundingBoxes.size()] = {0};

    for (auto it = matches.begin(); it != matches.end(); it++)
    {
       
        cv::KeyPoint prevKeypoint = prevFrame.keypoints[it->queryIdx];
        cv::KeyPoint currKeypoint = currFrame.keypoints[it->trainIdx];

        vector<int> prev_matchesIds, curr_matchesIds;
        for (auto it_1 = prevFrame.boundingBoxes.begin(); it_1 != prevFrame.boundingBoxes.end(); it_1++ )
        {

            if (it_1->roi.contains(prevKeypoint.pt))
            
                prev_matchesIds.push_back(it_1->boxID);
            
        }
   
        for (auto it_2= currFrame.boundingBoxes.begin(); it_2 != currFrame.boundingBoxes.end(); it_2++)
        {

            if (it_2->roi.contains(currKeypoint.pt))
            
                curr_matchesIds.push_back(it_2->boxID);
            
        }


        for (int prev_id : prev_matchesIds)
        {
            for (int curr_id : curr_matchesIds)
            
                array[prev_id][curr_id]++;
            
        }

    }

    for (size_t i = 0; i < prevFrame.boundingBoxes.size(); i++)
    {
        
        int highestMatches = 0;
        int highestMatchesIdx = 0;
        for (size_t j = 0; j < currFrame.boundingBoxes.size(); j++)
        {
            if (array[i][j] > highestMatches)
            {
                highestMatches = array[i][j];
                highestMatchesIdx = j;
            }
        
        
        bbBestMatches[i] = highestMatchesIdx;
        }
        
    }
}
  