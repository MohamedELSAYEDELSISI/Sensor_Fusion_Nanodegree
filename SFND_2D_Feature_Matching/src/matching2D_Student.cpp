#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
  
    double t = 0;
  
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;

        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        {
            descSource.convertTo(descSource, CV_32F);

            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

        cout << "FLANN matching";
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        t = ((double)cv::getTickCount());

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
       
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    }
    else if (selectorType.compare("SEL_KNN") == 0)
    {

      vector<vector<cv::DMatch>> knn_matches;
      
       t = ((double)cv::getTickCount());

        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches
       t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

        double minDescDistRatio = 0.8;

        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {

            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }

    }
    
    cout << "Matching Type :" << matcherType <<" Selector type :" << selectorType <<" Detector with n=" <<matches.size() << " Match_Keypoints in " << 1000 * t / 1.0 << " ms" << endl;
   
  return t;

}





// Use one of several types of state-of-art descriptors to uniquely identify keypoints
double descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;

    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    if (descriptorType.compare("BRIEF") == 0)
    {

       extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }


     if (descriptorType.compare("ORB") == 0)
    {

       extractor = cv::ORB::create();
    }


      if (descriptorType.compare("FREAK") == 0)
    {

        extractor = cv::xfeatures2d::FREAK::create();
    }


    if (descriptorType.compare("AKAZE") == 0)
    {

        extractor = cv::AKAZE::create();
    }


    if (descriptorType.compare("SIFT") == 0)
    {

        extractor = cv::xfeatures2d::SIFT::create();
    }


    // perform feature description
    double t = (double)cv::getTickCount();

    extractor->compute(img, keypoints, descriptors);

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    cout << descriptorType <<  1000 * t / 1.0 << " ms " << endl;
  
    return t ; 
}




// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypoints_SHITOMASI(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{


    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood

    double maxOverlap = 0.0; // max. permissible overlap between two features in %

    double minDistance = (1.0 - maxOverlap) * blockSize;

    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    //double t = (double)cv::getTickCount();

    vector<cv::Point2f> corners;

    double t = (double)cv::getTickCount();

    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;

        newKeyPoint.pt = *(it);//cv::Point2f((*it).x, (*it).y);

        newKeyPoint.size = blockSize;

        keypoints.push_back(newKeyPoint);

    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;




if(bVis)
{
    cv::Mat visImage = img.clone();

    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    string windowName = "Shi-Tomasi Corner Detector Results";

    cv::namedWindow(windowName);

    imshow(windowName, visImage);

    cout << "Press any key to continue\n";

    cv::waitKey(0);
  }
  
  return t;

}




double Harris_keypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    int blockSize = 2;

    int apertureSize = 3;

    int minResponse = 100;

    double k = 0.04;


    cv::Mat dst, dst_norm, dst_norm_scaled;

    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    double t = (double)cv::getTickCount();

    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    if(bVis)
    {

    string windowName = "Harris Corner Detector Response Matrix";

    cv::namedWindow(windowName);

    cv::imshow(windowName, dst_norm_scaled);

    cv::waitKey(0);

   }


    double maxOverlap = 0.0;

    for (size_t j = 0; j < dst_norm.rows; j++)
    {

        for (size_t i = 0; i < dst_norm.cols; i++)
        {

            int response = (int)dst_norm.at<float>(j, i);

            if (response > minResponse)
            {

                cv::KeyPoint newKeyPoint;

                newKeyPoint.pt = cv::Point2f(i, j);

                newKeyPoint.size = 2 * apertureSize;

                newKeyPoint.response = response;

                bool bOverlap = false;

                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {

                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);

                    if (kptOverlap > maxOverlap)
                    {

                        bOverlap = true;

                        if (newKeyPoint.response > (*it).response)
                        {
                            *it = newKeyPoint;
                            break;
                        }

                    }

                }

                if (!bOverlap)
                {

                    keypoints.push_back(newKeyPoint);

                }

            }
        }
    }

        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

        cout << "Haaris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;



    if(bVis)
    {

    string windowName = "Harris Corner Detection Results";

    cv::namedWindow(windowName);

    cv::Mat visImage = dst_norm_scaled.clone();

    cv::drawKeypoints(dst_norm_scaled, keypoints, visImage,  cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv::imshow(windowName, visImage);

    cv::waitKey(0);

    }

   return t;

}



double detKeypoints_Method(vector<cv::KeyPoint> &keypoints, cv::Mat &img, string detectorType, bool bVis)
{
     //I let the default parameters for every method

     cv::Ptr<cv::FeatureDetector> detector;

     if(detectorType.compare("SIFT") == 0) detector = cv::xfeatures2d::SIFT::create();

     if(detectorType.compare("BRISK") == 0) detector = cv::BRISK::create();

     if(detectorType.compare("FAST") == 0)
     {

     int threshold=30;

     detector = cv::FastFeatureDetector::create(threshold);

     }

    if(detectorType.compare("ORB") == 0) detector = cv::ORB::create();


    if(detectorType.compare("AKAZE") == 0) detector = cv::AKAZE::create();

    double t = (double)cv::getTickCount();

    detector->detect( img, keypoints);

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    cout << detectorType << " Detector with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;


    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();

        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        string windowName = " Corner Detector Results of" +    detectorType;

        cv::namedWindow(windowName );

        imshow(windowName, visImage);

        cv::waitKey(0);
    }

 return t;

}
void Performance_Evaluation(vector<int>& Container_Images_in_RectKeys, vector<int>& Container_Images_All_Keys, vector<double>& Time_Det_All, vector<double>& Time_Ext_ROI_K, vector<double>& Time_Ext_Desc, string detectorType, string descriptorType, int& N_Imgs)
{
  
  
   int Average_matched_k=0;
   double T_1 = 0,T_2 = 0,T_3 = 0;
   int i = 0;
   
   vector<int>::iterator it, it_s;
   vector<double>::iterator it_f, it_S, it_o;
  
   cout<<endl<<"Performance_Evaluation"<<endl;
   for( it = Container_Images_All_Keys.begin(),  it_s = Container_Images_in_RectKeys.begin(), it_f = Time_Det_All.begin(),  it_S = Time_Ext_ROI_K.begin(),  it_o =Time_Ext_Desc.begin(), i; it !=  Container_Images_All_Keys.end(), it_S !=  Time_Ext_ROI_K.end(); it++, it_s++, i++ ,it_f++, (it_S)++, (it_o)++)
   {
     Average_matched_k+= *(it_s);
     cout<< "#------------------------#"<<endl;
     cout<< "detectorType:" << detectorType<< "\n" <<"descriptorType:" << descriptorType<<"\t"<< "All keypoints in image0"<<i<<"\t"<<*(it) << "\t" << "Keys_in_ROI:"<< *(it_s)<<"\t"<< "Time_Det:"<<1000*(*(it_f)) <<"\t"<< "Time_Ext_ROI_K:"<<1000*(*(it_S))<<"\t"<<"Time_Ext_Desc:"<<1000*(*(it_o))<<endl;
     
        T_1 += (1000*(*(it_f)));
          
        T_2 += (1000* (*(it_S)));
          
        T_3 +=  (1000*(*(it_o)));

   }
  
    Average_matched_k/=(N_Imgs+1);
    
       cout<< "#------------------------#"<<endl;
    cout<< "detectorType:" << detectorType<<"\n" <<"descriptorType:" << descriptorType <<"\t" << "descriptorType:" << descriptorType<< "\t"<< " Average_ROI_K:"<< Average_matched_k <<endl;
  
   cout<<endl<<"Performance_Evaluation_time"<<"\n"<<"#------------------------#"<<endl;
  
  
  
          cout<< "detectorType:" << detectorType<< "\t" << "descriptorType:" << descriptorType<< "\t"<< "Time Excuation All key:"<< T_1 <<endl<<"#------------------------#"<<endl;
       
        cout<< "detectorType:" << detectorType<< "\t" << "descriptorType:" << descriptorType<< "\t"<< "Time Excuation ROI key:"<< T_2 <<endl<< "#------------------------#"<<endl;
  
        cout<< "detectorType:" << detectorType<< "\t" << "descriptorType:" << descriptorType<< "\t"<< "Time Excuation Desc:"<<T_3 <<endl<< "#------------------------#"<<endl;
  
  
}