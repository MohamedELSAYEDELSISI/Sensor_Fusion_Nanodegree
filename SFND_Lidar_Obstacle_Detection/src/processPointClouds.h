// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>
#include "tree.h"

template<typename PointT>
class ProcessPointClouds {
public:

      //constructor
      ProcessPointClouds();
      //deconstructor
      ~ProcessPointClouds();
      
      
      
      
      
      std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
      
      void cluster_helper(int indices, std::vector<std::vector<float>>& points,std::vector<int>& cluster,std::vector<bool>& Processed,KdTree* tree, float distanceTol);
      
      
     
      void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

      std::unordered_set<int> ransac_plane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

      typename pcl::PointCloud<PointT>::Ptr  FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

      std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

      std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

     
      Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

      void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

      typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

      std::vector<boost::filesystem::path> streamPcd(std::string dataPath);


std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr>Ransac_Segment_Plane( typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);


  };
  #endif /* PROCESSPOINTCLOUDS_H_ */

