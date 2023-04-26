
#include "processPointClouds.h"
#include <unordered_set>

#define Min_Area_Remove_D       Eigen::Vector4f(-1.8, -1.8, -2, 1)
#define Max_Area_Remove_D       Eigen::Vector4f( 1.8, 1.8, 2, 1)

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
  
        // Time segmentation process   
        auto startTime = std::chrono::steady_clock::now();
  
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
        
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered_2 (new pcl::PointCloud<PointT>());

  
		 pcl::VoxelGrid<PointT> sor;
		 sor.setInputCloud (cloud);
		 sor.setLeafSize (filterRes, filterRes, filterRes);
		 sor.filter (*cloud_filtered);
  
		 
		typename pcl::CropBox<PointT> ROI(true);
		ROI.setMin(minPoint);
		ROI.setMax(maxPoint );
		ROI.setInputCloud(cloud_filtered);
		ROI.filter(*cloud_filtered_2);
  
		
		typename pcl::CropBox<PointT> Lidar_Roof(true);
		Lidar_Roof.setMin(Min_Area_Remove_D);
		Lidar_Roof.setMax(Max_Area_Remove_D );
		Lidar_Roof.setInputCloud(cloud_filtered_2);
		Lidar_Roof.setNegative(true);
		Lidar_Roof.filter(*cloud_filtered_2);
  
  
    

        // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
       // Time of the whole 
        return cloud_filtered_2;
}

/*
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  
      // Time segmentation process   
      auto startTime = std::chrono::steady_clock::now();
      
      typename pcl::PointCloud<PointT>::Ptr obstacleCloud{new pcl::PointCloud<PointT>()};
      
      typename pcl::PointCloud<PointT>::Ptr planeCloud{new pcl::PointCloud<PointT>()};

    
      for (int index : inliers->indices)
          planeCloud->points.push_back(cloud->points[index]);


      pcl::ExtractIndices<PoinT> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(inliers);
      extract.filter(*obstacleCloud);

     // Just for Pointcloud data type of XYZI just for real data. For more general data type replace pcl::PointXYZI to PointT and add typename before the two arguments pcl::PointCloud
      std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(obstacleCloud, planeCloud);
  
     auto endTime = std::chrono::steady_clock::now();
     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
     std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
     
      // Time of the whole 
      return segResult;
}

*/
// implementation RANSAC for linear plane detection  
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::ransac_plane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol){

	  // Time segmentation process   
	  auto startTime = std::chrono::steady_clock::now();
	  srand(time(NULL));
      
      std::unordered_set<int> inliersResult;
      
      float x_1,y_1,z_1,x_2,y_2,z_2,x_3,y_3,z_3,A,B,C,x_4,y_4,z_4,d,D;
      
      while(maxIterations--){
      
           std::unordered_set<int> plane_data;
           
            for(int i= 0; plane_data.size()<3; i++){
            	
              plane_data.insert((rand())%(cloud->points.size()));
              
            }
              auto it= plane_data.begin();

              (y_1)=cloud->points[*it].y;
              (x_1)=cloud->points[*it].x;
              (z_1)=cloud->points[*it].z;
              it++;	  

              (y_2)=cloud->points[*it].y;
              (x_2)=cloud->points[*it].x;
              (z_2)=cloud->points[*it].z;
              it++;
              

              (y_3)=cloud->points[*it].y;
              (x_3)=cloud->points[*it].x;
              (z_3)=cloud->points[*it].z;

              A=(((y_2-y_1)*(z_3-z_1))-((z_2-z_1)*(y_3-y_1)));
              B=(((z_2-z_1)*(x_3-x_1))-((x_2-x_1)*(z_3-z_1)));
              C=(((x_2-x_1)*(y_3-y_1))-((y_2-y_1)*(x_3-x_1)));
              D=-((A*x_1)+(B*y_1)+(C*z_1));

              for(int index =0; index < cloud->points.size();index++){

                     if(plane_data.count(index)>0)continue;
                 
                     x_4=cloud->points[index].x;
                     y_4=cloud->points[index].y;
                     z_4=cloud->points[index].z;
                     
                     d = fabs(((A*x_4)+(B*y_4)+(C*z_4)+D)/sqrt((A*A)+(B*B)+(C*C)));

                     if(d<=distanceTol)
                       plane_data.insert(index); 
               }

             if(plane_data.size()>inliersResult.size())             
              inliersResult =  plane_data;
            
            
                
        }
      
      auto endTime = std::chrono::steady_clock::now();
      auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
      std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
      
      return inliersResult;
      

  }	



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac_Segment_Plane( typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold){
       
        auto startTime = std::chrono::steady_clock::now();

	    std::unordered_set<int> inliers =  ransac_plane(cloud, maxIterations, distanceThreshold );
	    
	    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	    
	    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

		for(int index = 0; index < cloud->points.size(); index++)
		{
			pcl::PointXYZI point = cloud->points[index];
			
			if(inliers.count(index))
				
				 cloudInliers->points.push_back(point);
			
			else
				
				 cloudOutliers->points.push_back(point);
		}
	
   
         
      auto endTime = std::chrono::steady_clock::now();
      auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
      std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
        
      std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers) ; 		
    
      return segResult;
         	
}



/*template<typename PointT>
std::pair< pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PoinT>::SegmentPlane( pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();    

        // TODO:: Fill in this function to find inliers for the cloud.
        //intilazing class pointer from type ModelCoeficients and then save the memory storage at the heap from type pcl: ModelCoefficients
         
        pcl::ModelCoefficients::Ptr Coe{new pcl::ModelCoefficients()};
        
        // intilazinf classs pointer from type pcl::PointIndcie and save the memory storage athe the heap from type of pcl::PointIndcies
         
        pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
        
        // create an object pointer from data type of pcl::SACSegmentation<pcl::PointXYZ>
         
        pcl::SACSegmentation<PoinT> seg;
        // this object pointer will be used for saving the segmentation data and access to all methods that are included in SACSegmenation class
        // using pointer we get to the first method that through, we used the segmentation model type in this case the planar
         
        seg.setOptimizeCoefficients(true); 

        seg.setModelType(pcl::SACMODEL_PLANE);
        
        // now which algthiom of plananr segmenation well be used ? at this time we are using SAC_RANSAC)
         
        seg.setMethodType(pcl::SAC_RANSAC);
        
        // we selected at this time itreation 1000 times
         
        seg.setMaxIterations(maxIterations);
        
        // selecting threshold 0.1
         
        seg.setDistanceThreshold(distanceThreshold);
        
        // then start the segmemaxIterationsntation process through setInputCloud which it takes the arg of datatype pcl::PointCloud<PoinT>, you have to pass the scanned laser data that is generated from scan();
        
        seg.setInputCloud(cloud);
        // start the segmentation process
         
        seg.segment(*inliers,*Coe);
        
        if(inliers->indices.size()==0) std::cout <<"couldent estimate";


        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        std::pair< pcl::PointCloud<PoinT>::Ptr,  pcl::PointCloud<PoinT> segResult(,);

         return segResult;
}*/




template<typename PointT>
void ProcessPointClouds<PointT>::cluster_helper(int indices, std::vector<std::vector<float>>& points,std::vector<int>& cluster,std::vector<bool>& Processed,KdTree* tree, float distanceTol){
   
	
      if(Processed[indices]!=true)
      {
      
    	  Processed[indices]=true;
    	  
          cluster.push_back(indices);
          
          std::vector<int> nearest =tree->search(points[indices],distanceTol);
     
          for(int id: nearest){
        	 
              if(!Processed[id]) cluster_helper(id, points, cluster, Processed, tree,  distanceTol);
              
        }
      
     }

}

 
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){

    auto startTime = std::chrono::steady_clock::now();
    
    KdTree* tree = new KdTree;
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clust ;
    
    std::vector<std::vector<float>> Points(cloud->points.size());
     
    int i = 0;
    
    while (i < cloud->points.size() )
    {
        Points[i] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        
        tree->insert(Points[i], i);
       
        i++;
    }
            
    std::vector<std::vector<int>> clusters;
        
    std::vector<bool> Processed(Points.size(),false);
        
    int x = 0;
        
    while(x < Points.size())
          {
            if(Processed[x])
              {
            	
                x++;
                continue;
                
               }
            
    std::vector<int> cluster; 
        
    cluster_helper(x,Points,cluster,Processed,tree,clusterTolerance);
        
    clusters.push_back(cluster);
        
    x++;     
    
    }    
    
   	for(std::vector<int> cluster : clusters)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
        
  		for(int indice: cluster) clusterCloud->points.push_back(cloud->points[indice]);
        
  		clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        
        clust.push_back(clusterCloud);
        
  		
  	}       
        
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clust;
}






template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
