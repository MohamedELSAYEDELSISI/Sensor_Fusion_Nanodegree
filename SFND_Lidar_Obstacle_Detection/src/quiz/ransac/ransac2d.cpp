/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> 
 ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	float x_1,y_1,z_1,x_2,y_2,z_2,x_3,y_3,z_3,A,B,C,x_4,y_4,z_4,d,D;
	std::unordered_set<int> x;
	// TODO: Fill in this function
    
	while(maxIterations--){
      
      for(int i= 0; x.size()<3; i++){
        x.insert((rand())%(cloud->points.size()));
      }
        auto it= x.begin();
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
	    for(int index =0; index<cloud->points.size();index++){
          
           if(x.count(index)>0)continue;
           x_4=cloud->points[index].x;
           y_4=cloud->points[index].y;
           z_4=cloud->points[index].z;
           d = ((A*x_4)+(B*y_4)+(C*z_4)+D)/sqrt((A*A)+(B*B)+(C*C));
           if(d<=distanceTol)
           x.insert(index); 
        }
        if(x.size()>inliersResult.size()) return x;
        else return inliersResult;
	}
	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	//return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	 //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = ransac(cloud, 10000, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
