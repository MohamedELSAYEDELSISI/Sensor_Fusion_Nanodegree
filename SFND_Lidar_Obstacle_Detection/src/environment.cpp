
/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
// Parameters cityblock
#define Maxiterations     100
#define Min_Clust_Points  42
#define Max_Clust_Points  1000
#define Planar_Threshold   0.2
#define Clustred_Objects_Threshold  0.4
#define Red   1,0,0                 
#define Green 0,1,0
#define Blue  0,0,1
#define Yellow 1,1,0
#define White  1,1,1


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(1,0,0), "car3");
    
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


/*void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
		bool renderScene = false;
		std::vector<Car> cars = initHighway(renderScene, viewer);
		// const  Vect3 *vet = new Vect3(0,0,2);
		// TODO:: Create lidar sensor
		Lidar *lidar = new Lidar(cars,0);
		
		lidar->minDistance =0;
		
		lidar->sderr =0.2;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_data =lidar->scan();
		
		ProcessPointClouds<pcl::PointXYZ> Point_Cloud;
		
		renderRays(viewer,Vect3(0,0,2),lidar_data);
		
		renderPointCloud(viewer,lidar_data,"PCD");
		
		std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = Point_Cloud.SegmentPlane(lidar_data, 80, 0.4);
	  
		std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> Clustred_data_Object = Point_Cloud.Clustering(segmentCloud.first, 1.0, 3,80);
	  
		renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(Red));
		renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(Green));
		
		int clusterId = 0;
		
		std::vector<Color> colors = {Color(White), Color(Yellow), Color(Blue)};
	
		for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : Clustred_data_Object)
		{
				  std::cout << "cluster size ";
				  
				  Point_Cloud.Clustering.numPoints(cluster);
				  
			      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
			      
			      Box box =oint.BoundingBox(cluster);
			      
			      renderBox(viewer,box,clusterId);
			      
				  ++clusterId;
		
		
		}
}*/

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){

  
  
	 pcl::PointCloud<pcl::PointXYZI>::Ptr Stored_Cloud_Data = inputCloud;
		
	 //renderRays(viewer,Vect3(0,0,2),object);
	 pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(Stored_Cloud_Data, 0.1f , Eigen::Vector4f(-15, -6.0, -3, 1), Eigen::Vector4f (30 ,7 ,1 , 1));
	  
	 std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =pointProcessorI->Ransac_Segment_Plane(filteredCloud, Maxiterations, Planar_Threshold);
  
	 //renderPointCloud(viewer,Stored_Cloud_Data,"Cloud_Every_Thing");
     //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(Red));
   
	 renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(Green));
  
     std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustred_data_Object = pointProcessorI->Clustering(segmentCloud.first,Clustred_Objects_Threshold,Min_Clust_Points,Max_Clust_Points);
  
     int clusterId = 0;
     
     std::vector<Color> colors = {Color(Red), Color(Green), Color(Blue)};

	for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : Clustred_data_Object)
	{
		 std::cout << "cluster size ";
		 
		 pointProcessorI->numPoints(cluster);
		 
	     renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
	     
	     Box box = pointProcessorI->BoundingBox(cluster);
	     
	     renderBox(viewer,box,clusterId);
	     
		 ++clusterId;
	
	
	}
 
	
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-15, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
      std::cout << "starting enviroment" << std::endl;

      pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D- Viewer"));
  
      CameraAngle setAngle = FPS;
  
      initCamera(setAngle, viewer);
  
      ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  
      std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  
      auto streamIterator = stream.begin();
      
      pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
      
       //simpleHighway(viewer);
       //cityBlock(viewer);

		while (!viewer->wasStopped ())
	      {
	
	  // Clear viewer
	  viewer->removeAllPointClouds();
	  
	  viewer->removeAllShapes();
	
	  // Load pcd and run obstacle detection process
	  inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
	  
	  cityBlock(viewer, pointProcessorI, inputCloudI);
	
	  streamIterator++;
	  
	  if(streamIterator == stream.end()) streamIterator = stream.begin();
	
	  viewer->spinOnce ();
	 }
  
}
