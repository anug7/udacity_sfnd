/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
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


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI> *ppc,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){
    bool renderScene = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = ppc->FilterCloud(cloud,
                                                          0.15, {-20.0, -5,
                                                          -2, 1}, {20, 6.5, 1, 1});
    
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
              pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = ppc->RansacPlane(cloudFiltered, 100, 0.2);

    renderPointCloud(viewer,segmentCloud.first,"obstCloud", Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud", Color(0,1,0));
		
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
            ppc->ClusterClouds(segmentCloud.first, 0.5, 30, 750);
		int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      //ppc->numPoints(cluster);
      //BoxQ box = ppc->BoundingBoxQ(cluster);
      Box box = ppc->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
      renderPointCloud(viewer, cluster, "obstCloud" +std::to_string(clusterId),colors[clusterId]);
      ++clusterId;
    }
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    Lidar *lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rays = lidar->scan();
    //renderRays(viewer, lidar->position, rays);
    //renderPointCloud(viewer, rays, "pcd");
    
    ProcessPointClouds<pcl::PointXYZ> *ppc = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ppc->SegmentPlane(rays, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud", Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud", Color(0,1,0));

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = ppc->Clustering(segmentCloud.first, 1.0, 3, 30);
		int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
      std::cout << "cluster size ";
      ppc->numPoints(cluster);
      BoxQ box = ppc->BoundingBoxQ(cluster);
      renderBox(viewer, box, clusterId);
      renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId),colors[clusterId]);
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
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    ProcessPointClouds<pcl::PointXYZI> *ppc = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    std::vector<boost::filesystem::path> stream = ppc->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    
    while (!viewer->wasStopped ())
    {
            // Clear viewer
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            
            cloud = ppc->loadPcd((*streamIterator).string());
            cityBlock(viewer, ppc, cloud);
            
            streamIterator++;
            if(streamIterator == stream.end())
                streamIterator = stream.begin(); 

            viewer->spinOnce ();
    } 
}
