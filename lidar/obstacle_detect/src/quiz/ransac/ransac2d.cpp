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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
  int maxIdx = cloud->size();
  int maxInliers = 0;
  int curInliers = 0;
  for (int i = 0; i < maxIterations; i++){
    int idx1 = rand() % maxIdx,
        idx2 = rand() % maxIdx;
    std::unordered_set<int> tmp;
    while (idx2 == idx1){
        idx2 = rand() % maxIdx;
    }
    tmp.insert(idx1);
    tmp.insert(idx2);
    pcl::PointXYZ p1 = cloud->at(idx1),
                  p2 = cloud->at(idx2);
    double a = p1.y - p2.y, b = p2.x - p1.x, c = p1.x * p2.y - p2.x * p1.y;
    double sqrtab = sqrt(a*a + b*b);
    curInliers = 0;
    for(int j = 0; j < maxIdx; j++){
      if (idx1 == j || idx2 == j)
        continue;
      pcl::PointXYZ p = cloud->at(j);
      double dist = fabs(a * p.x + b * p.y + c) / sqrtab;
      if (dist <= distanceTol){
        curInliers++;
        tmp.insert(j);
      }
    }
    if(curInliers > maxInliers){
      maxInliers = curInliers;
      inliersResult = tmp;
    }

  }
  std::cout << "Cur inliers: " << maxInliers << "\n";
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
  int maxIdx = cloud->size();
  int maxInliers = 0;
  int curInliers = 0;
  for (int i = 0; i < maxIterations; i++){
    std::unordered_set<int> tmp;
    while (tmp.size() < 3){
        tmp.insert(rand() % maxIdx);
    }
    auto itr = tmp.begin();
    pcl::PointXYZ p1 = cloud->at(*(itr++)),
                  p2 = cloud->at(*(itr++)),
                  p3 = cloud->at(*(itr));
    double a = ((p2.y - p1.y) * (p3.z - p1.z)) - ((p2.z - p1.z) * (p3.y - p1.y)),
           b = ((p2.z - p1.z) * (p3.x - p1.x)) - ((p2.x - p1.x) * (p3.z - p1.z)),
           c = ((p2.x - p1.x) * (p3.y - p1.y)) - ((p2.y - p1.y) * (p3.x - p1.x)),
           d = -1 * (a * p1.x + b * p1.y + c * p1.z);
    double sqrtab = sqrt(a*a + b*b + c*c);
    curInliers = 0;
    for(int j = 0; j < maxIdx; j++){
      if (tmp.find(j) != tmp.end())
        continue;
      pcl::PointXYZ p = cloud->at(j);
      double dist = fabs(a * p.x + b * p.y + c * p.z + d) / sqrtab;
      if (dist <= distanceTol){
        curInliers++;
        tmp.insert(j);
      }
    }
    if(curInliers > maxInliers){
      maxInliers = curInliers;
      inliersResult = tmp;
    }

  }
  std::cout << "Cur inliers: " << maxInliers << "\n";
	return inliersResult;

}



int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	//std::unordered_set<int> inliers = Ransac(cloud, 50, 0.2);
  auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.5);
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

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
