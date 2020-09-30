/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include <unordered_set>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  return viewer;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr CreateData(std::vector<std::vector<float>> points)
{
    
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		PointT point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		//point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

template <typename PointT>
void render2DTree(Node<PointT> *node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth % 2 == 0)
		{
			viewer->addLine(pcl::PointXYZ(node->point.x, window.y_min, 0),pcl::PointXYZ(node->point.x, window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point.x;
			upperWindow.x_min = node->point.x;
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point.y,
                              0), pcl::PointXYZ(window.x_max, node->point.y, 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point.y;
			upperWindow.y_min = node->point.y;
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

template <typename PointT>
void proximity(pcl::PointIndices::Ptr cluster, std::unordered_set<int> &is_processed,
               KdTree<PointT> *tree, typename pcl::PointCloud<PointT>::ConstPtr points, float distanceTol, int idx){
  is_processed.insert(idx);
  cluster->indices.push_back(idx);
  pcl::PointIndices::Ptr nearby = tree->search(points->points[idx], distanceTol);
  for(int curIdx: nearby->indices){
    if (is_processed.find(curIdx) == is_processed.end()){
      proximity(cluster, is_processed, tree, points, distanceTol, curIdx);
    }
  }
}

template <typename PointT>
std::vector<pcl::PointIndices::Ptr>
euclideanCluster(typename pcl::PointCloud<PointT>::ConstPtr points, KdTree<PointT> *tree, float distanceTol)
{
  std::vector<pcl::PointIndices::Ptr>  clusters;
  std::unordered_set<int> is_processed;

  for(int i = 0 ; i < points->points.size(); i++){
    if(is_processed.find(i) != is_processed.end()){ //already processed
      continue;
    }else{
      pcl::PointIndices::Ptr cur_cluster(new pcl::PointIndices);
      //proximity function
      proximity(cur_cluster, is_processed, tree, points, distanceTol, i);
      clusters.push_back(cur_cluster);
    }
  }
	return clusters;
}

int main ()
{

	// Create viewer
	Box window;
  window.x_min = -10;
  window.x_max =  10;
  window.y_min = -10;
  window.y_max =  10;
  window.z_min =   0;
  window.z_max =   0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXY>::Ptr cloud = CreateData<pcl::PointXY>(points);

	KdTree<pcl::PointXY> *tree = new KdTree<pcl::PointXY>();
 
  for (int i=0; i< cloud->points.size(); i++){
  	tree->insert(cloud->points[i], i);
  }

  int it = 0;
  render2DTree<pcl::PointXY>(tree->root, viewer, window, it);
  
  std::cout << "Test Search" << std::endl;
  pcl::PointXY testPoint;
  testPoint.x = -6.0f;
  testPoint.y = 7.0f;

  pcl::PointIndices::Ptr nearby = tree->search(testPoint, 3.0);
  for(int index : nearby->indices)
    std::cout << index << ",";
  std::cout << std::endl;

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  //
  std::vector<pcl::PointIndices::Ptr> clusters = euclideanCluster(cloud, tree, 3.0);
  //
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  // Render clusters
  int clusterId = 0;
	std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
  for(pcl::PointIndices::Ptr cluster : clusters)
  {
  	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new  pcl::PointCloud<pcl::PointXYZ>());
  	for(int indice: cluster->indices)
  		clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
  	renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  	++clusterId;
  }
  if(clusters.size()==0){
      pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new  pcl::PointCloud<pcl::PointXYZ>());
      for(int indice = 0; indice < points.size(); indice++)
      		clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
      renderPointCloud(viewer, clusterCloud, "data");
  }
  
	
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce ();
  }
  	
}
