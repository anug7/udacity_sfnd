// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>()), 
                                          filteredRegion(new pcl::PointCloud<PointT>()),
                                          filteredOut(new pcl::PointCloud<PointT>());
    
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*filteredCloud);

    pcl::CropBox<PointT> cbox;
    cbox.setMin(minPoint);
    cbox.setMax(maxPoint);
    cbox.setInputCloud(filteredCloud);
    cbox.filter(*filteredRegion);

    //Remove roof points from point cloud
    pcl::ExtractIndices<PointT> extract;
    pcl::CropBox<PointT> roofBox;
    roofBox.setMin({-1.5, -1.7, -1, 1});
    roofBox.setMax({2.6, 1.7, -0.4, 1});
    roofBox.setInputCloud(filteredRegion);
    std::vector<int> rIndices;
    roofBox.filter(rIndices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int i: rIndices)
        inliers->indices.push_back(i);

    extract.setInputCloud(filteredRegion);
    extract.setIndices (inliers);
    extract.setNegative(true);
    extract.filter(*filteredOut);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    return filteredOut;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{

    typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>()),
                            obst(new pcl::PointCloud<PointT>());
    //Filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices (inliers);
    extract.setNegative(false);
    extract.filter(*plane);

    extract.setNegative (true);
    extract.filter (*obst);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
   	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree( new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ext;
    
    ext.setClusterTolerance (clusterTolerance); // 2cm
    ext.setMinClusterSize (minSize);
    ext.setMaxClusterSize (maxSize);
    ext.setSearchMethod (tree);
    ext.setInputCloud (cloud);
    ext.extract (cluster_indices);

  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
  	  typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
  	  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  	    cloud_cluster->push_back ((*cloud)[*pit]); //*
  	  cloud_cluster->width = cloud_cluster->size ();
  	  cloud_cluster->height = 1;
  	  cloud_cluster->is_dense = true;

      clusters.push_back(cloud_cluster);
  	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
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


/*Take from : https://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
  */
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    Eigen::Vector4f cen({0., 0, 0, 0});
    pcl::compute3DCentroid(*cluster, cen);
    Eigen::Matrix3f covariance;

    pcl::computeCovarianceMatrixNormalized(*cluster, cen, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVecPCA = eigen_solver.eigenvectors();
    // This line is necessary for proper orientation in some cases. The numbers come out the same without it, bu
    // the signs are different and the box doesn't get correctly oriented in some cases
    eigenVecPCA.col(2) = eigenVecPCA.col(0).cross(eigenVecPCA.col(1));

    //Transform points to eigen space to get min and max of bounding box
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVecPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * cen.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    const Eigen::Quaternionf bboxQuaternion(eigenVecPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVecPCA * meanDiagonal + cen.head<3>();

    BoxQ boxq;
    boxq.bboxQuaternion = bboxQuaternion;
    boxq.bboxTransform = bboxTransform;
    boxq.cube_length = maxPoint.x - minPoint.x;
    boxq.cube_width = maxPoint.y - minPoint.y;
    boxq.cube_height = maxPoint.z - minPoint.z;
    
    return boxq;
}

/*
 * Exercise Functions
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  auto startTime = std::chrono::steady_clock::now();
	
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
    PointT p1 = cloud->at(*(itr++)),
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
      PointT p = cloud->at(j);
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
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
  
  typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>()),
                          obst(new pcl::PointCloud<PointT>());
	
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst, plane);
  for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			segResult.second->points.push_back(point);
		else
			segResult.first->points.push_back(point);
	}

  return segResult;
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
