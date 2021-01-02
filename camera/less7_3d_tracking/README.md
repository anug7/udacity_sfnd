# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.


Review

**FP.1 Match 3D Objects**
Bounding boxes from current and previous frames are matched using the keypoints present in side them. And boxId with maximum no of occurences is considered as the matched box.
```
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int idxArray[prevFrame.boundingBoxes.size()][currFrame.boundingBoxes.size()] ={0};
    for (auto &mat : matches)
    {
        auto cKp = currFrame.keypoints[ mat.trainIdx ],
             pKp = prevFrame.keypoints[ mat.queryIdx ];

        for (auto pBBox : prevFrame.boundingBoxes)
        {
            for (auto cBBox : currFrame.boundingBoxes)
            {
                if (pBBox.roi.contains(pKp.pt) && cBBox.roi.contains(cKp.pt))
                {
                    auto pIdx = pBBox.boxID,
                         cIdx = cBBox.boxID;
                    idxArray[pIdx][cIdx]++;
                }
            }
        }
    }

    for (int i = 0; i < prevFrame.boundingBoxes.size(); i++){
      int cIdx = -1;
      int cPts = 0;
      for (int j = 0; j < currFrame.boundingBoxes.size(); j++){
        if (idxArray[i][j] > cPts){
          cIdx = j;
          cPts = idxArray[i][j];
        }
      }
      if (cIdx != -1){
        bbBestMatches.emplace(i, cIdx);
      }
    }

}
```
**FP.2 Compute Lidar-based TTC**
The distance to collision is calculated based on bunch of lidar points, corresponding to the vehicle, which are closer to ego lane. In order to accomodate for the outliers, mean is computed to reject some of the outlier. 
```
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double laneWidth = 4.0; // assumed width of the ego lane

    double prevCen = 0.0, currCen = 0.0;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        prevCen += it->y;
    }
    prevCen /= lidarPointsPrev.size();

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        currCen += it->y;
    }
    currCen /= lidarPointsCurr.size();

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (it->y >=  prevCen - laneWidth / 2. && it->y <= prevCen + laneWidth /2.0)
            minXPrev = minXPrev > it->x ? it->x : minXPrev;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (it->y >=  currCen - laneWidth / 2. && it->y <= currCen + laneWidth /2.0)
            minXCurr = minXCurr > it->x ? it->x : minXCurr;
    }

    // compute TTC from both measurements
    TTC = minXCurr * (1/frameRate) / abs(minXPrev - minXCurr);
}
```
**FP.3 Associate Keypoint Correspondences with Bounding Boxes**
For the TTC based on Camera, computation of keypoints to each of the bounding boxes is required. This is done by iterating to all keypoints, which are matched across the previous and current frame, and finding them ones which are inside the bounding boxes. Also, used distance matric from DMatches to reject outliers to some level based on the mean of all the enclosed points. Can be improved further.
```
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::DMatch> filtMat;
    float meanDist = 0;
    for (auto &mat : kptMatches)
    {
        auto cKp = kptsCurr[ mat.trainIdx ],
             pKp = kptsPrev[ mat.queryIdx ];
        if (boundingBox.roi.contains(cKp.pt)){
          meanDist += mat.distance;
          filtMat.push_back(mat);
        }
    }
    meanDist = meanDist / filtMat.size();

    for (auto itr = filtMat.begin(); itr != filtMat.end(); itr++)
    {
        if (itr->distance < meanDist)
        {
            boundingBox.kptMatches.push_back(*itr);
            boundingBox.keypoints.push_back(kptsCurr[itr->trainIdx]);
        }
    }
}
```
**FP.4 Compute Camera-based TTC**
For calculating the TTC based on camera, the distance of the matched keypoints between the current and previous frames are used. The mean distance of the matched keypoints are calculated to compute the value.
```
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();

    double dT = 1 / frameRate;
    TTC = -dT / (1 - meanDistRatio);

    // TODO: STUDENT TASK (replacement for meanDistRatio)
    int no_of_points = distRatios.size();
    sort(distRatios.begin(), distRatios.end());
    double medDistRatio = 0.0;
    if (no_of_points % 2){
            medDistRatio = distRatios[no_of_points / 2 + 1];
    }else{
            medDistRatio = (distRatios[no_of_points / 2] +
                            distRatios[no_of_points / 2 + 1]) / 2.;
    }
    TTC = -dT / (1 - medDistRatio);
}
```
**FP.5 Performance Evaluation 1**
There are cases where the TTC lidar is off from the actual. This is because of the method used to compute the x min which can be influenced by lot of points present on curvy regions and spread wider which are not accounting to the actual min point to the car. Also, using proper shrink factor would improve the results.

![img1](https://github.com/anug7/udacity_sfnd/blob/dev/camera/less7_3d_tracking/refs/ttc_off_lidar_1.png)
![img2](https://github.com/anug7/udacity_sfnd/blob/dev/camera/less7_3d_tracking/refs/ttc_off_lidar_2.png)

**FP.6 Performance Evaluation 2**
Different KP and decriptors were evaluated for measuring the performance in terms of quality of the results and the time. It's mainly measured based on deviation from average and compared with the TTC of lidar. By that, it seems FAST/ORB is the good combination interms of time and quality of results. The following image is attached for timing and quality of matching.

![Results](https://github.com/anug7/udacity_sfnd/blob/dev/camera/less7_3d_tracking/images/perf.png)
![Points](https://github.com/anug7/udacity_sfnd/blob/dev/camera/less7_3d_tracking/images/tabular.png)

