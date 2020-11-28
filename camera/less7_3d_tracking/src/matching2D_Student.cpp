
#include <numeric>
#include <opencv2/features2d.hpp>
#include "matching2D.hpp"
#include "kdtree.h"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_L1;
        //int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        if (descSource.type() != CV_32F || descSource.type() != CV_8U 
            || descSource.type() != CV_32S)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "(NN) "<< matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        std::vector <std::vector<cv::DMatch> > raw_matches;
        double t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, raw_matches, 2);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << raw_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
        
        const float ratio_thresh = 0.8f;
        for (size_t i = 0; i < raw_matches.size(); i++)
        {
                if (raw_matches[i][0].distance < ratio_thresh * raw_matches[i][1].distance)
                {
                        matches.push_back(raw_matches[i][0]);
                }
        }
    }
    std::cout << "Match size: "<< matches.size() << "\n";
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
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
    else if(descriptorType.compare("SIFT") == 0)
    {
        int nfeatures = 0, nOctaveLayers = 4;
        double contrastThreshold = 0.040000000000000001,
               edgeThreshold = 10, sigma = 1.6000000000000001;
        extractor = cv::xfeatures2d::SIFT::create(nfeatures, nOctaveLayers, 
                                                  contrastThreshold,
                                                  edgeThreshold,
                                                  sigma);
    }
    else if(descriptorType.compare("ORB") == 0){
        extractor = cv::ORB::create(cv::ORB::FAST_SCORE);
    }
    else if(descriptorType.compare("FREAK") == 0){
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if(descriptorType.compare("AKAZE") == 0){
        cv::AKAZE::DescriptorType descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
        int descriptor_size = 0, descriptor_channels = 3;
        float threshold = 0.00100000005F;
        int nOctaves = 4;
        int nOctaveLayers = 4;
        cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;
        extractor = cv::AKAZE::create(descriptor_type, descriptor_size,
                                      descriptor_channels, threshold,
                                      nOctaves, nOctaveLayers, diffusivity);
    }else if(descriptorType.compare("BRIEF") == 0){
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }else{
        std::cout << "Invalid descriptor selection..\n";
        exit(-1);
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        newKeyPoint.class_id = it - corners.begin();
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis){
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)
    int nms_window = 3;

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    double t = (double)cv::getTickCount();
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);
    
    KdTree *tree = new KdTree();
    int n = 0;
    for (int i = 0; i < dst_norm_scaled.rows; i++){
        for(int j = 0; j < dst_norm_scaled.cols; j++){
            auto score = dst_norm_scaled.at<uchar>(i, j);
            if (score >= minResponse){
                cv::KeyPoint kp(cv::Point2f(j, i), 2, 0, score, 0, n);
                vector<int> ids = tree->search(kp, nms_window);
                if (ids.size() == 0){
                    tree->insert(kp, n++);
                    keypoints.push_back(kp);
                }else{
                    auto prev_kp = keypoints[ids[0]];
                    if (prev_kp.response < kp.response){
                        keypoints[ids[0]] = kp;
                    }
                }
            }
        }
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris Corner detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis){
   cv::Ptr<cv::FeatureDetector> detector;
   if (detectorType.compare("BRISK") == 0){
     detector = cv::BRISK::create();
   }else if (detectorType.compare("FAST") == 0){
     detector = cv::FastFeatureDetector::create();
   }else if(detectorType.compare("ORB") == 0){
     detector = cv::ORB::create();
   }else if (detectorType.compare("AKAZE") == 0){
     cv::AKAZE::DescriptorType descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
     int descriptor_size = 0, descriptor_channels = 3;
     float threshold = 0.00100000005F;
     int nOctaves = 4;
     int nOctaveLayers = 4;
     cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;
     detector = cv::AKAZE::create(descriptor_type, descriptor_size,
                                  descriptor_channels, threshold,
                                  nOctaves, nOctaveLayers, diffusivity);
   }else if(detectorType.compare("SIFT") == 0){
     int nfeatures = 0, nOctaveLayers = 4;
     double contrastThreshold = 0.040000000000000001,
            edgeThreshold = 10, sigma = 1.6000000000000001;
     detector = cv::xfeatures2d::SIFT::create(nfeatures, nOctaveLayers,
                                              contrastThreshold,
                                              edgeThreshold, sigma);
   }
   double t = (double)cv::getTickCount();
   detector->detect(img, keypoints);
   t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
   cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
   if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
