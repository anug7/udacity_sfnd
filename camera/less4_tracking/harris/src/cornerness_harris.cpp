#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "kdtree.h"

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)
    int nms_window = 3;

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);

    KdTree *tree = new KdTree();
    vector<cv::KeyPoint> kps;
    int n = 0;
    for (int i = 0; i < dst_norm_scaled.rows; i++){
        for(int j = 0; j < dst_norm_scaled.cols; j++){
            auto score = dst_norm_scaled.at<uchar>(i, j);
            if (score >= minResponse){
                cv::KeyPoint kp(cv::Point2f(j, i), 2, 0, score, 0, n);
                vector<int> ids = tree->search(kp, nms_window);
                if (ids.size() == 0){
                    tree->insert(kp, n++);
                    kps.push_back(kp);
                }else{
                    auto prev_kp = kps[ids[0]];
                    if (prev_kp.response < kp.response){
                        kps[ids[0]] = kp;
                    }
                }
            }
        }
    }
    std::cout << "Total size: " << kps.size() << "\n";
    cv::drawKeypoints(dst_norm_scaled, kps, dst_norm_scaled);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);
}

int main()
{
    cornernessHarris();
}
