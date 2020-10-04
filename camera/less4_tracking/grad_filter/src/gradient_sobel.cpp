#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void gradientSobel()
{
    // TODO: Based on the image gradients in both x and y, compute an image 
    // which contains the gradient magnitude according to the equation at the 
    // beginning of this section for every pixel position. Also, apply different 
    // levels of Gaussian blurring before applying the Sobel operator and compare the results.
    cv::Mat img = cv::imread("../images/img1.png", 0);
    cv::Mat simg;
    cv::GaussianBlur(img, simg, cv::Size(7, 7), 0);

    float sobel_x[] = {-1, 0, 1,
                       -2, 0, 2,
                       -1, 0, 1
                      };
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);

    cv::Mat result_x;
    cv::filter2D(simg, result_x, -1, kernel_x, cv::Point(-1, -1), 0,
                 cv::BORDER_DEFAULT);

    cv::namedWindow("op", 1);
    cv::imshow("op", result_x);
    cv::waitKey(0);
}

int main()
{
    gradientSobel();
}
