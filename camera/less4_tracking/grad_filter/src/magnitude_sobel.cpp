#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
    
    float sobel_x[] = {-1, 0, 1,
                       -2, 0, 2,
                       -1, 0, 1
                      };
    float sobel_y[] = { -1, -2, -1,
                         0,  0,  0,
                         1,  2,  1
                      };
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);
    cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);

    cv::Mat result_x, result_y;
    cv::filter2D(imgGray, result_x, -1, kernel_x, cv::Point(-1, -1), 0,
                 cv::BORDER_DEFAULT);
    cv::filter2D(imgGray, result_y, -1, kernel_y, cv::Point(-1, -1), 0,
                 cv::BORDER_DEFAULT);

    cv::Mat magnitude = (result_x ^ 2 + result_y ^ 2)^-2;
    // show result
    string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitude);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}
