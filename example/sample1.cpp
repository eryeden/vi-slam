#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
    std::cout << "Hello sample" << std::endl;

    cv::Mat mat = cv::imread("/home/ery/Desktop/IMG_4277-e1495593275855-150x150.jpg");
    cv::imshow("Test", mat);
    cv::waitKey(0);
}