#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;

int main() {
    Mat testColor = imread("test.jpg",CV_LOAD_IMAGE_COLOR);

    Mat testGray = imread("test.jpg",CV_LOAD_IMAGE_GRAYSCALE);

    imshow("lol2",testGray);
    imshow("lol",testColor);

    imwrite("outputGrey.jpg",testGray);

    waitKey();
}
