#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;

int main() {

    Mat file1 = imread("test.jpg",CV_LOAD_IMAGE_UNCHANGED);
    Mat file2 = imread("test.jpg",CV_LOAD_IMAGE_GRAYSCALE);
    namedWindow("color", CV_WINDOW_FREERATIO);
    namedWindow("fixed", CV_WINDOW_AUTOSIZE);

    imshow("color", file1);
    imshow("fixed", file2);

    //resizeWindow("color", file1.cols/2, file1.rows/2);
    //resizeWindow("fixed", file1.cols/2, file1.rows/2);

    moveWindow("color", 100, 200);
    moveWindow("fixed", 100 + file1.cols , 200);
    waitKey();
}
