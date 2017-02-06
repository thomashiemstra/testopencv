#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>


using namespace cv;
char name[1024];
int r,c,i;
int main() {

    Mat original = imread("test.jpg", CV_LOAD_IMAGE_COLOR);
    Mat modified = imread("test.jpg", CV_LOAD_IMAGE_COLOR);
    //imshow("Original", original);
    for(i = 0; i<3; i++){
        for(r = 0; r< modified.rows; r++){
            for(c = 0; c < modified.cols; c++){
                modified.at<cv::Vec3b>(r,c)[i] = modified.at<cv::Vec3b>(r,c)[i] * 0;
            }
        }
        sprintf(name,"%d",i);
        imshow(name, modified);
        modified = imread("test.jpg", CV_LOAD_IMAGE_COLOR);

    }

waitKey();


}
