#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>


using namespace cv;

const int fps = 60;

int main() {

    Mat frame;

    VideoCapture vid(0);

    if(!vid.isOpened()){
        return -1;
    }

    while(true){

        vid >> frame;
        imshow("webcam", frame);
        if(waitKey(1000/fps) == 27){ //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
            break;
        }


    }

    return 1;



}
