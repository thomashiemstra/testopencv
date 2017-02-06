#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <list>



using namespace cv;
using namespace std;
int i;
char imageName[1024];
const int fps = 5;

const float calibrationSquareDimension = 0.026f; //meters
const float arucoSquareDimension = 0.0661f;
const Size chessboardDimensions = Size(6,9);

void createArucoMarkers(){
    Mat outputMarker;
    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    for(i=0;i<50;i++){
        aruco::drawMarker(markerDictionary, i, 500, outputMarker,1);
        sprintf(imageName,"dump/4x4_%d.jpg",i);
        printf(imageName); printf("\n");
        imwrite(imageName,outputMarker );
    }
}

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners){
    for(int i =0; i<boardSize.height; i++){
        for(int j = 0; j<boardSize.width; j++){
            corners.push_back(Point3f(j*squareEdgeLength, i*squareEdgeLength, 0.0f));
        }
    }
}

void getChessboardCorners( vector<Mat> images, vector<vector<Point2f> >& allFoundCorners, bool showResults = false ){
    for(vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++){
        vector<Point2f> pointBuf;
        bool found = findChessboardCorners(*iter, Size(9,6), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        if(found){
            allFoundCorners.push_back(pointBuf);
        }
        if(showResults){
                drawChessboardCorners(*iter, Size(9,6), pointBuf, found);
                imshow("looking for corners", *iter);
                waitKey(0);
           }
    }

}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients){
    vector<vector<Point2f> > checkerboardImageSPacePoints;
    getChessboardCorners(calibrationImages, checkerboardImageSPacePoints, false);

    vector<vector<Point3f> > worldSpaceCornerPoints(1);

    createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerboardImageSPacePoints.size(), worldSpaceCornerPoints[0]);

    vector<Mat> rVectors, tVectors;

    calibrateCamera(worldSpaceCornerPoints, checkerboardImageSPacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients){
    ofstream outStream(name);
    if(outStream){
        uint16_t rows = cameraMatrix.rows;
        uint16_t columns = cameraMatrix.cols;

        for(int r = 0; r < rows; r++){
            for(int c = 0; c < columns; c++){
                double value = cameraMatrix.at<double>(r,c);
                outStream << value << endl;
            }
        }

        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;


        for(int r = 0; r < rows; r++){
            for(int c = 0; c < columns; c++){
                double value = distanceCoefficients.at<double>(r,c);
                outStream << value << endl;
            }
        }
        outStream.close();
        return true;
    }
    return false;
}

bool getMatrixFromFile(string name, Mat cameraMatrix, Mat distanceCoefficients){
    ifstream inStream(name);
    if(inStream){

        uint16_t rows = cameraMatrix.rows;
        uint16_t columns = cameraMatrix.cols;

        for(int r = 0; r < rows; r++){
            for(int c = 0; c < columns; c++){
                inStream >> cameraMatrix.at<double>(r,c);
            }
        }

        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;


        for(int r = 0; r < rows; r++){
            for(int c = 0; c < columns; c++){
                inStream >> distanceCoefficients.at<double>(r,c);
            }
        }

        inStream.close();
        return true;
    }
    return false;
}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension ){

    Mat frame;

    vector<int> markerIds(2);
    vector<vector<Point2f> > markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameters;

    Ptr< aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    VideoCapture vid(0);

    if(!vid.isOpened()){
        return -1;
    }

    namedWindow("Webcam",CV_WINDOW_AUTOSIZE);

    vector<Vec3d> rotationVectors, translationVectors;

    while(true){
        if(!vid.read(frame))
            break;

        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);

        int max = markerIds.size();
        //cout << "\r" << "markerIds[0]=" << markerIds[0] << "  markerIds[1]=" << markerIds[1] << " markerIds[2]=" << markerIds[2] <<"                   " << flush;
        if(translationVectors.size() > 1 ){
            cout << "\r" << "dx=" << translationVectors[0][0] -  translationVectors[1][0] << " dy=" << translationVectors[0][1] -  translationVectors[1][1] << " dz=" << translationVectors[0][2] -  translationVectors[1][2] << "                   " << flush;
        }

        for(int i = 0; i < max; i++){
            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
        }
        imshow("Webcam", frame);
        if(waitKey(1000/fps) == 27){ //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
            break;
        }

    }
    return 1;


}

bool calibrateRoutine(int cameraNumber, Mat cameraMatrix, Mat distanceCoefficients){

    Mat frame;
    Mat drawToFrame;
    vector<Mat> savedImages;

    vector<vector<Point2f> > markerCorners, rejectedCandidates;

    VideoCapture vid(cameraNumber);

    if(!vid.isOpened()){
        return false;
    }

    namedWindow("webcam", CV_WINDOW_AUTOSIZE);

    while(true){
        if(!vid.read(frame)){
            break;
        }
        vector<Vec2f> foundPoints;
        bool found = false;

        found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
        if(found)
            imshow("Webcam", drawToFrame);
        else
            imshow("Webcam", frame);
        char character = waitKey(1000/fps);

        switch(character)
        {
            case' ':
                //saving image
                if(found){
                    Mat temp;
                    frame.copyTo(temp);
                    savedImages.push_back(temp);
                   cout << " total = " << savedImages.size() << "\n";
                }
                break;
            case 13: //enter key
                //start calibration
                if(savedImages.size() > 15){
                    cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients  );
                    saveCameraCalibration("CameraCalibration", cameraMatrix, distanceCoefficients);
                    printf("dikke");
                }
                break;
            case 27: //escape key
                //exit program
                return true;
                break;
        }

    }
    return false;

}

int main(){

    Mat cameraMatrix = Mat::eye(3,3, CV_64F);
    Mat distanceCoefficients = Mat::zeros(5,1, CV_64F);
    getMatrixFromFile("CameraCalibration", cameraMatrix, distanceCoefficients);
    startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension);

    return 0;
}
