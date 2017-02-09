#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <list>

#define radtodeg 57.29577951

using namespace cv;
using namespace std;
int i;
char imageName[1024];
const int fps = 10;

const float calibrationSquareDimension = 0.026f; //meters
const float arucoSquareDimension = 0.0397f;
const Size chessboardDimensions = Size(6,9);

void createArucoMarkers(){
    Mat outputMarker;
    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_7X7_50);

    for(i=0;i<50;i++){
        aruco::drawMarker(markerDictionary, i, 150, outputMarker,1);
        sprintf(imageName,"dump/7x7_%d.jpg",i);
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

int findIndex(vector<int>& vec, int val){
    int res;
    uint16_t length = vec.size();
    res = find(vec.begin(), vec.end(), val) - vec.begin();
        if (res >= length){
            res = -1;
        }
    return res;
}

void findRelativeVectors(int basePos, int Pos, vector<Vec3d>& translationVectors, vector<Vec3d>& rotationVectors, vector<double>& posRes, vector<double>& Rotres ){
    int i,j;
    vector<double> R(3);
    Mat baseRotMatrix = Mat::eye(3,3, CV_64F);

    /* posRes is the vector from the world coordinate system to object 1 expressed in world base vectors*/
    /* R is the vector from object to base in expressed in the camera frame*/
    for(i = 0; i < 3; i++)
        R[i] = translationVectors[Pos][i] -  translationVectors[basePos][i];

    /* R is still expressed with respect to the camera frame, to fix this we must multiply R by the transpose of the rotation matrix between the world and camera frame */
    Rodrigues(rotationVectors[basePos],baseRotMatrix);

    for(i = 0; i < 3; i++){
        posRes[i] = 0;
        for(j = 0; j < 3; j++){
            posRes[i] += baseRotMatrix.at<double>(j,i)*R[j];
        }
    }

    /* since the rotation "vector" are just the 3 euler angles we simply substract (I hope)*/
    for(i = 0; i < 3; i++)
        Rotres[i] = rotationVectors[Pos][i] - rotationVectors[basePos][i];

}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension ){

    Mat frame;
    vector<int> markerIds(2);
    vector<vector<Point2f> > markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameters;
    vector<Vec3d> rotationVectors, translationVectors;
    vector<double> relPos1(3), relRot1(3);

    Ptr< aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_7X7_50);

    VideoCapture vid(0);

    if(!vid.isOpened()){
        return -1;
    }

    namedWindow("Webcam",CV_WINDOW_AUTOSIZE);

    while(true){
        if(!vid.read(frame))
            break;

        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);

        int max = markerIds.size();
        /* I take marker number 49 to mark my world coordinate system. I need it's index to find the vectors corresponding to it*/
        int basePos = findIndex(markerIds, 49);
        /* marker 48 will be used as object 1 which I want to know the position of with respect to the base coordinate system (marker 49)*/
        int Pos1 = findIndex(markerIds, 48);

        //cout << "\r" << "pos=" << basePos << "markerIds[0]=" << markerIds[0] << "  markerIds[1]=" << markerIds[1] << " markerIds[2]=" << markerIds[2] <<"                   " << flush;

        if(Pos1 != -1 && basePos != -1){
            findRelativeVectors(basePos, Pos1, translationVectors, rotationVectors, relPos1, relRot1);
            cout << "\r" << Pos1 << " "<<basePos << " dx=" << 100*relPos1[0] << " dy=" << 100*relPos1[1] << " dz=" << 100*relPos1[2] << "                   " << flush;
        }


        for(int i = 0; i < max; i++){
            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.08f);
        }
        imshow("Webcam", frame);
        if(waitKey(1000/fps) == 27){ //wait for 'esc' key press. If 'esc' key is pressed, break loop

            /* baseMatrix is the wrong name but I'm to lazy to change it */
            Mat baseMatrix = Mat::eye(3,3, CV_64F);
            Rodrigues(relRot1,baseMatrix);
            printf("\n%lf %lf %lf", baseMatrix.at<double>(0,0), baseMatrix.at<double>(0,1), baseMatrix.at<double>(0,2));
            printf("\n%lf %lf %lf", baseMatrix.at<double>(1,0), baseMatrix.at<double>(1,1), baseMatrix.at<double>(1,2));
            printf("\n%lf %lf %lf", baseMatrix.at<double>(2,0), baseMatrix.at<double>(2,1), baseMatrix.at<double>(2,2));
            //printf("%lf %lf %lf \n ", rotationVectors[0][0]*radtodeg, rotationVectors[0][1]*radtodeg, rotationVectors[0][2]*radtodeg);
            //waitKey();
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

    namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

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
                   cout << "\r" << " total = " << savedImages.size() << "                   " << flush;
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

    //createArucoMarkers();
    //calibrateRoutine(0, cameraMatrix, distanceCoefficients);

    getMatrixFromFile("CameraCalibration", cameraMatrix, distanceCoefficients);
    startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension);

    return 0;
}
