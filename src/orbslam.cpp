#include <iostream>

#include <opencv2/opencv.hpp>

// Include GLM
#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>

#include "ORB_SLAM/System.h"

using namespace std;
using namespace cv;

Mat map11, map12, map21, map22, RCalib, TCalib, P1, P2, K;
Mat initRvec, initTvec;
Mat R=Mat::eye(4, 4, CV_32F);
Mat tvec = cv::Mat::zeros(4, 1, CV_32F);
cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_32F);

Mat getCameraMatrix(){
    return K;
}

void InitMat(Mat& m,float* num)
{
 for(int i=0;i<m.rows;i++)
  for(int j=0;j<m.cols;j++)
   m.at<float>(i,j)=*(num+i*m.rows+j);
}


void stereoRemap(Mat frame_left, Mat frame_right, Mat& frame_left_rectified, Mat& frame_right_rectified){

    remap(frame_left, frame_left_rectified, map11, map12, cv::INTER_LINEAR);
    remap(frame_right, frame_right_rectified, map21, map22, cv::INTER_LINEAR);

}

glm::mat4 getViewMatrix(bool slamMode){
    glm::mat4 V;
    Mat viewMatrix = cv::Mat::zeros(4, 4, CV_32F);

    if (slamMode){


        // for(unsigned int row=0; row<3; ++row)
        // {
        //     for(unsigned int col=0; col<3; ++col)
        //     {
        //         viewMatrix.at<double>(row, col) = R.at<double>(row, col);
        //     }
        //     viewMatrix.at<double>(row, 3) = tvec.at<double>(row, 0);
        // }
        // viewMatrix.at<double>(3, 3) = 1.0f;

        // viewMatrix = cvToGl * viewMatrix;
            float qx,qy,qz,qw;
    qw = sqrt(1.0 + R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2)) / 2.0;
    qx = (R.at<float>(2,1) - R.at<float>(1,2)) / (4*qw) ;
    qy = -(R.at<float>(0,2) - R.at<float>(2,0)) / (4*qw) ;
    qz = -(R.at<float>(1,0) - R.at<float>(0,1)) / (4*qw) ;


float m0[]={1 - 2*qy*qy - 2*qz*qz, 2*qx*qy + 2*qz*qw, 2*qx*qz - 2*qy*qw, 0, 2*qx*qy - 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz + 2*qx*qw, 0, 2*qx*qz + 2*qy*qw, 2*qy*qz - 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, 0, tvec.at<float>(0, 0), -tvec.at<float>(1, 0), -tvec.at<float>(2, 0), 1};
InitMat(viewMatrix, m0);







    } else{
        viewMatrix = cvToGl;
    }

    // cv::Mat Rcw = viewMatrix.rowRange(0,3).colRange(0,3);
    // cv::Mat tcw = viewMatrix.rowRange(0,3).col(3);
    // cv::Mat Rwc = Rcw.t();
    // cv::Mat Ow = -Rwc*tcw;

    // cv::Mat Twc = cv::Mat::eye(4,4,viewMatrix.type());
    // Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    // Ow.copyTo(Twc.rowRange(0,3).col(3));


    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++) {
            V[i][j] = viewMatrix.at<float>(i,j);
        }
    }
glm::mat4 myScalingMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(1));;
V=V*myScalingMatrix;
    return V;


}

glm::mat4 getInitModelMatrix(){
    glm::mat4 initModelMatrix;
    Mat initR;
    Mat viewMatrix = cv::Mat::zeros(4, 4, CV_32F);
    Rodrigues(initRvec, initR);

    for(unsigned int row=0; row<3; ++row)
    {
        for(unsigned int col=0; col<3; ++col)
        {
            viewMatrix.at<double>(row, col) = initR.at<double>(row, col);
        }
        viewMatrix.at<double>(row, 3) = initTvec.at<double>(row, 0);
    }
    viewMatrix.at<double>(3, 3) = 1.0f;

    //viewMatrix = cvToGl * viewMatrix;

    viewMatrix.convertTo(viewMatrix, CV_32F);


    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++) {
            initModelMatrix[i][j] = viewMatrix.at<float>(j,i);
        }
    }

    return initModelMatrix;
}

bool initTracking(const char * Remap_path, const char * Extrinsics_path){

    cvToGl.at<double>(0, 0) = 1.0f;
    cvToGl.at<double>(1, 1) = -1.0f;
// Invert the y axis
    cvToGl.at<double>(2, 2) = -1.0f;
// invert the z axis
    cvToGl.at<double>(3, 3) = 1.0f;
   //cvToGl.at<double>(2, 3) = -10.0f;

    cv::FileStorage f1,f2;
    f1.open(Remap_path, cv::FileStorage::READ);
    f2.open(Extrinsics_path, cv::FileStorage::READ);

    if (f1.isOpened()&&f2.isOpened()){
        f1["map11"] >> map11;
        f1["map12"] >> map12;
        f1["map21"] >> map21;
        f1["map22"] >> map22;
        f2["R"] >> RCalib;
        f2["T"] >> TCalib;
        f2["P1"] >> P1;
        f2["P2"] >> P2;
        f1.release();
        f2.release();
    } else{
        cout << "Couldn't open Remap.xml or Extrinsics.xml" << endl;
        return 0;
    }

    P1.rowRange(0,3).colRange(0,3).copyTo(K);

    return 1;
}

bool trackStereo(Mat CameraPose){

    if (CameraPose.empty()) {
        return 0;
    }

    CameraPose.rowRange(0,3).colRange(0,3).copyTo(R);
    CameraPose.rowRange(0,3).col(3).copyTo(tvec);

    return 1;
}

bool TryInitModelMatrix(Mat frame_left, bool slamMode){

    if (slamMode)
        return 1;

    Mat frame_left_rectified;
    vector<cv::Point2f> ImagePoints_1;
    vector<Point3f> ObjectPoints;

    cv::Size ChessboardSize(8,6);

    for (int p = 0; p < ChessboardSize.height; p++) {
        for (int q = 0; q < ChessboardSize.width; q++) {
            ObjectPoints.push_back(Point3f(q,p,0));
        }
    }
    bool found_left;


    remap(frame_left, frame_left_rectified, map11, map12, cv::INTER_LINEAR);

    found_left = cv::findChessboardCorners(frame_left_rectified, ChessboardSize, ImagePoints_1, cv::CALIB_CB_FAST_CHECK);

    if (found_left) {

        solvePnP(ObjectPoints, ImagePoints_1, K, noArray(), initRvec, initTvec);

        return 1;

    } else
        return 0;
}