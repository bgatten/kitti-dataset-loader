#ifndef KITTI_H
#define KITTI_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp> //stereo matching
#include <fstream>
#include <filesystem>
// #include <opencv4/opencv2/opencv.hpp>
// #include <opencv2/opencv.hpp>

class kitti{
    private:
        std::string _path; // Path to KITTI dataset
        int _sequence; //sequence in the KITTI dataset. e.g. 00, 01, etc.
        int _frame; // Frame number
        int _numFrames; // number of camera frames in the dataset.
        //calibration matrix of the left camera. (P2 in calib.txt)
        Eigen::MatrixXd _calibration; //Intrinsic calibration matrix
        float _cx; //center of the frame (x-pixel)
        float _cy; //center of the frame (y-pixel)
        float _fx; //focal length (x)
        float _fy; //focal length (y)
        float _cameraBaseline = 0.54; //[m], Distance between left and right camera lens
        long _timestamp; //timestamp for the current frame.
        cv::Mat _imageLeft;
        cv::Mat _imageRight;
        cv::Mat _disparityImage;
        cv::Ptr<cv::StereoSGBM> _stereoMatcher;	
        void _setCameraIntrinsics();
        std::string _convertToPaddedString(int number, int padding) const;

        // cv::Mat depthImage;
    public:
        //TODO: Add documentation for methods.
        kitti(std::string path, int sequence);
        bool grabLeftCameraFrame(cv::Mat &frame);
        bool grabRightCameraFrame(cv::Mat &frame);
        void grabDepthImage(cv::Mat &depthImage);
        void grab3DFromPixel(int u, int v, float &x, float &y, float &z) const;
        float grabDepthFromPixel(int u, int v) const;
        bool next();
        ~kitti();
};

#endif