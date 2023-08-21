#include "kitti.h"
#include <iostream>

int main(){

    std::string path_to_kitti_dataset = "/home/ben/vision/data/kitti/dataset";
    std::string path_to_calib_file = "/home/ben/vision/data"
    int sequence = 0;
    kitti dataloader(path_to_kitti_dataset, sequence);
    cv::Mat leftIm;
    cv::Mat rightIm;
    cv::Mat depthIm;
    while (dataloader.next()){
        dataloader.grabLeftCameraFrame(leftIm);
        cv::imshow("left image", leftIm);
        dataloader.grabRightCameraFrame(rightIm);
        cv::imshow("right image", rightIm);
        dataloader.grabDepthImage(depthIm);
        cv::imshow("Depth Image", depthIm);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    return 0;
}