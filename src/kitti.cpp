#include "kitti.h"

kitti::kitti(std::string path, int sequence){
    std::cout << "Initializing...\n";
    _sequence = sequence;
    _path = path;
    _frame = 0;
    _numFrames = 0;
    int minDisparities = 0;
    int numDisparities = 16*8;
    int windowSize = 3;
    int blockSize = 21;
    blockSize = 5;
    int SADWindowSize = 11;
    // _stereoMatcher = cv::StereoSGBM::create(minDisparities, numDisparities, windowSize, 8*3*windowSize*windowSize, 32*3*windowSize*windowSize);
    _stereoMatcher = cv::StereoSGBM::create(minDisparities, numDisparities, windowSize);

    // _stereoMatcher->setBlockSize(SADWindowSize);
    _setCameraIntrinsics();
    // std::string imageDataPath = _path + "/sequences/0" + _sequence + "/image_0";
    std::string imageDataPath = _path + "/sequences/" + kitti::_convertToPaddedString(_sequence, 2) + "/image_0";
    //TODO: count number of frames in path
    for (const auto& entry : std::filesystem::directory_iterator(imageDataPath)) {
        if (std::filesystem::is_regular_file(entry.path())) {
            _numFrames++;
        }
    }
    std::cout << "Number of frames " << _numFrames << std::endl;
}

void kitti::_setCameraIntrinsics(){
    /*
    In the calib.txt file there are 4 Projection matrices: p0, p1, p2, p3
    P0: Projection matrix of the left color camera.
    P1: Projection matrix of the right color camera.
    P2: Projection matrix of the left grayscale camera.
    P3: Projection matrix of the right grayscale camera.
    */
    // std::string calib_path = _path + "/sequences/0" + std::to_string(_sequence) + "/calib.txt";
    std::string calib_path = _path + "/sequences/" + kitti::_convertToPaddedString(_sequence, 2) + "/calib.txt";
    std::cout << "Grabbing camera calibration from " << calib_path << std::endl;
    std::fstream file;
    file.open(calib_path);
    if (file.is_open()){
        std::string line;
        double P0[3][4];
        while (std::getline(file, line)){
            //Grab P2 values out of calib.txt file for the left camera.
            if (line.substr(0,3) == "P2:"){
                //Extract each value separated by spaces
                std::istringstream iss(line);
                // std::cout << "test " << iss << std::endl;
                double values[12];
                std::sscanf(line.c_str() + 3, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                &values[0], &values[1], &values[2], &values[3], &values[4], &values[5],
                &values[6], &values[7], &values[8], &values[9], &values[10], &values[11]);
                _fx = values[0];
                _cx = values[2];
                _fy = values[5];
                _cy = values[6];
            }
        }//Traverse file
        file.close();
    }
}

std::string kitti::_convertToPaddedString(int number, int padding) const {
    std::stringstream ss;
    ss << std::setw(padding) << std::setfill('0') << number;
    return ss.str();
}

bool kitti::grabLeftCameraFrame(cv::Mat &frame) {
    std::string leftImPath = _path + "/sequences/" + kitti::_convertToPaddedString(_sequence, 2) + "/image_0/" + kitti::_convertToPaddedString(_frame, 6) + ".png";
    // Read the image from the file
    frame = cv::imread(leftImPath, cv::IMREAD_UNCHANGED);
    frame.copyTo(_imageLeft);
    // image_left = cv::imcopy(frame);
    // Check if the image was loaded successfully
    if (frame.empty()) {
        std::cout << "Error: Could not read the image." << std::endl;
        return false;
    }
    return true;
}

bool kitti::grabRightCameraFrame(cv::Mat &frame) {
    std::string rightImPath = _path + "/sequences/" + kitti::_convertToPaddedString(_sequence, 2) + "/image_1/" + kitti::_convertToPaddedString(_frame, 6) + ".png";;
    // Read the image from the file
    frame = cv::imread(rightImPath, cv::IMREAD_UNCHANGED);
    frame.copyTo(_imageRight);
    // _image_right = cv::imcopy(frame);
    // Check if the image was loaded successfully
    if (frame.empty()) {
        std::cout << "Error: Could not read the image." << std::endl;
        return false;
    }
    return true;
}

void kitti::grabDepthImage(cv::Mat &depthImage){
    //todo

    // std::cout << "left image " << _imageLeft.size() << std::endl;
    // std::cout << "right image " << _imageRight.size() << std::endl;
    std::cout << "image number: " << _frame << std::endl;
    _disparityImage= cv::Mat::zeros(_imageLeft.size(), CV_32F);
    _stereoMatcher->compute(_imageLeft, _imageRight, _disparityImage);
    depthImage = cv::Mat::zeros(_disparityImage.size(), CV_32F);
    //calculate depth image.
    for (int y=0; y<_disparityImage.rows; y++){
        for (int x=0; x<_disparityImage.cols; x++){
            float disparity = static_cast<float>(_disparityImage.at<float>(y,x));
            if (disparity != 0 && std::isfinite(disparity)){
                float depth = (_cameraBaseline * _fx ) / disparity;
                if (std::isfinite(depth)){
                    std::cout << "camera baseline: " << _cameraBaseline << std::endl;
                    std::cout << "fx: " << _fx << std::endl;
                    std::cout << "Depth: " << depth << std::endl;
                    std::cout << "Disparity " << disparity << std::endl;
                    depthImage.at<float>(y, x) = depth;
                }
            }
        }
    }
    // depthImage = _disparityImage;
}

// void kitti::_stereoRecfify(){
//     //TODO
// }

float kitti::grabDepthFromPixel(int u, int v) const{
    //todo
    return 1;
}

void kitti::grab3DFromPixel(int u, int v, float &x, float &y, float &z) const{
    /*
    u: x pixel (+ top left corner to right)
    v: y pixel (+ top left corner down)
    z: depth at pixel position (u,v)
    fx: focal length x
    fy: focal length y
    cx: center x
    cy: center y
    x: x-value in world frame (return value)
    y: y-value in world frame (return value)
    z: z-value in world frame (return value)
    */
    z = this->grabDepthFromPixel(u, v);
    x = z*(u - _cx)/_fx;
    y = z*(v - _cy)/_fy;
}

bool kitti::next(){
    //TODO: return false if there are no more frames.
    //Update the left image
    //Update the right image.
    _frame++;
    //no more frames are available
    if (_frame > _numFrames){
        return false;
    }
    //successfully updated to next frame.
    return true;
}

kitti::~kitti(){

}