# Kitti Dataset Loader

Create a **c++** library around the KITTI dataset that can be used in the following way:

kitti kittiDatasetObject(‘/path/to/dataset’);

kittiDatasetObject.getIntrinsicCameraMatrix(eigen::Matrixxd);

cv::Mat frame;

kittaDatasetObject.grabLeftCameraFrame(frame);

cv::Mat cvDepthImage;

kittiDatasetObject.grabDepthImage(cvDepthImage);

u: (x-pixel)

v: (y-pixel)

x: (x-position in meters)

y: (y-position in meters)

z: (z-position in meters)

kittiDatasetObject.grabDepthFromPixel(int u, int v, float &x, float &y, float &z)

kittaDatasetObject.nextFrame();

kittaDatasetObject.jumpToFrame(int frame_number);