#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>

#include <clams/discrete_depth_distortion_model.h>
#include <stream_sequence/pcl_typedefs.h>
#include <stream_sequence/frame_projector.h>
class Camera
{

public:
    Camera();
    ~Camera();
    int init();
    void run();
    bool isRunning();
    void getDepthData(cv::Mat &depthData);
    void getBGR(cv::Mat &bgr);
    void getPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
    void undistort(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_undistorted);
    void getPointCloudUndistorted(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
private:
    cv::Mat _depth;
    cv::Mat _bgr;
    cv::VideoCapture _sensor;
    bool _isRunning;
    float rgbFocalInvertedX;
    float rgbFocalInvertedY;

    clams::DiscreteDepthDistortionModel _distortionModel;
};
