#include "camera.h"

#include <iostream>
//#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
const double C_X = 319.5;
const double C_Y = 239.5;
const double F_X = 525.0;
const double F_Y = 525.0;
const int WIDTH = 640;
const int HEIGHT = 480;
const double FOCAL_INVERTED_X = 1/525.0;
const double FOCAL_INVERTED_Y = 1/525.0;
const std::string CLAMS_MODEL = "my_model";
Camera::Camera()
{
    _isRunning = true;
    rgbFocalInvertedX = FOCAL_INVERTED_X;
    rgbFocalInvertedY = FOCAL_INVERTED_Y;
    _distortionModel.load(CLAMS_MODEL);
}

Camera::~Camera(){}

int Camera::init()
{
    _sensor.open(CV_CAP_OPENNI_ASUS);
    //_sensor.open(CV_CAP_OPENNI);
    if( !_sensor.isOpened() )
    {
        std::cout << "Can not open capture object 1." << std::endl;
        return -1;
    }
    return 0;
}
bool Camera::isRunning()
{
    return _isRunning;
}

void Camera::getDepthData(cv::Mat &depthData)
{
    _depth.copyTo(depthData, _depth);
    //cv::copyTo(_depth, depthData);
}
void Camera::getBGR(cv::Mat &bgr)
{
    bgr.copyTo(bgr, _bgr);
    //cv::copyTo(_bgr, bgr);
}
void Camera::undistort(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_undistorted)
{
    cloud_undistorted.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    clams::FrameProjector proj;
    //depth.z = ;
    //std::cout << "cloud " << cloud->size() << std::endl;
    clams::Frame *frame = new clams::Frame ;
    proj.width_ = WIDTH;
    proj.height_ = HEIGHT;
    proj.cx_ = C_X;
    proj.cy_ = C_Y;
    proj.fx_ = F_X ;
    proj.fy_ = F_Y;
    clams::Cloud cloudToCalibrate;
    clams::Cloud *cloud_dist = new clams::Cloud;
    cloudToCalibrate.width = WIDTH;
    cloudToCalibrate.height = HEIGHT;
    //std::cout << "hi" << std:: endl;
    for(size_t a = 0; a < HEIGHT ; a++)
    {
        for(size_t b = 0; b < WIDTH ; b++)
        {
            clams::Point point;
            //std::cout << "a " << (int) a << " b " << (int) b << std::endl;
            point.x = cloud->points[a * WIDTH + b].x;
            point.y = cloud->points[a * WIDTH + b].y;
            point.z = cloud->points[a * WIDTH + b].z;
            point.r = cloud->points[a * WIDTH + b].r;
            point.g = cloud->points[a * WIDTH + b].g;
            point.b = cloud->points[a * WIDTH + b].b;
            cloudToCalibrate.push_back(point);

        }
    }
    //std::cout << "hi2" << std:: endl;
    //lams::IndexMap* indexmap;
    clams::FrameProjector::IndexMap *indexMap = new clams::FrameProjector::IndexMap;
    cloudToCalibrate.header.stamp = cloud->header.stamp;
    //std::cout << "hi3" << std:: endl;
    proj.cloudToFrame(cloudToCalibrate, frame, indexMap);

    _distortionModel.undistort(frame->depth_.get());


    proj.frameToCloud(*frame, cloud_dist);

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_undistorted (new pcl::PointCloud<pcl::PointXYZRGBA>);
    for(size_t c = 0; c < HEIGHT ; c++)
    {
        for(size_t d = 0; d < WIDTH ; d++)
        {
            pcl::PointXYZRGBA point;

            point.x = cloud_dist->points[c * WIDTH + d].x;
            point.y = cloud_dist->points[c * WIDTH + d].y;
            point.z = cloud_dist->points[c * WIDTH + d].z;
            point.r = cloud_dist->points[c * WIDTH + d].r;
            point.g = cloud_dist->points[c * WIDTH + d].g;
            point.b = cloud_dist->points[c * WIDTH + d].b;
            cloud_undistorted->push_back(point);
            // cloud_dist = *cloud[a];
            //_distortionModel.undistort(frame);
        }
    }
    delete indexMap;
    delete cloud_dist;
}
void Camera::getPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    cloud.reset(new  pcl::PointCloud<pcl::PointXYZRGBA>);
    for (unsigned int h = 0; h < _depth.rows; h++)
    {
        for (unsigned int w = 0; w < _depth.cols; w++)
        {
            //            point.z = 1024 * rand () / (RAND_MAX + 1.0f); // depthValue
            //            //point.z /= 1000;
            //            point.x = 1024 * rand () / (RAND_MAX + 1.0f) ;//(w - 319.5); //321.075 1024 * rand () / (RAND_MAX + 1.0f) //(w - 319.5) * point.z * rgbFocalInvertedX;
            //            point.y = 1024
            unsigned short depthValue;
            depthValue = _depth.at<unsigned short>(h,w);
            //cv::Vec3f pt3D = _bgr.at<Vec3f>(h, w);
            pcl::PointXYZRGBA point;
            point.z = depthValue; // depthValue
            point.z /= 1000;
            point.x = (w - C_X) * point.z * rgbFocalInvertedX ;//(w - 319.5); //321.075 1024 * rand () / (RAND_MAX + 1.0f) //(w - 319.5) * point.z * rgbFocalInvertedX;
            point.y = (h - C_Y) * point.z * rgbFocalInvertedY; //248.919//(h - 239.5) * point.z * rgbFocalInvertedY
            point.r = 255;
            point.g = 255;
            point.b = 255;
            cloud->push_back(point);
            //std::cout << " point x " << point.x << "  " << point.y << " " << point.z << std::endl;
        }

    }

}
void Camera::getPointCloudUndistorted(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_distorted(new  pcl::PointCloud<pcl::PointXYZRGBA>);
    getPointCloud(cloud_distorted);
    undistort(cloud_distorted,cloud);
}
void Camera::run()
{
    _isRunning = true;
    for(;;)
    {
        //cv::Mat depth;

        _sensor.grab();
        _sensor.set(CV_CAP_PROP_OPENNI_REGISTRATION,1);

        _sensor.retrieve( _depth, CV_CAP_OPENNI_DEPTH_MAP );
        _sensor.retrieve( _bgr, CV_CAP_OPENNI_BGR_IMAGE );
        cv::imshow("BGR ",_bgr);

        if( cv::waitKey( 30 ) == 27 )
        {
            _isRunning = false;
            std::cout << "camera stopped" << std::endl;
            break;//ESC to exit
        }

    }


}
