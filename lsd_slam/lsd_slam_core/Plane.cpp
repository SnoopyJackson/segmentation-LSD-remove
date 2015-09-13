#include "Plane.h"


Plane::Plane() : coefficients( new pcl::ModelCoefficients), indices( new pcl::PointIndices) ,
    planeCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), _hull(new pcl::PointCloud<pcl::PointXYZRGBA>)
{

}
Plane::~Plane()
{

}
void Plane::clear()
{
    planeCloud->clear();
    _hull->clear();
}
void Plane::setCoefficients(pcl::ModelCoefficients::Ptr &coeff)
{
    coefficients = coeff;
}
void Plane::setIndices(pcl::PointIndices::Ptr &ind)
{
    indices = ind;
}
void Plane::setPlaneCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    planeCloud = cloud;
}
void Plane::setHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &hull)
{
    _hull = hull;
}
pcl::ModelCoefficients::Ptr Plane::getCoefficients() const
{
    return coefficients;
}
pcl::PointIndices::Ptr Plane::getIndices() const
{
    return indices;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  Plane::getPlaneCloud() const
{
    return planeCloud;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  Plane::getHull() const
{
    return _hull;
}
void Plane::concatenateCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    *planeCloud += *cloud;
//    pcl::ConcaveHull<pcl::PointXYZRGBA> chull;
//    chull.setInputCloud(planeCloud);
//    chull.setAlpha(0.1);
//    chull.reconstruct(*planeCloud);
}
void Plane::setHeight(Eigen::Vector4f &height)
{
    _height = height;
}
Eigen::Vector4f Plane::getHeight()
{
    return _height;
}
void Plane::setCentroid(Eigen::Vector4f &centroid)
{
    _centroid = centroid;

}
Eigen::Vector4f Plane::getCentroid()
{
    return _centroid;
}
