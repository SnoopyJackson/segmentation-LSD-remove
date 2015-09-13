#include "MainPlane.h"


MainPlane::MainPlane() : coefficients( new pcl::ModelCoefficients), indices( new pcl::PointIndices) ,
    planeCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), _hull(new pcl::PointCloud<pcl::PointXYZRGBA>)
{

}
MainPlane::~MainPlane()
{

}
void MainPlane::clear()
{
    planeCloud->clear();
    _hull->clear();
}
void MainPlane::setCoefficients(pcl::ModelCoefficients::Ptr &coeff)
{
    coefficients = coeff;
}
void MainPlane::setIndices(pcl::PointIndices::Ptr &ind)
{
    indices = ind;
}
void MainPlane::setPlaneCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    planeCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    planeCloud = cloud;
}
void MainPlane::setHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &hull)
{
    _hull = hull;
}
pcl::ModelCoefficients::Ptr MainPlane::getCoefficients() const
{
    return coefficients;
}
pcl::PointIndices::Ptr MainPlane::getIndices() const
{
    return indices;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  MainPlane::getPlaneCloud() const
{
    return planeCloud;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  MainPlane::getHull() const
{
    return _hull;
}
