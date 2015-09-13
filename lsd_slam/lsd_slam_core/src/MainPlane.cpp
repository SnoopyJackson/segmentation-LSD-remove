#include "MainPlane.h"

/*
* Default Constructor.
*/
MainPlane::MainPlane() : _coefficients( new pcl::ModelCoefficients), _indices( new pcl::PointIndices) ,
    _planeCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), _hull(new pcl::PointCloud<pcl::PointXYZRGBA>)
{
}
/*
* Destructor.
*/
MainPlane::~MainPlane()
{

}
/*
* Clear the cloud and the hull.
*/
void MainPlane::clear()
{
    _planeCloud->clear();
    _hull->clear();
}

/*
* Set the coefficients of the planes (normals).
* params[in]: the coeffs
*/
void MainPlane::setCoefficients(pcl::ModelCoefficients::Ptr &coeff)
{
    _coefficients = coeff;
}
/*
* Set Indices.(usefull ??)
* params[in]: the indices
*/
void MainPlane::setIndices(pcl::PointIndices::Ptr &ind)
{
    _indices = ind;
}
/*
* Set the plane Cloud.
* params[in]: the cloud
*/
void MainPlane::setPlaneCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    _planeCloud->clear();
    //std::cout << "YOLO" << std::endl;
    _planeCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    _planeCloud = cloud;
}
/*
* Set the hull of the plane.
* params[in]: the hull
*/
void MainPlane::setHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &hull)
{
    _hull = hull;
}
/*
* Get the coefficients of the planee.
* return coefficients
*/
pcl::ModelCoefficients::Ptr MainPlane::getCoefficients() const
{
    return _coefficients;
}
/*
* get the indices of the plane.
* return indices
*/
pcl::PointIndices::Ptr MainPlane::getIndices() const
{
    return _indices;
}
/*
* Get the cloud.
* return cloud
*/
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  MainPlane::getPlaneCloud() const
{
    return _planeCloud;
}
/*
* Get the hull.
* return the hull
*/
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  MainPlane::getHull() const
{
    return _hull;
}
pcl::ModelCoefficients::Ptr MainPlane::getNormalizedCoefficients() const
{
    pcl::ModelCoefficients::Ptr normalizedCoeff = _coefficients;
    //normalizedCoeff  = _coefficients;
    double normalLenght = sqrt((_coefficients->values[0] * _coefficients->values[0]) + (_coefficients->values[1] * _coefficients->values[1]) + (_coefficients->values[2] * _coefficients->values[2]));
    normalizedCoeff->values[0] =  _coefficients->values[0] / normalLenght ;
    normalizedCoeff->values[1] = _coefficients->values[1] / normalLenght ;
    normalizedCoeff->values[2] = _coefficients->values[2] / normalLenght ;

    return normalizedCoeff;

}
void MainPlane::setVectorGround(Eigen::Vector3f &vector)
{
    double norm = std::sqrt((_vector[0]*_vector[0])+(_vector[1]*_vector[1])+(_vector[2]*_vector[2]));
    _vector[0] = vector[0]/norm;
    _vector[1] = vector[1]/norm;
    _vector[2] = vector[2]/norm;
}
Eigen::Vector3f MainPlane::getVector()
{
    return _vector;
}
