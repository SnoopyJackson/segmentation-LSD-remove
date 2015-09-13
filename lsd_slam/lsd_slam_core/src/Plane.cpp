#include "Plane.h"

/*
* Default Constructor.
*/
Plane::Plane() : _coefficients( new pcl::ModelCoefficients), _indices( new pcl::PointIndices) ,
    _planeCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), _hull(new pcl::PointCloud<pcl::PointXYZRGBA>)
{

}
/*
* Destructor.
*/
Plane::~Plane()
{

}
/*
* Clear the plane cloud and the hull of the plane.
*/
void Plane::clear()
{
    _planeCloud->clear();
    _hull->clear();
}
/*
* Set the coefficients of the plane.
* params[in]: the coefficients
*/
void Plane::setCoefficients(pcl::ModelCoefficients::Ptr &coeff)
{
    _coefficients = coeff;
}
/*
* Set the indices of the plane.
* params[in]: the indices
*/
void Plane::setIndices(pcl::PointIndices::Ptr &ind)
{
    _indices = ind;
}
/*
* Set the cloud of the plane.
* params[in]: the cloud
*/
void Plane::setPlaneCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    _planeCloud = cloud;
}
/*
* Set the hull.
* params[in]: the hull
*/
void Plane::setHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &hull)
{
    _hull = hull;
}
/*
* Get the coefficients of the plane.
* return the coefficients
*/
pcl::ModelCoefficients::Ptr Plane::getCoefficients() const
{
    return _coefficients;
}
pcl::ModelCoefficients::Ptr Plane::getNormalizedCoefficients() const
{
    pcl::ModelCoefficients::Ptr normalizedCoeff = _coefficients;
    //normalizedCoeff  = _coefficients;
    double normalLenght = sqrt((_coefficients->values[0] * _coefficients->values[0]) + (_coefficients->values[1] * _coefficients->values[1]) + (_coefficients->values[2] * _coefficients->values[2]));
    normalizedCoeff->values[0] =  _coefficients->values[0] / normalLenght ;
    normalizedCoeff->values[1] = _coefficients->values[1] / normalLenght ;
    normalizedCoeff->values[2] = _coefficients->values[2] / normalLenght ;
    return normalizedCoeff;

}
/*
* Get the indices the plane.
* return the indices
*/
pcl::PointIndices::Ptr Plane::getIndices() const
{
    return _indices;
}
/*
* Get the cloud of the plane.
* return the cloud
*/
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  Plane::getPlaneCloud() const
{
    return _planeCloud;
}
/*
* Get the hull of the plane.
* return the hull
*/
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  Plane::getHull() const
{
    return _hull;
}
/*
* Concatenate Point cloud and recompute the hull.
* params[in]: the cloud
*/
void Plane::concatenateCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    *_hull += *cloud;
    // Recompute the hull
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeHull (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::ConcaveHull<pcl::PointXYZRGBA> phull;
    phull.setInputCloud(_hull);
    phull.setAlpha(2);
    phull.reconstruct(*_hull);
}
/*
* Set the Height of the plane.
* params[in]: the Height
*/
void Plane::setHeight(Eigen::Vector4f &height)
{
    _height = height;
}
/*
* Get the height of the plane.
* return the height
*/
Eigen::Vector4f Plane::getHeight()
{
    return _height;
}
/*
* Set the centroid of the plane.
* params[in]: the centroid
*/
void Plane::setCentroid(Eigen::Vector4f &centroid)
{
    _centroid = centroid;

}
void Plane::setCentroidTrans(Eigen::Vector4f &centroid)
{
    _centroidTrans[0] = 0.0;
    _centroidTrans[1] = 0.0;
    _centroidTrans[2] = 0.0;
    _centroidTrans = centroid;
}
/*
* Get the centroid of the plane.
* return the centroid
*/
Eigen::Vector4f Plane::getCentroid()
{
    return _centroid;
}
Eigen::Vector4f Plane::getCentroidTrans()
{
    return _centroidTrans;
}
void Plane::setType(TYPE type)
{
    _type = type;
}
TYPE Plane::getTYPE()
{
    return _type;
}
void Plane::setVectorGround(Eigen::Vector3f &vector)
{
    double norm = std::sqrt((_vector[0]*_vector[0])+(_vector[1]*_vector[1])+(_vector[2]*_vector[2]));
    _vector[0] = vector[0]/norm;
    _vector[1] = vector[1]/norm;
    _vector[2] = vector[2]/norm;
}
Eigen::Vector3f Plane::getVector()
{
    return _vector;
}
