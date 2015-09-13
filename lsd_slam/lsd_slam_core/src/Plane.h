#ifndef PLANE_H_
#define PLANE_H_


// TODO Clean
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
enum TYPE
{
    HORIZONTAL = 0,
    VERTICAL = 1
};
class Plane
{

public:

    Plane();
    ~Plane();
    void clear();
    void setCoefficients(pcl::ModelCoefficients::Ptr &coeff);
    void setIndices(pcl::PointIndices::Ptr &ind);
    void setPlaneCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
    void setHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &hull);
    pcl::ModelCoefficients::Ptr getCoefficients() const;
    pcl::PointIndices::Ptr getIndices() const;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  getPlaneCloud() const;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  getHull() const;
    Eigen::Vector4f getHeight();
    void setHeight(Eigen::Vector4f &height);
    void concatenateCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
    void setCentroid(Eigen::Vector4f &centroid);
    void setCentroidTrans(Eigen::Vector4f &centroid);
    Eigen::Vector4f getCentroid();
    Eigen::Vector4f getCentroidTrans();
    pcl::ModelCoefficients::Ptr getNormalizedCoefficients() const;
    void setType(TYPE type);
    TYPE getTYPE();
    void setVectorGround(Eigen::Vector3f &vector);
    Eigen::Vector3f getVector();

private:
    Eigen::Vector3f _vector;
    pcl::ModelCoefficients::Ptr _coefficients;
    pcl::PointIndices::Ptr _indices;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _planeCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _hull;
    Eigen::Vector4f _height;
    Eigen::Vector4f _centroid;
    Eigen::Vector4f _centroidTrans;
    TYPE _type;
};
#endif
