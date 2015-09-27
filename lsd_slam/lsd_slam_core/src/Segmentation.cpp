#include "Segmentation.h"

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>

std::vector<bool> isHorizontal;
/*
* Default Constructor.
*/
Segmentation::Segmentation():_axis(0,-std::sqrt(2)/2,-std::sqrt(2)/2)
{
    _coeffRansac = 0.15; //0.25
    isComputed = false;
    _planes.clear();
    _vectorNormalPlanes.clear();
    _mainCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    transformXY = Eigen::Matrix4f::Identity();
    transformTranslationOrigin = Eigen::Matrix4f::Identity();
    _vectorNormalPlanes.clear();
    _first = false;
    isHorizontal.clear();
}

/*
* Destructor.
*/
Segmentation::~Segmentation()
{}

/*
* Apply a passthrough filter.
* Params[in]: x_min, x_max, y_min, y_man, z_min, z_max, the input cloud
* Params[in/out]: the output cloud
* return true if the output cloud exist
*/
bool Segmentation::passthroughFilter(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &out_cloud)
{

    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max); //2
    pass.filter (*out_cloud);

    pass.setInputCloud (out_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_min, x_max); // 1.5
    //pass.setFilterLimitsNegative (true);
    pass.filter (*out_cloud);

    pass.setInputCloud (out_cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_min, y_max); // 0.5
    pass.filter (*out_cloud);

    if(out_cloud->size() > 0)
    {
        return true;
    }
    else
    {
        std::cerr << "The output cloud is empty !!!" << std::endl;
        return false;

    }

}

/*
* Find the ground in the environment.
* Params[in/out]: the cloud, the coeff of the planes, the indices, the ground cloud, the hull cloud
*/
bool Segmentation::findGround(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, MainPlane &mp)
{
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr hullGround(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE); // We search a plane perpendicular to the y
    seg.setMethodType (pcl::SAC_RANSAC);

    seg.setDistanceThreshold (0.030); //0.25 / 35

    seg.setAxis(_axis); // Axis Y 0, -1, 0
    seg.setEpsAngle(30.0f * (M_PI/180.0f)); // 50 degrees of error

    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    seg.setInputCloud (cloud);
    seg.segment (*indices, *coeff);
    mp.setCoefficients(coeff);
    //mp.setIndices(indices);

    if (indices->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model (Ground).");
        return false;
    }
    else
    {
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*ground);
        mp.setPlaneCloud(ground);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr concaveHull(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // Copy the points of the plane to a new cloud.
        pcl::ExtractIndices<pcl::PointXYZRGBA> extractHull;
        extractHull.setInputCloud(cloud);
        extractHull.setIndices(indices);
        extractHull.filter(*plane);

        // Compute the concave Hull TODO Convex ?
        pcl::ConcaveHull<pcl::PointXYZRGBA> chull;
        chull.setInputCloud (plane);
        chull.setAlpha (0.1);
        chull.reconstruct (*concaveHull);

        mp.setHull(concaveHull);
        //vectorCloudinliers.push_back(convexHull);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
        extract.filter(*cloud_f);
        cloud.swap(cloud_f);
        return true;
    }

}
/*
* Find the wall in the environment and remove it.
* Params[in/out]: the cloud
*/
void Segmentation::removeWall(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE); // We search a plane perpendicular to the y
    seg.setMethodType (pcl::SAC_RANSAC);

    seg.setDistanceThreshold (0.023); //0.25

    seg.setAxis(Eigen::Vector3f(0,-std::sqrt(2)/2,std::sqrt(2)/2)); // Axis Y 0, -1, 0
    seg.setEpsAngle(30.0f * (M_PI/180.0f)); // 50 degrees of error

    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    seg.setInputCloud (cloud);
    seg.segment (*indices, *coeff);

    if (indices->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model (Ground).");
    }
    else
    {
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
        extract.filter(*cloud_f);
        cloud.swap(cloud_f);
    }

}
/*
* Do an Euclidian Clustering.
* Params[in/out]: the cloud, the clusters
*/
void Segmentation::euclidianClustering( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  &cloud, std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &clusters)
{
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (300); //1000
    ec.setMaxClusterSize (100000000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }
}
void Segmentation::regionGrowing2( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  &cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  &outliers, std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &clusters)
{
    pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> > (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50); //10
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
    reg.setMinClusterSize (300);//1000 // 300 //30
    reg.setMaxClusterSize (100000000);

    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (50); // 50 //10
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (2.0 / 180.0 * M_PI); //4.0 / 6
    reg.setCurvatureThreshold (1.0); //4.0
    std::vector <pcl::PointIndices> clustersIndices;
    reg.extract (clustersIndices);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mc (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::copyPointCloud(*cloud,*mc);
    pcl::PointIndices::Ptr ind(new pcl::PointIndices);

    //pcl::PointCloud <pcl::PointXYZRGBA>::Ptr cloud_cluster;
    for (std::vector<pcl::PointIndices>::const_iterator it = clustersIndices.begin (); it != clustersIndices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud->points[*pit]);
            ind->indices.push_back(*pit);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);

        //        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        //        extract.setInputCloud(mc);
        //        extract.setIndices();
        //        extract.setNegative(true);
        //        //extract.setNegative(false);
        //        extract.filter(*mc);
        //        i++;
    }
    //pcl::PointIndices::Ptr ind(new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(cloud);

    extract.setIndices(ind);
    extract.setNegative(true);
    extract.filter(*mc);

    //std::cout << "size " << mc->points.size() << std::endl;
    //_mainCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //_mainCloud->clear();
    //*_mainCloud = *mc;
    *outliers = *mc;
}
/*
* Compute the error between the normal of a plane and the normal of the ground.
* Params[in/out]: the normal of a plane
* return true if it is < than the limit
*/
bool Segmentation::normalDifferenceError(pcl::ModelCoefficients::Ptr &normal, double error)
{
    //    if(abs(normal->values[0] - _ground.getCoefficients()->values[0]) < 0.04 &&
    //            abs(normal->values[1] - _ground.getCoefficients()->values[1]) < 0.1 &&
    //            abs(normal->values[2] - _ground.getCoefficients()->values[2]) < 0.1 )
    //    {
    //        return true;
    //    }
    //    return false;

    double groundLenght = sqrt((_ground.getCoefficients()->values[0] * _ground.getCoefficients()->values[0]) + (_ground.getCoefficients()->values[1] * _ground.getCoefficients()->values[1]) + (_ground.getCoefficients()->values[2] * _ground.getCoefficients()->values[2]));
    double grX= _ground.getCoefficients()->values[0] / groundLenght ;
    double grY = _ground.getCoefficients()->values[1] / groundLenght ;
    double grZ = _ground.getCoefficients()->values[2] / groundLenght ;
    double normalLenght = sqrt((normal->values[0] * normal->values[0]) + (normal->values[1] * normal->values[1]) + (normal->values[2] * normal->values[2]));
    double nX=  normal->values[0] / normalLenght ;
    double nY = normal->values[1] / normalLenght ;
    double nZ = normal->values[2] / normalLenght ;
    double scalar = (grX*nX) + (grY*nY) + (grZ*nZ);
    //std::cout << "The scalar is " << scalar << std::endl;

    if (std::abs(scalar) > error )
    {
        //std::cout << "true" << std::endl;
        return true;
    }
    return false;
    //    else
    //    {
    //        return false;
    //    }
}
bool Segmentation::normalDifferenceError2(pcl::ModelCoefficients::Ptr &normal, double error)
{
    //    if(abs(normal->values[0] - _ground.getCoefficients()->values[0]) < 0.04 &&
    //            abs(normal->values[1] - _ground.getCoefficients()->values[1]) < 0.1 &&
    //            abs(normal->values[2] - _ground.getCoefficients()->values[2]) < 0.1 )
    //    {
    //        return true;
    //    }
    //    return false;

    double groundLenght = sqrt((_ground.getCoefficients()->values[0] * _ground.getCoefficients()->values[0]) + (_ground.getCoefficients()->values[1] * _ground.getCoefficients()->values[1]) + (_ground.getCoefficients()->values[2] * _ground.getCoefficients()->values[2]));
    double grX= _ground.getCoefficients()->values[0] / groundLenght ;
    double grY = _ground.getCoefficients()->values[1] / groundLenght ;
    double grZ = _ground.getCoefficients()->values[2] / groundLenght ;
    double normalLenght = sqrt((normal->values[0] * normal->values[0]) + (normal->values[1] * normal->values[1]) + (normal->values[2] * normal->values[2]));
    double nX=  normal->values[0] / normalLenght ;
    double nY = normal->values[1] / normalLenght ;
    double nZ = normal->values[2] / normalLenght ;
    double scalar = (grX*nX) + (grY*nY) + (grZ*nZ);
    //std::cout << "The scalar is " << scalar << std::endl;

    if (std::abs(scalar) < error )
    {
        //std::cout << "true" << std::endl;
        return true;
    }
    return false;
    //    else
    //    {
    //        return false;
    //    }
}
/*
* Apply Ransac for each cluster.
* Params[in/out]: the clusters, the axis, the inliers
*/
void Segmentation::ransac(std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &clusters, Eigen::Vector3f axis, std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &vectorCloudInliers, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  &outliers )
{
    vectorCloudInliers.clear();
    isHorizontal.clear();
    for(unsigned int i = 0; i < clusters.size(); i++)
    {
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);

        pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg;
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Estimate Normals
        ne.setSearchMethod(tree);
        ne.setInputCloud(clusters[i]);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);

        // Create the segmentation object
        //pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
        // Optional
        seg.setOptimizeCoefficients (true);

        // Mandatory
        //seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE); // We search a plane perpendicular to the y
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight(0.005);
        seg.setMethodType (pcl::SAC_RANSAC);

        seg.setDistanceThreshold (0.020); //0.25 / 35

        //seg.setAxis(axis); // Axis Y 0, -1, 0

        seg.setEpsAngle(20.0f * (M_PI/180.0f)); // 50 degrees of error

        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        seg.setInputCloud (clusters[i]);
        seg.setInputNormals(cloud_normals);
        seg.segment (*indices, *coeff);


        if(normalDifferenceError(coeff, 0.8))
        {
            if (indices->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model (Ground).");
                //return false;
            }
            else
            {
                // std::cout << "coeff" << coeff->values[0] << "  " << coeff->values[1] << "   " << coeff->values[2] << "    "<< coeff->values[3] <<  std::endl;

                //                std::cout << "YOLOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << std::endl;
                extract.setInputCloud(clusters[i]);
                extract.setIndices(indices);
                extract.setNegative(true);
                //extract.setNegative(false);
                extract.filter(*cloud_);
                *outliers +=*cloud_;
                // project points on the plane
                pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
                proj.setModelType(pcl::SACMODEL_PLANE);
                proj.setIndices(indices);
                proj.setInputCloud(clusters[i]);
                proj.setModelCoefficients(coeff);
                proj.filter(*cloud_p);
                //pcl::removeNaNFromPointCloud(*cloud_p,*cloud_p);
                vectorCloudInliers.push_back(cloud_p);
                _vectorNormalPlanes.push_back(coeff);
                isHorizontal.push_back(true);
                //return true;
            }
        }
        //        else if(normalDifferenceError2(coeff, 0.3))
        //        {
        //            if (indices->indices.size () == 0)
        //            {
        //                PCL_ERROR ("Could not estimate a planar model (Ground).");
        //                //return false;
        //            }
        //            else
        //            {
        //                // std::cout << "coeff" << coeff->values[0] << "  " << coeff->values[1] << "   " << coeff->values[2] << "    "<< coeff->values[3] <<  std::endl;

        //                //                std::cout << "YOLOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << std::endl;
        //                extract.setInputCloud(clusters[i]);
        //                extract.setIndices(indices);
        //                extract.setNegative(true);
        //                //extract.setNegative(false);
        //                extract.filter(*cloud_);
        //                *outliers +=*cloud_;
        //                // project points on the plane
        //                pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
        //                proj.setModelType(pcl::SACMODEL_PLANE);
        //                proj.setIndices(indices);
        //                proj.setInputCloud(clusters[i]);
        //                proj.setModelCoefficients(coeff);
        //                proj.filter(*cloud_p);
        //                //pcl::removeNaNFromPointCloud(*cloud_p,*cloud_p);
        //                vectorCloudInliers.push_back(cloud_p);
        //                _vectorNormalPlanes.push_back(coeff);
        //                isHorizontal.push_back(false);
        //                //return true;
        //            }
        //        }
        //        else
        //        {
        //            *outliers += *clusters[i];
        //        }
    }
}
/*
* Find the other planes in the environment.
* Params[in/out]: the cloud, the normal of the ground, the coeff of the planes, the planes, the hull
* return the number of inliers
*/
int  Segmentation::findOtherPlanesRansac(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,Eigen::Vector3f axis, std::vector <pcl::ModelCoefficients> &coeffPlanes, std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &vectorCloudInliers, std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &vectorHull )
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients (true);

    vectorCloudInliers.clear();
    coeffPlanes.clear();
    vectorHull.clear();

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);

    const int nb_points = cloud->points.size();
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    seg.setMaxIterations(5);
    seg.setDistanceThreshold (0.020); //0.15
    seg.setAxis(axis);
    //std::cout<< "axis are  a : " << axis[0] << " b : " << axis[1] << " c ; " << axis[2] << std::endl;

    seg.setEpsAngle(20.0f * (M_PI/180.0f));
    while (cloud->points.size() > _coeffRansac * nb_points)
    {
        // std::cout << "the number is " << cloud->points.size() << std::endl;
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            //PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            break;
        }
        else
        {
            coeffPlanes.push_back(*coefficients);
            //vectorInliers.push_back(*inliers);
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGBA>);
            extract.filter(*cloud_p);

            //Eigen::Vector4f centroid;
            //pcl::compute3DCentroid(*cloud_p,*inliers,centroid);
            // passthroughFilter(centroid[0] - 2, centroid[0] + 2, centroid[1] - 2, centroid[1] + 2, centroid[2] - 2, centroid[2] + 2, cloud_p, cloud_p);


            //statisticalRemovalOutliers(cloud_p);
            //statisticalRemovalOutliers(cloud_p); -0.00485527 b : -0.895779 c ; -0.444474
            //if((std::abs(coefficients->values[0]) < (0.1 ))  && ( std::abs(coefficients->values[1]) > (0.89)) && (std::abs(coefficients->values[2]) > (0.40)))
            //{
            //  std::cout<< "coeff are  a : " << coefficients->values[0] << " b : " << coefficients->values[1] << " c ; " << coefficients->values[2] << std::endl;

            vectorCloudInliers.push_back(cloud_p);
            //}

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr concaveHull(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
            // Copy the points of the plane to a new cloud.
            pcl::ExtractIndices<pcl::PointXYZRGBA> extractHull;
            extractHull.setInputCloud(cloud);
            extractHull.setIndices(inliers);
            extractHull.filter(*plane);

            // Object for retrieving the convex hull.
            //                pcl::ConvexHull<pcl::PointXYZRGBA> hull;
            //                hull.setInputCloud(plane);
            //                hull.reconstruct(*convexHull);

            pcl::ConcaveHull<pcl::PointXYZRGBA> chull;
            //            chull.setInputCloud (plane);
            //            chull.setAlpha (0.1);
            //            chull.reconstruct (*concaveHull);
            //            vectorHull.push_back(concaveHull);

            //vectorCloudinliers.push_back(convexHull);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
            extract.filter(*cloud_f);
            cloud.swap(cloud_f);
            //  std::cout << "the number is " << cloud->points.size() << std::endl;
        }

    }

    return vectorCloudInliers.size();
}

/*
* Apply a the region growing algorithm.
* Params[in/out]: the inliers detected by Ransac, the output clusters found by RG
* return the number of clusters
*/
int Segmentation::regionGrowing(std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &inliers, std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &clusters)
{

    //    //      REGION GROWING
    //    if (inliers.size() > 0)
    //    {
    //        for(unsigned int i = 0; i < inliers.size(); i++)
    //        {

    //            pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> > (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    //            pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    //            pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimator;
    //            normal_estimator.setSearchMethod (tree);
    //            normal_estimator.setInputCloud (inliers[i]);
    //            normal_estimator.setKSearch (50);
    //            normal_estimator.compute (*normals);

    //            pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
    //            reg.setMinClusterSize (1000);//1000
    //            reg.setMaxClusterSize (100000000);

    //            reg.setSearchMethod (tree);
    //            reg.setNumberOfNeighbours (30);
    //            reg.setInputCloud (inliers[i]);
    //            //reg.setIndices (indices);
    //            reg.setInputNormals (normals);
    //            reg.setSmoothnessThreshold (7.0 / 180.0 * M_PI); //4.0 / 6
    //            reg.setCurvatureThreshold (4.0); //4.0
    //            std::vector <pcl::PointIndices> clustersIndices;
    //            reg.extract (clustersIndices);
    //            //pcl::PointCloud <pcl::PointXYZRGBA>::Ptr cloud_cluster;
    //            for (std::vector<pcl::PointIndices>::const_iterator it = clustersIndices.begin (); it != clustersIndices.end (); ++it)
    //            {

    //                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    //                {
    //                    cloud_cluster->points.push_back (inliers[i]->points[*pit]);
    //                }
    //                cloud_cluster->width = cloud_cluster->points.size ();
    //                cloud_cluster->height = 1;
    //                cloud_cluster->is_dense = true;
    //                clusters.push_back(cloud_cluster);

    //            }
    //        }
    //        return clusters.size();
    //    }
    //    else
    //    {
    //        std::cerr << "No planes detected" << std::endl;
    //        return -1;
    //    }

    //      REGION GROWING
    if (inliers.size() > 0)
    {
        for(unsigned int i = 0; i < inliers.size(); i++)
        {
            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
            tree->setInputCloud (inliers[i]);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
            ec.setClusterTolerance (0.02); // 2cm
            ec.setMinClusterSize (1000);
            ec.setMaxClusterSize (100000000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (inliers[i]);
            ec.extract (cluster_indices);
            //            pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> > (new pcl::search::KdTree<pcl::PointXYZRGBA>);
            //            pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
            //            pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimator;
            //            normal_estimator.setSearchMethod (tree);
            //            normal_estimator.setInputCloud (inliers[i]);
            //            normal_estimator.setKSearch (50);
            //            normal_estimator.compute (*normals);

            //            pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
            //            reg.setMinClusterSize (1000);//1000
            //            reg.setMaxClusterSize (100000000);

            //            reg.setSearchMethod (tree);
            //            reg.setNumberOfNeighbours (30);
            //            reg.setInputCloud (inliers[i]);
            //            //reg.setIndices (indices);
            //            reg.setInputNormals (normals);
            //            reg.setSmoothnessThreshold (7.0 / 180.0 * M_PI); //4.0 / 6
            //            reg.setCurvatureThreshold (4.0); //4.0

            //            std::vector <pcl::PointIndices> clustersIndices;
            //            reg.extract (clustersIndices);
            //            //pcl::PointCloud <pcl::PointXYZRGBA>::Ptr cloud_cluster;
            //            for (std::vector<pcl::PointIndices>::const_iterator it = clustersIndices.begin (); it != clustersIndices.end (); ++it)
            //            {
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    cloud_cluster->points.push_back (inliers[i]->points[*pit]);
                }
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                clusters.push_back(cloud_cluster);
            }
            //                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
            //                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            //                {
            //                    cloud_cluster->points.push_back (inliers[i]->points[*pit]);
            //                }
            //                cloud_cluster->width = cloud_cluster->points.size ();
            //                cloud_cluster->height = 1;
            //                cloud_cluster->is_dense = true;
            //                clusters.push_back(cloud_cluster);

            //            }
        }
        return clusters.size();
    }
    else
    {
        std::cerr << "No planes detected" << std::endl;
        return -1;
    }
}

/*
*Compute the Transformation to the Plane XY (usefull to calculate the height).
*
*/
void Segmentation::computeTransformation()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    gr = _ground.getPlaneCloud();
    pcl::PointXYZ normalXY;
    pcl::PointXYZ normalGround;
    pcl::PointXYZ result;
    normalXY.x = 0.0;
    normalXY.y = 0.0;
    normalXY.z = 1.0;

    double normalLength = sqrt((_ground.getCoefficients()->values[0] * _ground.getCoefficients()->values[0]) + (_ground.getCoefficients()->values[1] * _ground.getCoefficients()->values[1]) + (_ground.getCoefficients()->values[2] * _ground.getCoefficients()->values[2]));
    normalGround.x = _ground.getCoefficients()->values[0] / normalLength ;
    normalGround.y = _ground.getCoefficients()->values[1] / normalLength ;
    normalGround.z = _ground.getCoefficients()->values[2] / normalLength ;

    result.x = normalGround.y * normalXY.z - normalXY.y*normalGround.z;
    result.y = normalXY.x*normalGround.z - normalGround.x*normalXY.z;
    result.z = normalGround.x*normalXY.y - normalGround.y*normalXY.x;
    double resultLenght = sqrt((result.x * result.x) + (result.y * result.y) +(result.z * result.z));
    result.x /=resultLenght;
    result.y /=resultLenght;
    result.z /=resultLenght;

    double theta = std::acos(normalGround.z/sqrt((normalGround.x* normalGround.x)+(normalGround.y*normalGround.y)+(normalGround.z*normalGround.z)));
    //std::cout << "The crossproduct is " << result.x << " "<< result.y<< " "<< result.z <<std::endl;
    std::cout << "norm " << std::sqrt((result.x * result.x  )+(result.y * result.y)+ (result.z * result.z))<< std::endl;

    double xx = (result.x * result.x) + ((1 - (result.x * result.x)) * std::cos(theta));
    double xy = (result.x * result.y)* (1 - std::cos(theta)) - (result.z * std::sin(theta));
    double xz = (result.x * result.z)* (1 - std::cos(theta)) + (result.y * std::sin(theta));
    double yx = (result.x * result.y)* (1 - std::cos(theta)) + (result.z * std::sin(theta));
    double yy = (result.y * result.y) + ((1 - (result.y * result.y)) * std::cos(theta));
    double yz = (result.y * result.z)* (1 - std::cos(theta)) - (result.x * std::sin(theta));
    double zx = (result.x * result.z)* (1 - std::cos(theta)) - (result.y * std::sin(theta));
    double zy = (result.y * result.z)* (1 - std::cos(theta)) + (result.x * std::sin(theta));
    double zz = (result.z * result.z) + ((1 - (result.z * result.z)) * std::cos(theta));

    transformXY = Eigen::Matrix4f::Identity();

    //    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    //    //float theta = M_PI/4; // The angle of rotation in radians

    transformXY (0,0) = xx; //cos (theta);
    transformXY (0,1) = xy;// -sin(theta)
    transformXY (0,2) = xz;
    transformXY (1,0) = yx; // sin (theta)
    transformXY (1,1) = yy;
    transformXY (1,2) = yz;
    transformXY (2,0) = zx;
    transformXY (2,1) = zy;
    transformXY (2,2) = zz;

    pcl::transformPointCloud(*gr, *gr, transformXY);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*gr, centroid);
//    transformTranslationOrigin(0,3) = - centroid[0];
//    transformTranslationOrigin(1,3) = - centroid[1];
//    transformTranslationOrigin(2,3) = - centroid[2];
}
/*
* Compute the height of each plane.
*
*/
void Segmentation::transformOrigin()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    ground = _ground.getHull();
    pcl::transformPointCloud(*ground, *gr, transformXY);
    pcl::transformPointCloud(*gr, *gr, transformTranslationOrigin);
    _ground.setHull(gr);
    Eigen::Vector3f normal_ground;
    normal_ground[0] = _ground.getNormalizedCoefficients()->values[0];
    normal_ground[1] = _ground.getNormalizedCoefficients()->values[1];
    normal_ground[2] = _ground.getNormalizedCoefficients()->values[2];

    //rotateVector
    normal_ground = transformXY.block(0,0,3,3) * normal_ground;
    std::cout << "normal ground origin" << normal_ground[0] << " " << normal_ground[1]<< " " << normal_ground[2]<< std::endl;

    //normal_ground[0] +=transformTranslationOrigin(0,3);
    //normal_ground[1] +=transformTranslationOrigin(1,3);
    //normal_ground[2] +=transformTranslationOrigin(2,3);
    //setNormalizedVector TODO
    _ground.setVectorGround(normal_ground);
    Eigen::Vector4f centroid_ground;
    pcl::compute3DCentroid (*gr, centroid_ground);
    for(size_t i = 0; i < _planes.size();i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
        Eigen::Vector4f centroidPlane;
        Eigen::Vector4f centroidPlaneT;

        hull = _planes[i].getHull();
        pcl::transformPointCloud(*hull, *plane, transformXY);
        pcl::transformPointCloud(*plane, *plane, transformTranslationOrigin);
        _planes[i].setHull(plane);
        Eigen::Vector3f normal_plane;
        normal_plane[0] = _planes[i].getNormalizedCoefficients()->values[0];
        normal_plane[1] = _planes[i].getNormalizedCoefficients()->values[1];
        normal_plane[2] = _planes[i].getNormalizedCoefficients()->values[2];
        //rotateVector
        normal_plane = transformXY.block(0,0,3,3) * normal_plane;
        normal_plane[0] +=transformTranslationOrigin(0,3);
        normal_plane[1] +=transformTranslationOrigin(1,3);
        normal_plane[2] +=transformTranslationOrigin(2,3);
        _planes[i].setVectorGround(normal_plane);
        pcl::compute3DCentroid (*plane, centroidPlane);
        pcl::compute3DCentroid (*plane, centroidPlaneT);

        Eigen::Vector4f height = centroidPlaneT - centroid_ground;
        _planes[i].setHeight(height);
        Eigen::Vector4f height_out = _planes[i].getHeight();
        _planes[i].setCentroid(centroidPlane);
    }

}
void Segmentation::computeHeight()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gr_hull(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    gr = _ground.getPlaneCloud();
    pcl::transformPointCloud(*gr, *out_cloud, transformXY);

    //gr_hull = _ground.getHull();
    //pcl::transformPointCloud(*gr_hull, *gr_hull, transformXY);
    //Eigen::Matrix4f transformTranslation = Eigen::Matrix4f::Identity();


    //_ground.setPlaneCloud(out_cloud);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*out_cloud, centroid);
    //transformTranslation(0,3) = - centroid[0];
    //transformTranslation(1,3) = - centroid[1];
    //transformTranslation(2,3) = - centroid[2];

    //pcl::transformPointCloud(*gr_hull, *gr_hull, transformTranslation);
    //_ground.setHull(gr_hull);
    for(size_t i = 0; i < _planes.size();i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane_out(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr hull_out(new pcl::PointCloud<pcl::PointXYZRGBA>);


        plane = _planes[i].getPlaneCloud();
        //hull = _planes[i].getHull();
        pcl::transformPointCloud(*plane, *plane_out, transformXY);
        //pcl::transformPointCloud(*hull, *hull_out, transformXY);

        //_planes[i].setPlaneCloud(plane_out);
        //_planes[i].setHull(hull_out);

        Eigen::Vector4f centroidPlane;
        Eigen::Vector4f centroidPlaneT;
        pcl::compute3DCentroid (*plane, centroidPlane);
        pcl::compute3DCentroid (*plane_out, centroidPlaneT);

        Eigen::Vector4f height = centroidPlaneT - centroid;
        _planes[i].setHeight(height);
        Eigen::Vector4f height_out = _planes[i].getHeight();
        _planes[i].setCentroid(centroidPlane);
        //std::cout << "centroid:" << centroidPlane[0] << " " << centroidPlane[1] << " " <<  centroidPlane[2] << " " <<centroidPlane[3] << " \n";
        //std::cout << "height:" << height[0] << " " << height[1] << " " <<  height[2] << " " <<height[3] << " \n";
        //    pcl::PointXYZ c;

    }


}


/*
* Create planes object depending on the number of clusters.
* Params[in/out]: the vector of planes
*/
void Segmentation::createPlane(std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &clusters)
{
    _planes.clear();

    for (int v = 0 ; v < clusters.size(); v++)
    {
        Plane plane;
        //pcl::transformPointCloud(*clusters[v], *clusters[v], transformXY);
        plane.setPlaneCloud(clusters[v]);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr concaveHull(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::ConcaveHull<pcl::PointXYZRGBA> chull;
        chull.setInputCloud (clusters[v]);
        chull.setAlpha (0.1);
        chull.reconstruct (*concaveHull);
        //vectorHull.push_back(concaveHull);
        plane.setHull(concaveHull);
        if(v < isHorizontal.size())
        {
            if(isHorizontal[v] == true)
            {
                //std::cout << "horizontal" << std::endl;
                plane.setType(HORIZONTAL);
            }
            else
            {
                //std::cout << "vertical" << std::endl;

                plane.setType(VERTICAL);
            }
        }
        if(v < _vectorNormalPlanes.size())
        {
            plane.setCoefficients(_vectorNormalPlanes[v]);
        }
        _planes.push_back(plane);

    }
    if(!_first)
    {   // compute Transformtation plan XY
        computeTransformation();
        _first = true;
    }
    // Compute the height of planes
    //computeHeight();
    transformOrigin();
}

/*
* Get the planes.
* params[out]: the vector of planes
*/
void Segmentation::getPlanes(std::vector<Plane> &planes)
{
    planes.clear();
    for(unsigned int i = 0 ;  i < _planes.size() ; i++ )
    {
        planes.push_back(_planes[i]);
    }
}

/*
* check if it is computed.
* return the value of isComputed;
*/
bool Segmentation::getIsComputed() const
{
    return isComputed;
}

/*
* Apply a statistical Removal Outlier.
* Params[in/out]: the cloud
*/
void Segmentation::statisticalRemovalOutliers(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (100); //200
    sor.setStddevMulThresh (1); // m0.5
    sor.filter (*cloud);

}
/*
* Get the ground.
* return the ground;
*/
MainPlane Segmentation::getMainPlane() const
{
    return _ground;
}
/*
* Get the remaining main cloud.
* params[out]: the remaining main cloud
*/
void Segmentation::getMainCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    _mainCloud->swap(*cloud);
}
/*
* Do the segmentation.
* params[in]: the cloud to segmentate
*/
void Segmentation::doSegmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    //MainPlane groundMainPlane;
    // Variables
    isComputed = false ;
    struct timeval tbegin,tend;
    double texec=0.0;

    // Start timer
    gettimeofday(&tbegin,NULL);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFilteredCopy(new pcl::PointCloud<pcl::PointXYZRGBA>);
    _mainCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    (void) passthroughFilter(-5.0, 5.0, -5.0, 5.0, -5.0, 5.0, cloud, cloudFiltered); // 3.0, 3.0, -0.2, 2.0, -4.0, 4.0
    pcl::copyPointCloud(*cloudFiltered, *_mainCloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Hullground (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mainground2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

    std::vector<pcl::PointIndices> vectorInliers(0);
    std::vector <pcl::ModelCoefficients> vectorCoeff(0);
    std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > vectorCloudinliers(0);
    std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > vectorHull(0);
    std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > vectorCluster(0);

    pcl::ModelCoefficients::Ptr coefficientsGround (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliersGround (new pcl::PointIndices);

    //    pcl::VoxelGrid< pcl::PointXYZRGBA > sor;
    //    sor.setInputCloud (_mainCloud);
    //    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    //    sor.filter (*_mainCloud);
    //(void) findGround(_mainCloud,_ground);
    //isComputed = true;

    // Find the ground
    if(findGround(_mainCloud,_ground))
    {
        // Normal of the ground
        Eigen::Vector3f gravityVector(_ground.getCoefficients()->values[0], _ground.getCoefficients()->values[1], _ground.getCoefficients()->values[2]);

        //(void) findOtherPlanesRansac(_mainCloud,gravityVector,vectorCoeff , vectorCloudinliers,vectorHull);

        //(void) regionGrowing(vectorCloudinliers, vectorCluster);
        //Euclidian Clustering
        regionGrowing2(_mainCloud,mainground2,vectorCloudinliers);
        //euclidianClustering(_mainCloud,vectorCloudinliers);
        std::cout << "number of cluster " << vectorCloudinliers.size() << std::endl;
        // Ransac
        _mainCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        _mainCloud->clear();

        *_mainCloud= * mainground2;

        ransac(vectorCloudinliers,gravityVector,vectorCluster,_mainCloud);
        //pcl::transformPointCloud(*_mainCloud,*_mainCloud,transformXY);
        //pcl::transformPointCloud(*_mainCloud,*_mainCloud,transformTranslationOrigin);
        // Create planes
        createPlane(vectorCluster);
        // End timer
        pcl::transformPointCloud(*cloud,*cloud,transformXY);
        pcl::transformPointCloud(*cloud,*cloud,transformTranslationOrigin);


    }

    gettimeofday(&tend,NULL);
    // Compute execution time
    texec = ((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000;
    std::cout << "time Segmentation : " << texec << std::endl;
}
