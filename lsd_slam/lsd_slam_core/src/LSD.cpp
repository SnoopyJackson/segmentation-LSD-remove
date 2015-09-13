/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <pcl/registration/icp.h>

#include <boost/thread.hpp>

#include "util/settings.h"
#include "util/Parse.h"
#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "util/Undistorter.h"
#include "util/RawLogReader.h"
#include "util/Resolution.h"
#include "util/Intrinsics.h"

#include "DataStructures/Frame.h"
#include "SlamSystem.h"
#include "osg_gui.h"
#include "Helper.h"
#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>
#include <map>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <ctype.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "IOWrapper/ImageDisplay.h"

#include <X11/Xlib.h>

#include <boost/thread/thread.hpp>
#include <iostream>
#include "camera.h"
#include "Segmentation.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Texture2D>
#include <osgUtil/ShaderGen>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Math>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
//
//#include "Plane.h"
using namespace cv;
using namespace std;
using namespace lsd_slam;


std::vector<std::string> files;
int w, h, w_inp, h_inp;
ThreadMutexObject<bool> lsdDone(false);
RawLogReader * logReader = 0;
int numFrames = 0;

Sophus::Sim3f camPoseCurrent;
Sophus::Sim3f camPoseSaved;
SE3 poseCu;
SE3 poseSaved;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_cloud;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr prev_cloud;
pcl::PointCloud<pcl::PointNormal>::Ptr  prev_cloud2;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _ground;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _mc;
std::vector<Plane> _planes_cloud;
std::vector<Plane> _prev_planes;

bool stopTracking = false;
bool planeDetected = false;
bool isFirstFrame = true;
cv::Mat _depth;
Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
Eigen::Matrix4f _transformationICP = Eigen::Matrix4f::Identity();
Eigen::Matrix4f _transformationXYPlane = Eigen::Matrix4f::Identity();
Eigen::Matrix4f _transformationXYTranslation = Eigen::Matrix4f::Identity();
int cpt= 0;
const float leaf = 0.015f; //0.015 // 0.050
void registerCloudWithNormals2(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &ground,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &mc, std::vector<Plane> &pl)
{
    // create cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_transf(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr frameIn1(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  cloud2 (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  cloud_transf2 (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  final2 (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  frameIn1_ (new pcl::PointCloud<pcl::PointNormal>);

    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        pcl::PointNormal point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        cloud2->push_back(point);

    }
    // transfrom camera pose
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();


    transform_1 (0,0) = camPoseSaved.rotationMatrix()(0,0); //cos (theta);
    transform_1 (0,1) = camPoseSaved.rotationMatrix()(0,1);// -sin(theta)
    transform_1 (0,2) = camPoseSaved.rotationMatrix()(0,2);
    transform_1 (1,0) = camPoseSaved.rotationMatrix()(1,0); // sin (theta)
    transform_1 (1,1) = camPoseSaved.rotationMatrix()(1,1); //cos (theta)
    transform_1 (1,2) = camPoseSaved.rotationMatrix()(1,2);
    transform_1 (2,0) = camPoseSaved.rotationMatrix()(2,0) ;
    transform_1 (2,1) = camPoseSaved.rotationMatrix()(2,1);
    transform_1 (2,2) = camPoseSaved.rotationMatrix()(2,2);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform_1 (0,3) = camPoseSaved.translation()[0]; //2.5
    transform_1 (1,3) = camPoseSaved.translation()[1]; //2.5
    transform_1 (2,3) = camPoseSaved.translation()[2]; //2.5
    //std::cout << " Translation : " << savedTranslation[0]<< "  " << savedTranslation[1]<< "  "<< savedTranslation[2] << std::endl;
    //computeRotationMatrix(transform_1);
    pcl::transformPointCloud (*cloud2, *cloud_transf2, transform_1);


    //std::stringstream ss;
    //ss << cpt;
    //pcl::io::savePCDFileASCII (ss.str() + ".pcd", *out_cloud);
    //cpt++;

    // if prev_cloud empty fill it with current frame
    if(prev_cloud2->points.size() == 0)
    {
        *prev_cloud2 = *cloud_transf2;
        //*source_cloud = *cloud_transf;
        std::cout << "initialisation" << std::endl;
    }
    // pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    // ICP
    //pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    //pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    //pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
    //typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> PointToPlane;
    //boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
    //icp.setTransformationEstimation(point_to_plane);

    pcl::NormalEstimationOMP<pcl::PointNormal,pcl::PointNormal> nest;
    nest.setRadiusSearch (leaf+0.05);
    nest.setInputCloud (prev_cloud2);
    nest.compute (*prev_cloud2);
    nest.setInputCloud (cloud_transf2);
    nest.compute (*cloud_transf2);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setInputSource(cloud_transf2);
    icp.setInputTarget(prev_cloud2);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (leaf * 1.5);//0.1 0.25
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (500);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);//1e-5
    //icp.setTransformationEpsilon (1e-12);//1e-5
    // Set the euclidean distance difference epsregisterCloudilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1e-8); //1e-5, 1e-8
    // icp.setUseReciprocalCorrespondences(true);
    //icp.setTransformationEpsilon(0);
    icp.setRANSACOutlierRejectionThreshold(1.5f * leaf);
    //    icp.setMaxCorrespondenceDistance (0.1 ); // 0.25, 0.10
    //    icp.setMaximumIterations(5000); // 10, 100 /// 50
    //    //icp.setTransformationEpsilon (1-5); //1-8,  //1-5
    //    //    // Set the euclidean distance difference epsilon (criterion 3)
    //    icp.setEuclideanFitnessEpsilon (0.01);//1-5, 0.001 //0.01
    //    icp.setTransformationEpsilon(1-5);
    //    icp.setRANSACOutlierRejectionThreshold(1.5f * 0.015);


    icp.align(*final2);
    if(icp.hasConverged())
    {
        //std::cout << "convergence criteria" << icp.getConvergeCriteria()-><< std::endl;
        std::cout << "number of iteration " << icp.nr_iterations_ << std::endl;
        if(cpt == 0)
        {
            _transformationICP = icp.getFinalTransformation();
            cpt++;
        }
        else // add Transformation in order to be in frame 1
        {
            //transformation = icp.getFinalTransformation() * transformation;
            //transformation = transformation.inverse().eval();
            _transformationICP = _transformationICP * icp.getFinalTransformation();
            // transformation = icp.getFinalTransformation();
        }
        //_transformationICP = _transformationICP * icp.getFinalTransformation();
        std::cout << "----- score " << icp.getFitnessScore() << std::endl;
        pcl::transformPointCloud (*cloud_transf2, *frameIn1_, _transformationICP);
        pcl::transformPointCloud (*ground, *ground, transform_1);
        pcl::transformPointCloud (*ground, *ground, _transformationICP);
        if(isFirstFrame)
        {
            pcl::PointIndices::Ptr floor_inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr floor_coeff(new pcl::ModelCoefficients);
            pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);
            seg.setInputCloud(ground);
            seg.segment(*floor_inliers, *floor_coeff);
            Eigen::Vector3f normal_floor(floor_coeff->values[0],floor_coeff->values[1],floor_coeff->values[2]);
            computeTransformationToOrigin(ground,normal_floor, _transformationXYPlane, _transformationXYTranslation);
            isFirstFrame = false;
        }
        pcl::transformPointCloud (*ground, *ground, _transformationXYPlane);
        pcl::transformPointCloud (*ground, *ground, _transformationXYTranslation);
        *_ground += *ground;

        if(cpt != 0 )
        {
            //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground_hull (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::ConcaveHull<pcl::PointXYZRGBA> ghull;
            ghull.setInputCloud(_ground);
            ghull.setAlpha(0.8);
            ghull.reconstruct(*_ground);
        }

        pcl::transformPointCloud (*mc, *mc, transform_1);
        pcl::transformPointCloud (*mc, *mc, _transformationICP);
        *_mc += *mc;

        // check planes
        for(unsigned int p = 0; p < pl.size(); p++)
        {

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plTransf(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr centroidTransf(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Vector4f centroid = pl[p].getCentroid();
            pcl::PointXYZ centroidPoint;
            centroidPoint.x = centroid[0];
            centroidPoint.y = centroid[1];
            centroidPoint.z = centroid[2];
            //std::cout << " centroid Z  " << centroid[2] << std::endl;

            centroidTransf->push_back(centroidPoint);
            pcl::transformPointCloud (*centroidTransf, *centroidTransf, transform_1);
            pcl::transformPointCloud (*centroidTransf, *centroidTransf, _transformationICP);
            pcl::transformPointCloud (*centroidTransf, *centroidTransf, _transformationXYPlane);
            pcl::transformPointCloud (*centroidTransf, *centroidTransf, _transformationXYTranslation);
            plTransf = pl[p].getHull();
            Eigen::Vector4f height = pl[p].getHeight();
            pcl::transformPointCloud (*plTransf, *plTransf, transform_1);
            pcl::transformPointCloud (*plTransf, *plTransf, _transformationICP);
            pcl::transformPointCloud (*plTransf, *plTransf, _transformationXYPlane);
            pcl::transformPointCloud (*plTransf, *plTransf, _transformationXYTranslation);
            pcl::PointCloud<pcl::PointXYZ>::Ptr coeff(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ModelCoefficients::Ptr coefficients(new  pcl::ModelCoefficients);
            coefficients = pl[p].getCoefficients();
            pcl::PointXYZ coeffPoint;
            coeffPoint.x = coefficients->values[0];
            coeffPoint.y = coefficients->values[1];
            coeffPoint.z = coefficients->values[2];
            coeff->push_back(coeffPoint);
            pcl::transformPointCloud (*coeff, *coeff, transform_1);
            pcl::transformPointCloud (*coeff, *coeff, _transformationICP);
            pcl::transformPointCloud (*coeff, *coeff, _transformationXYPlane);
            pcl::transformPointCloud (*coeff, *coeff, _transformationXYTranslation);
            coefficients->values[0] = coeff->points[0].x;
            coefficients->values[1] = coeff->points[0].y;
            coefficients->values[2] = coeff->points[0].z;

            Plane plane;
            plane.setCentroid(centroid);
            plane.setHull(plTransf);
            plane.setHeight(height);
            plane.setCoefficients(coefficients);
            plane.setType(pl[p].getTYPE());
            //pcl::ModelCoefficients::Ptr coeff(new  pcl::ModelCoefficients);
            //coeff = pl[p].getCoefficients();
            //                pcl::ModelCoefficients::Ptr coeffNew(new  pcl::ModelCoefficients);
            //                coeffNew->values[0] = coffTrans->points[0].x;
            //                coeffNew->values[1] = coffTrans->points[0].y;
            //                coeffNew->values[2] = coffTrans->points[0].z;
            //                plane.setCoefficients(coeff);
            //std::cout << "yolo" << plane.getCoefficients()->values[0] << std::endl;

            Eigen::Vector4f centroid_(centroidTransf->points[0].x,centroidTransf->points[0].y,centroidTransf->points[0].z, 0.0 );
            plane.setCentroidTrans(centroid_);
            pcl::ModelCoefficients::Ptr coeff_current(new  pcl::ModelCoefficients);
            coeff_current = plane.getNormalizedCoefficients();
            if(!planeDetected)
            {
                _planes_cloud.push_back(plane);
            }
            else
            {
                for(unsigned int j = 0 ; j < _planes_cloud.size() + 1;j++)
                {
                    if(j == _planes_cloud.size())
                    {
                        _planes_cloud.push_back(plane);
                        break;
                    }

                    //std::cout << "Hi !!! " << j << std::endl;
                    //Eigen::Vector4f heightPrev = _planes_cloud[j].getHeight();
                    Eigen::Vector4f centroidPrev = _planes_cloud[j].getCentroidTrans();
                    pcl::ModelCoefficients::Ptr coeff_prev(new  pcl::ModelCoefficients);
                    coeff_prev = _planes_cloud[j].getNormalizedCoefficients();

                    //Eigen::Vector4f centroid_(centroidTransf->points[0].x,centroidTransf->points[0].y,centroidTransf->points[0].z, 0.0 );
                    //std::cout << "height " << height[0] << " and" << heightPrev[0] << std::endl;

                    // check the position of the centroid
                    if((computeDistanceCentroid(centroid_, centroidPrev, 0.15 ) == true))
                    {
                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes4(new pcl::PointCloud<pcl::PointXYZRGBA>);
                        //Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
                        planes4 = _planes_cloud[j].getHull();
                        _planes_cloud[j].setHeight(height);
                        _planes_cloud[j].setCentroidTrans(centroid_);
                        _planes_cloud[j].concatenateCloud(plTransf); // concatenate Hull
                        break;
                    }
                    //                        else if (computeDistanceCentroidXY(centroid_,  centroidPrev, 0.03 ) == true)
                    //                        {
                    //                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes4(new pcl::PointCloud<pcl::PointXYZRGBA>);
                    //                            //Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
                    //                            planes4 = _planes_cloud[j].getHull();
                    //                            _planes_cloud[j].setHeight(height);
                    //                            _planes_cloud[j].setCentroid(centroid_);
                    //                            _planes_cloud[j].concatenateCloud(plTransf); // concatenate Hull
                    //                            break;
                    //                        }
                    //                    else if(computeAngleNormal(coeff_prev, coeff_current,8))
                    //                    {
                    //                        //checkSamePlane(plane,_planes_cloud[j]);
                    //                        if(checkSamePlane(plane,_planes_cloud[j]) == true)
                    //                        {
                    //                           std::cout << "same plane !!" << std::endl;
                    //                           ///pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes5(new pcl::PointCloud<pcl::PointXYZRGBA>);
                    //                           //Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
                    //                           //planes5 = _planes_cloud[j].getHull();
                    //                           _planes_cloud[j].setHeight(height);
                    //                           _planes_cloud[j].setCentroidTrans(centroid_);
                    //                           _planes_cloud[j].concatenateCloud(plTransf); // concatenate Hull
                    //                           break;
                    //                        }
                    //                    }
                    else
                    {
                        // std::cout << "Add new plane !!" << std::endl;
                        //double diff = std::abs(centroid_[2]) - std::abs(centroidPrev[2]);
                    }
                }
            }
            //std::cout << "_planes_cloud " << _planes_cloud[p].getPlaneCloud()->points.size() << std::endl;

        }
        if(!planeDetected)
        {
            _prev_planes.clear();
            _prev_planes = pl;
            planeDetected = true;
        }


        //    if(prealignement(out_cloud,prev_cloud) == true)
        //    {
        //        pcl::transformPointCloud (*out_cloud, *out_cloud, _transformationICP);
        //        prev_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        //
        //    }
        for(size_t i = 0; i < frameIn1_->points.size(); i++)
        {
            pcl::PointXYZRGBA point;
            point.x = frameIn1_->points[i].x;
            point.y = frameIn1_->points[i].y;
            point.z = frameIn1_->points[i].z;
            frameIn1->push_back(point);

        }
        //pcl::transformPointCloud (*cloud, *cloud_transf, transform_1);
        pcl::transformPointCloud (*frameIn1, *frameIn1, _transformationXYPlane);
        pcl::transformPointCloud (*frameIn1, *frameIn1, _transformationXYTranslation);
        *source_cloud += *frameIn1;
        *prev_cloud2 = *cloud_transf2;
    }
    else
    {
        std::cerr << "icp failed" << std::endl;
    }

}

void registerCloudWithNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &ground,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &mc, std::vector<Plane> &pl)
{
    // create cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_transf(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr frameIn1(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  cloud2 (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  cloud_transf2 (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  final2 (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr  frameIn1_ (new pcl::PointCloud<pcl::PointNormal>);

    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        pcl::PointNormal point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        cloud2->push_back(point);

    }
    // transfrom camera pose
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();


    transform_1 (0,0) = camPoseSaved.rotationMatrix()(0,0); //cos (theta);
    transform_1 (0,1) = camPoseSaved.rotationMatrix()(0,1);// -sin(theta)
    transform_1 (0,2) = camPoseSaved.rotationMatrix()(0,2);
    transform_1 (1,0) = camPoseSaved.rotationMatrix()(1,0); // sin (theta)
    transform_1 (1,1) = camPoseSaved.rotationMatrix()(1,1); //cos (theta)
    transform_1 (1,2) = camPoseSaved.rotationMatrix()(1,2);
    transform_1 (2,0) = camPoseSaved.rotationMatrix()(2,0) ;
    transform_1 (2,1) = camPoseSaved.rotationMatrix()(2,1);
    transform_1 (2,2) = camPoseSaved.rotationMatrix()(2,2);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform_1 (0,3) = camPoseSaved.translation()[0]; //2.5
    transform_1 (1,3) = camPoseSaved.translation()[1]; //2.5
    transform_1 (2,3) = camPoseSaved.translation()[2]; //2.5
    //std::cout << " Translation : " << savedTranslation[0]<< "  " << savedTranslation[1]<< "  "<< savedTranslation[2] << std::endl;
    //computeRotationMatrix(transform_1);
    pcl::transformPointCloud (*cloud2, *cloud_transf2, transform_1);


    //std::stringstream ss;
    //ss << cpt;
    //pcl::io::savePCDFileASCII (ss.str() + ".pcd", *out_cloud);
    //cpt++;

    // if prev_cloud empty fill it with current frame
    if(prev_cloud2->points.size() == 0)
    {
        *prev_cloud2 = *cloud_transf2;
        //*source_cloud = *cloud_transf;
        std::cout << "initialisation" << std::endl;
    }
    // pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    // ICP
    //pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    //pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    //pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
    //typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> PointToPlane;
    //boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
    //icp.setTransformationEstimation(point_to_plane);

    pcl::NormalEstimationOMP<pcl::PointNormal,pcl::PointNormal> nest;
    nest.setRadiusSearch (leaf+0.05);
    nest.setInputCloud (prev_cloud2);
    nest.compute (*prev_cloud2);
    nest.setInputCloud (cloud_transf2);
    nest.compute (*cloud_transf2);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setInputSource(cloud_transf2);
    icp.setInputTarget(prev_cloud2);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (leaf * 1.5);//0.1 0.25
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (500);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);//1e-5
    //icp.setTransformationEpsilon (1e-12);//1e-5
    // Set the euclidean distance difference epsregisterCloudilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1e-8); //1e-5, 1e-8
    // icp.setUseReciprocalCorrespondences(true);
    //icp.setTransformationEpsilon(0);
    icp.setRANSACOutlierRejectionThreshold(1.5f * leaf);
    //    icp.setMaxCorrespondenceDistance (0.1 ); // 0.25, 0.10
    //    icp.setMaximumIterations(5000); // 10, 100 /// 50
    //    //icp.setTransformationEpsilon (1-5); //1-8,  //1-5
    //    //    // Set the euclidean distance difference epsilon (criterion 3)
    //    icp.setEuclideanFitnessEpsilon (0.01);//1-5, 0.001 //0.01
    //    icp.setTransformationEpsilon(1-5);
    //    icp.setRANSACOutlierRejectionThreshold(1.5f * 0.015);


    icp.align(*final2);
    if(icp.hasConverged())
    {
        //std::cout << "convergence criteria" << icp.getConvergeCriteria()-><< std::endl;
        std::cout << "number of iteration " << icp.nr_iterations_ << std::endl;
        if(cpt == 0)
        {
            _transformationICP = icp.getFinalTransformation();
            cpt++;
        }
        else // add Transformation in order to be in frame 1
        {
            //transformation = icp.getFinalTransformation() * transformation;
            //transformation = transformation.inverse().eval();
            _transformationICP = _transformationICP * icp.getFinalTransformation();
            // transformation = icp.getFinalTransformation();
        }
        //_transformationICP = _transformationICP * icp.getFinalTransformation();
        std::cout << "----- score " << icp.getFitnessScore() << std::endl;
        pcl::transformPointCloud (*cloud_transf2, *frameIn1_, _transformationICP);
        pcl::transformPointCloud (*ground, *ground, transform_1);
        pcl::transformPointCloud (*ground, *ground, _transformationICP);

        *_ground += *ground;

        if(cpt != 0 )
        {
            //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground_hull (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::ConcaveHull<pcl::PointXYZRGBA> ghull;
            ghull.setInputCloud(_ground);
            ghull.setAlpha(0.8);
            ghull.reconstruct(*_ground);
        }

        pcl::transformPointCloud (*mc, *mc, transform_1);
        pcl::transformPointCloud (*mc, *mc, _transformationICP);
        *_mc += *mc;

        // check planes
        for(unsigned int p = 0; p < pl.size(); p++)
        {

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plTransf(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr centroidTransf(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Vector4f centroid = pl[p].getCentroid();
            pcl::PointXYZ centroidPoint;
            centroidPoint.x = centroid[0];
            centroidPoint.y = centroid[1];
            centroidPoint.z = centroid[2];
            //std::cout << " centroid Z  " << centroid[2] << std::endl;

            centroidTransf->push_back(centroidPoint);
            pcl::transformPointCloud (*centroidTransf, *centroidTransf, transform_1);
            pcl::transformPointCloud (*centroidTransf, *centroidTransf, _transformationICP);

            plTransf = pl[p].getHull();
            Eigen::Vector4f height = pl[p].getHeight();
            pcl::transformPointCloud (*plTransf, *plTransf, transform_1);
            pcl::transformPointCloud (*plTransf, *plTransf, _transformationICP);

            pcl::PointCloud<pcl::PointXYZ>::Ptr coeff(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ModelCoefficients::Ptr coefficients(new  pcl::ModelCoefficients);
            coefficients = pl[p].getCoefficients();
            pcl::PointXYZ coeffPoint;
            coeffPoint.x = coefficients->values[0];
            coeffPoint.y = coefficients->values[1];
            coeffPoint.z = coefficients->values[2];
            coeff->push_back(coeffPoint);
            pcl::transformPointCloud (*coeff, *coeff, transform_1);
            pcl::transformPointCloud (*coeff, *coeff, _transformationICP);
            coefficients->values[0] = coeff->points[0].x;
            coefficients->values[1] = coeff->points[0].y;
            coefficients->values[2] = coeff->points[0].z;

            Plane plane;
            plane.setCentroid(centroid);
            plane.setHull(plTransf);
            plane.setHeight(height);
            plane.setCoefficients(coefficients);
            plane.setType(pl[p].getTYPE());
            //pcl::ModelCoefficients::Ptr coeff(new  pcl::ModelCoefficients);
            //coeff = pl[p].getCoefficients();
            //                pcl::ModelCoefficients::Ptr coeffNew(new  pcl::ModelCoefficients);
            //                coeffNew->values[0] = coffTrans->points[0].x;
            //                coeffNew->values[1] = coffTrans->points[0].y;
            //                coeffNew->values[2] = coffTrans->points[0].z;
            //                plane.setCoefficients(coeff);
            //std::cout << "yolo" << plane.getCoefficients()->values[0] << std::endl;

            Eigen::Vector4f centroid_(centroidTransf->points[0].x,centroidTransf->points[0].y,centroidTransf->points[0].z, 0.0 );
            plane.setCentroidTrans(centroid_);
            pcl::ModelCoefficients::Ptr coeff_current(new  pcl::ModelCoefficients);
            coeff_current = plane.getNormalizedCoefficients();
            if(!planeDetected)
            {
                _planes_cloud.push_back(plane);
            }
            else
            {
                for(unsigned int j = 0 ; j < _planes_cloud.size() + 1;j++)
                {
                    if(j == _planes_cloud.size())
                    {
                        _planes_cloud.push_back(plane);
                        break;
                    }

                    //std::cout << "Hi !!! " << j << std::endl;
                    //Eigen::Vector4f heightPrev = _planes_cloud[j].getHeight();
                    Eigen::Vector4f centroidPrev = _planes_cloud[j].getCentroidTrans();
                    pcl::ModelCoefficients::Ptr coeff_prev(new  pcl::ModelCoefficients);
                    coeff_prev = _planes_cloud[j].getNormalizedCoefficients();

                    //Eigen::Vector4f centroid_(centroidTransf->points[0].x,centroidTransf->points[0].y,centroidTransf->points[0].z, 0.0 );
                    //std::cout << "height " << height[0] << " and" << heightPrev[0] << std::endl;

                    // check the position of the centroid
                    if((computeDistanceCentroid(centroid_, centroidPrev, 0.15 ) == true))
                    {
                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes4(new pcl::PointCloud<pcl::PointXYZRGBA>);
                        //Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
                        planes4 = _planes_cloud[j].getHull();
                        _planes_cloud[j].setHeight(height);
                        _planes_cloud[j].setCentroidTrans(centroid_);
                        _planes_cloud[j].concatenateCloud(plTransf); // concatenate Hull
                        break;
                    }
                    //                        else if (computeDistanceCentroidXY(centroid_,  centroidPrev, 0.03 ) == true)
                    //                        {
                    //                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes4(new pcl::PointCloud<pcl::PointXYZRGBA>);
                    //                            //Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
                    //                            planes4 = _planes_cloud[j].getHull();
                    //                            _planes_cloud[j].setHeight(height);
                    //                            _planes_cloud[j].setCentroid(centroid_);
                    //                            _planes_cloud[j].concatenateCloud(plTransf); // concatenate Hull
                    //                            break;
                    //                        }
                    //                    else if(computeAngleNormal(coeff_prev, coeff_current,8))
                    //                    {
                    //                        //checkSamePlane(plane,_planes_cloud[j]);
                    //                        if(checkSamePlane(plane,_planes_cloud[j]) == true)
                    //                        {
                    //                           std::cout << "same plane !!" << std::endl;
                    //                           ///pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes5(new pcl::PointCloud<pcl::PointXYZRGBA>);
                    //                           //Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
                    //                           //planes5 = _planes_cloud[j].getHull();
                    //                           _planes_cloud[j].setHeight(height);
                    //                           _planes_cloud[j].setCentroidTrans(centroid_);
                    //                           _planes_cloud[j].concatenateCloud(plTransf); // concatenate Hull
                    //                           break;
                    //                        }
                    //                    }
                    else
                    {
                        // std::cout << "Add new plane !!" << std::endl;
                        //double diff = std::abs(centroid_[2]) - std::abs(centroidPrev[2]);
                    }
                }
            }
            //std::cout << "_planes_cloud " << _planes_cloud[p].getPlaneCloud()->points.size() << std::endl;

        }
        if(!planeDetected)
        {
            _prev_planes.clear();
            _prev_planes = pl;
            planeDetected = true;
        }


        //    if(prealignement(out_cloud,prev_cloud) == true)
        //    {
        //        pcl::transformPointCloud (*out_cloud, *out_cloud, _transformationICP);
        //        prev_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        //
        //    }
        for(size_t i = 0; i < frameIn1_->points.size(); i++)
        {
            pcl::PointXYZRGBA point;
            point.x = frameIn1_->points[i].x;
            point.y = frameIn1_->points[i].y;
            point.z = frameIn1_->points[i].z;
            frameIn1->push_back(point);

        }
        //pcl::transformPointCloud (*cloud, *cloud_transf, transform_1);
        *source_cloud += *frameIn1;
        *prev_cloud2 = *cloud_transf2;
    }
    else
    {
        std::cerr << "icp failed" << std::endl;
    }

}

/*
* Transform the cloud in the first frame and register it.
* params[in/out]: cloud, ground, main cloud, planes
*/
void registerCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &ground,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &mc, std::vector<Plane> &pl)
{
    // create cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_transf(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr frameIn1(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // transfrom camera pose
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();


    transform_1 (0,0) = camPoseSaved.rotationMatrix()(0,0); //cos (theta);
    transform_1 (0,1) = camPoseSaved.rotationMatrix()(0,1);// -sin(theta)
    transform_1 (0,2) = camPoseSaved.rotationMatrix()(0,2);
    transform_1 (1,0) = camPoseSaved.rotationMatrix()(1,0); // sin (theta)
    transform_1 (1,1) = camPoseSaved.rotationMatrix()(1,1); //cos (theta)
    transform_1 (1,2) = camPoseSaved.rotationMatrix()(1,2);
    transform_1 (2,0) = camPoseSaved.rotationMatrix()(2,0) ;
    transform_1 (2,1) = camPoseSaved.rotationMatrix()(2,1);
    transform_1 (2,2) = camPoseSaved.rotationMatrix()(2,2);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform_1 (0,3) = camPoseSaved.translation()[0]; //2.5
    transform_1 (1,3) = camPoseSaved.translation()[1]; //2.5
    transform_1 (2,3) = camPoseSaved.translation()[2]; //2.5
    //std::cout << " Translation : " << savedTranslation[0]<< "  " << savedTranslation[1]<< "  "<< savedTranslation[2] << std::endl;
    //computeRotationMatrix(transform_1);
    pcl::transformPointCloud (*cloud, *cloud_transf, transform_1);


    //std::stringstream ss;
    //ss << cpt;
    //pcl::io::savePCDFileASCII (ss.str() + ".pcd", *out_cloud);
    //cpt++;

    // if prev_cloud empty fill it with current frame
    if(prev_cloud->points.size() == 0)
    {
        *prev_cloud = *cloud_transf;
        *source_cloud = *cloud_transf;
        std::cout << "initialisation" << std::endl;
    }

    // ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(cloud_transf);
    icp.setInputTarget(prev_cloud);
    const float leaf2 = 0.015f;
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (1.5f * leaf2);//0.1 0.25
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50000);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-5);//1e-5
    // Set the euclidean distance difference epsregisterCloudilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1e-5); //1e-5, 1e-8
    icp.setTransformationEpsilon(0);
    icp.setRANSACOutlierRejectionThreshold(1.5f * leaf2);
    //    icp.setMaxCorrespondenceDistance (0.1 ); // 0.25, 0.10
    //    icp.setMaximumIterations(5000); // 10, 100 /// 50
    //    //icp.setTransformationEpsilon (1-5); //1-8,  //1-5
    //    //    // Set the euclidean distance difference epsilon (criterion 3)
    //    icp.setEuclideanFitnessEpsilon (0.01);//1-5, 0.001 //0.01
    //    icp.setTransformationEpsilon(1-5);
    //    icp.setRANSACOutlierRejectionThreshold(1.5f * 0.015);

    icp.align(*final);
    if(icp.hasConverged())
    {
        if(cpt == 0)
        {
            _transformationICP = icp.getFinalTransformation();
            cpt++;
        }
        else // add Transformation in order to be in frame 1
        {
            //transformation = icp.getFinalTransformation() * transformation;
            //transformation = transformation.inverse().eval();
            _transformationICP = _transformationICP * icp.getFinalTransformation();
            // transformation = icp.getFinalTransformation();
        }
        //_transformationICP = _transformationICP * icp.getFinalTransformation();
        std::cout << "----- score " << icp.getFitnessScore() << std::endl;

        pcl::transformPointCloud (*cloud_transf, *frameIn1, _transformationICP);
        pcl::transformPointCloud (*ground, *ground, transform_1);
        pcl::transformPointCloud (*ground, *ground, _transformationICP);

        *_ground += *ground;

        if(cpt != 0 )
        {
            //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground_hull (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::ConcaveHull<pcl::PointXYZRGBA> ghull;
            ghull.setInputCloud(_ground);
            ghull.setAlpha(0.5);
            ghull.reconstruct(*_ground);
        }

        pcl::transformPointCloud (*mc, *mc, transform_1);
        pcl::transformPointCloud (*mc, *mc, _transformationICP);
        *_mc += *mc;

        // check planes
        for(unsigned int p = 0; p < pl.size(); p++)
        {

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plTransf(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr centroidTransf(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Vector4f centroid = pl[p].getCentroid();
            pcl::PointXYZ centroidPoint;
            centroidPoint.x = centroid[0];
            centroidPoint.y = centroid[1];
            centroidPoint.z = centroid[2];
            centroidTransf->push_back(centroidPoint);
            pcl::transformPointCloud (*centroidTransf, *centroidTransf, transform_1);
            pcl::transformPointCloud (*centroidTransf, *centroidTransf, _transformationICP);

            plTransf = pl[p].getHull();
            Eigen::Vector4f height = pl[p].getHeight();
            pcl::transformPointCloud (*plTransf, *plTransf, transform_1);
            pcl::transformPointCloud (*plTransf, *plTransf, _transformationICP);

            //                pcl::ModelCoefficients::Ptr coeff(new  pcl::ModelCoefficients);
            //                pcl::PointCloud<pcl::PointXYZ>::Ptr coffTrans(new pcl::PointCloud<pcl::PointXYZ>);

            //                pcl::PointXYZ coff;
            //                coeff = pl[p].getCoefficients();
            //                coff.x = coeff->values[0];
            //                coff.y = coeff->values[1];
            //                coff.z = coeff->values[2];
            //                std ::cout << "Hey " << coff.x << std::endl;
            //                coffTrans->push_back(coff);
            //                pcl::transformPointCloud (*coffTrans, *coffTrans, transform_1);
            //                pcl::transformPointCloud (*coffTrans, *coffTrans, _transformationICP);

            Plane plane;
            plane.setHull(plTransf);
            plane.setHeight(height);
            //pcl::ModelCoefficients::Ptr coeff(new  pcl::ModelCoefficients);
            //coeff = pl[p].getCoefficients();
            //                pcl::ModelCoefficients::Ptr coeffNew(new  pcl::ModelCoefficients);
            //                coeffNew->values[0] = coffTrans->points[0].x;
            //                coeffNew->values[1] = coffTrans->points[0].y;
            //                coeffNew->values[2] = coffTrans->points[0].z;
            //                plane.setCoefficients(coeff);
            //std::cout << "yolo" << plane.getCoefficients()->values[0] << std::endl;

            Eigen::Vector4f centroid_(centroidTransf->points[0].x,centroidTransf->points[0].y,centroidTransf->points[0].z, 0.0 );
            plane.setCentroid(centroid_);

            if(!planeDetected)
            {
                _planes_cloud.push_back(plane);
            }
            else
            {
                for(unsigned int j = 0 ; j < _planes_cloud.size() + 1;j++)
                {
                    if(j == _planes_cloud.size())
                    {
                        _planes_cloud.push_back(plane);
                        break;
                    }
                    //std::cout << "Hi !!! " << j << std::endl;
                    //Eigen::Vector4f heightPrev = _planes_cloud[j].getHeight();
                    Eigen::Vector4f centroidPrev = _planes_cloud[j].getCentroid();


                    //Eigen::Vector4f centroid_(centroidTransf->points[0].x,centroidTransf->points[0].y,centroidTransf->points[0].z, 0.0 );
                    //std::cout << "height " << height[0] << " and" << heightPrev[0] << std::endl;

                    // check the position of the centroid
                    if((computeDistanceCentroid(centroid_, centroidPrev, 0.12 ) == true))
                    {
                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes4(new pcl::PointCloud<pcl::PointXYZRGBA>);
                        //Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
                        planes4 = _planes_cloud[j].getHull();
                        _planes_cloud[j].setHeight(height);
                        _planes_cloud[j].setCentroid(centroid_);
                        _planes_cloud[j].concatenateCloud(plTransf); // concatenate Hull
                        break;
                    }
                    else
                    {
                        double diff = centroid_[3] - centroidPrev[3];
                        std::cout << " centroid Z " << diff << std::endl;
                    }
                    //                        else if (computeDistanceCentroidXY(centroid_,  centroidPrev, 0.03 ) == true)
                    //                        {
                    //                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes4(new pcl::PointCloud<pcl::PointXYZRGBA>);
                    //                            //Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
                    //                            planes4 = _planes_cloud[j].getHull();
                    //                            _planes_cloud[j].setHeight(height);
                    //                            _planes_cloud[j].setCentroid(centroid_);
                    //                            _planes_cloud[j].concatenateCloud(plTransf); // concatenate Hull
                    //                            break;
                    //                        }


                }
            }
            //std::cout << "_planes_cloud " << _planes_cloud[p].getPlaneCloud()->points.size() << std::endl;

        }
        if(!planeDetected)
        {
            _prev_planes.clear();
            _prev_planes = pl;
            planeDetected = true;
        }


        //    if(prealignement(out_cloud,prev_cloud) == true)
        //    {
        //        pcl::transformPointCloud (*out_cloud, *out_cloud, _transformationICP);
        //        prev_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        //
        //    }

        *source_cloud += *frameIn1;
        *prev_cloud = *cloud_transf;
    }
    else
    {
        std::cerr << "icp failed" << std::endl;
    }

}

void transformCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &ground,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &mc, std::vector<Plane> &pl)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out2(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    //float theta = M_PI/4; // The angle of rotation in radians
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    transform_1 (0,0) = camPoseSaved.rotationMatrix()(0,0); //cos (theta);
    transform_1 (0,1) = camPoseSaved.rotationMatrix()(0,1);// -sin(theta)
    transform_1 (0,2) = camPoseSaved.rotationMatrix()(0,2);
    transform_1 (1,0) = camPoseSaved.rotationMatrix()(1,0); // sin (theta)
    transform_1 (1,1) = camPoseSaved.rotationMatrix()(1,1); //cos (theta)
    transform_1 (1,2) = camPoseSaved.rotationMatrix()(1,2);
    transform_1 (2,0) = camPoseSaved.rotationMatrix()(2,0) ;
    transform_1 (2,1) = camPoseSaved.rotationMatrix()(2,1);
    transform_1 (2,2) = camPoseSaved.rotationMatrix()(2,2);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform_1 (0,3) = camPoseSaved.translation()[0]; //2.5
    transform_1 (1,3) = camPoseSaved.translation()[1]; //2.5
    transform_1 (2,3) = camPoseSaved.translation()[2]; //2.5
    //computeRotationMatrix(transform_1);
    pcl::transformPointCloud (*cloud, *out_cloud, transform_1);
    //    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGBA>);

    //    if(prev_cloud->size() > 0)
    //    {
    //        pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    //        icp.setInputSource(out_cloud);
    //        icp.setInputTarget(prev_cloud);
    //        //-----------------------------
    //        //        icp.setMaxCorrespondenceDistance (0.10); // 0.25, 0.10

    //        //        icp.setMaximumIterations(1000); // 10, 100 /// 50
    //        //        icp.setTransformationEpsilon (1e-5); //1-8, 1-5 //1-5
    //        //        // Set the euclidean distance difference epsilon (criterion 3)
    //        //        icp.setEuclideanFitnessEpsilon (0.01);//1-5, 0.001 //0.01
    //        //-----------------------------
    //        const float leaf = 0.015f;
    //        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    //        icp.setMaxCorrespondenceDistance (1.5f * leaf);//0.1 0.25
    //        // Set the maximum number of iterations (criterion 1)
    //        icp.setMaximumIterations (50000);
    //        // Set the transformation epsilon (criterion 2)
    //        icp.setTransformationEpsilon (1e-5);//1e-5
    //        // Set the euclidean distance difference epsilon (criterion 3)
    //        icp.setEuclideanFitnessEpsilon (1e-5); //1e-5, 1e-8

    //        icp.setTransformationEpsilon(0);
    //        icp.setRANSACOutlierRejectionThreshold(1.5f * leaf);

    //        //icp.setRANSACOutlierRejectionThreshold(0.01);

    //        icp.align(*final);
    //        //std::vector<int> indices;
    //        //final->is_dense = false;
    //        //pcl::removeNaNFromPointCloud(*final,*final, indices);
    //        if(cpt == 0)
    //        {
    //            transformation = icp.getFinalTransformation();
    //            cpt++;
    //        }
    //        else
    //        {
    //            //transformation = icp.getFinalTransformation() * transformation;
    //            //transformation = transformation.inverse().eval();
    //            transformation = transformation * icp.getFinalTransformation();
    //            // transformation = icp.getFinalTransformation();
    //        }

    //        pcl::transformPointCloud (*out_cloud, *out2, transformation);
    //        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground_transf(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //        //final.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //        pcl::transformPointCloud (*ground, *ground, transform_1);
    //        pcl::transformPointCloud (*ground, *ground_transf, transformation);
    //        pcl::transformPointCloud (*mc, *mc, transform_1);
    //        pcl::transformPointCloud (*mc, *mc, transformation);

    //        *source_cloud += *final;
    //        *_ground += *ground_transf;
    //        //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground_hull (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //        //        pcl::ConcaveHull<pcl::PointXYZRGBA> chull;
    //        //        chull.setInputCloud(_ground);
    //        //        chull.setAlpha(0.1);
    //        //        chull.reconstruct(*_ground);
    //        *_mc += *mc;
    //        std::cout << " fitness " << icp.getFitnessScore() << std::endl;
    //        //_planes_cloud.clear();
    //        for(unsigned int p = 0; p < pl.size(); p++)
    //        {

    //            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plTransf(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //            pcl::PointCloud<pcl::PointXYZ>::Ptr centroidTransf(new pcl::PointCloud<pcl::PointXYZ>);
    //            Eigen::Vector4f centroid = pl[p].getCentroid();
    //            pcl::PointXYZ centroidPoint;
    //            centroidPoint.x = centroid[0];
    //            centroidPoint.y = centroid[1];
    //            centroidPoint.z = centroid[2];
    //            centroidTransf->push_back(centroidPoint);
    //            pcl::transformPointCloud (*centroidTransf, *centroidTransf, transform_1);
    //            pcl::transformPointCloud (*centroidTransf, *centroidTransf, transformation);


    //            plTransf = pl[p].getHull();
    //            Eigen::Vector4f height = pl[p].getHeight();
    //            pcl::transformPointCloud (*plTransf, *plTransf, transform_1);
    //            pcl::transformPointCloud (*plTransf, *plTransf, transformation);
    //            Plane plane;
    //            plane.setHull(plTransf);
    //            plane.setHeight(height);
    //            Eigen::Vector4f centroid_(centroidTransf->points[0].x,centroidTransf->points[0].y,centroidTransf->points[0].z, 0.0 );

    //            plane.setCentroid(centroid_);
    //            if(!planeDetected)
    //            {
    //                _planes_cloud.push_back(plane);
    //            }
    //            else
    //            {
    //                for(unsigned int j = 0 ; j < _planes_cloud.size() + 1;j++)
    //                {
    //                    if(j == _planes_cloud.size())
    //                    {
    //                        _planes_cloud.push_back(plane);
    //                        break;
    //                    }
    //                    //std::cout << "Hi !!! " << j << std::endl;
    //                    //Eigen::Vector4f heightPrev = _planes_cloud[j].getHeight();
    //                    Eigen::Vector4f centroidPrev = _planes_cloud[j].getCentroid();


    //                    //Eigen::Vector4f centroid_(centroidTransf->points[0].x,centroidTransf->points[0].y,centroidTransf->points[0].z, 0.0 );
    //                    //std::cout << "height " << height[0] << " and" << heightPrev[0] << std::endl;
    //                    if((computeDistanceCentroid(centroid_, centroidPrev, 0.08 ) == true))
    //                    {
    //                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planes4(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //                        Eigen::Matrix4f transformation2 = Eigen::Matrix4f::Identity();
    //                        planes4 = _planes_cloud[j].getHull();
    //                        pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp_planes;
    //                        icp.setInputSource(plTransf);
    //                        icp.setInputTarget(planes4);
    //                        icp.setMaxCorrespondenceDistance (0.10); // 0.25, 0.10

    //                        icp.setMaximumIterations(1000); // 10, 100 /// 50
    //                        icp.setTransformationEpsilon (1e-8); //1-8, 1-5 //1-5
    //                        // Set the euclidean distance difference epsilon (criterion 3)
    //                        icp.setEuclideanFitnessEpsilon (0.01);//
    //                        transformation2 = icp_planes.getFinalTransformation();
    //                        //pcl::transformPointCloud (*centroid_, *centroid_, transformation2);
    //                        pcl::transformPointCloud (*plTransf, *plTransf, transformation2);
    //                        //std::cout << "plane " << p << " with plane " << j << std::endl;
    //                        _planes_cloud[j].setHeight(height);
    //                        _planes_cloud[j].setCentroid(centroid_);
    //                        _planes_cloud[j].concatenateCloud(plTransf);
    //                        break;
    //                    }

    //                }
    //            }
    //            //std::cout << "_planes_cloud " << _planes_cloud[p].getPlaneCloud()->points.size() << std::endl;
    //        }
    //        if(!planeDetected)
    //        {
    //            _prev_planes.clear();
    //            _prev_planes = pl;
    //            planeDetected = true;
    //        }

    //    }
    //    else
    //    {
    *source_cloud += *out_cloud;
    //    }
    //    //prev_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //    //if(prev_cloud->size() > 0)
    //    // {
    //    //   prev_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    //    // prev_cloud.swap(out2);
    //    // out2.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //    //}
    //    //else
    //    // {
    //    //   std::cout<< "YOLO" << std::endl;
    //    prev_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //    prev_cloud.swap(out_cloud);

    //    //}

}



void run(SlamSystem * system, Undistorter* undistorter, Sophus::Matrix3f K)
{

    //    cv::VideoCapture sensor1;
    //    sensor1.open(CV_CAP_OPENNI_ASUS);

    //    if( !sensor1.isOpened() ){
    //        std::cout << "Can not open capture object 1." << std::endl;
    //        //return -1;
    //    }
    //    else
    //    {
    //        std::cout << "sensor Open !!" << std::endl;
    //    }

    // get HZ
    double hz = 30;

    cv::Mat image = cv::Mat(h, w, CV_8U);
    int runningIDX=0;
    float fakeTimeStamp = 0;
    //for(;;)
    //{


    //          if( cv::waitKey( 30 ) == 27 )   break;//ESC to exit
    for(unsigned int i = 0; i < numFrames; i++)
    {
        if(lsdDone.getValue())
            break;
        // cv::Mat rgb;
        //cv::Mat imageDist;
        //        if( !sensor1.grab() ){
        //            std::cout << "Sensor1 can not grab images." << std::endl;

        //        }
        // else if( sensor1.retrieve( rgb, CV_CAP_OPENNI_BGR_IMAGE ) )
        //cv::imshow("depth1",rgb);
        cv::Mat imageDist = cv::Mat(h, w, CV_8U);
        cv::imshow("rgb",imageDist);
        if(logReader)
        {
            logReader->getNext();

            cv::Mat3b img(h, w, (cv::Vec3b *)logReader->rgb);

            cv::cvtColor(img, imageDist, CV_RGB2GRAY);
        }
        else
        {
            //cv::cvtColor(rgb, imageDist, CV_BGR2GRAY);
            imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);

            if(imageDist.rows != h_inp || imageDist.cols != w_inp)
            {
                if(imageDist.rows * imageDist.cols == 0)
                    std::cout << "failed to load image" << std::endl;
                //printf("failed to load image %s! skipping.\n", files[i].c_str());
                else
                    std::cout << "image has wrong dimensions" << std::endl;
                //printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
                //      files[i].c_str(),
                //    w,h,imageDist.cols, imageDist.rows);
                continue;
            }
        }

        assert(imageDist.type() == CV_8U);

        undistorter->undistort(imageDist, image);

        assert(image.type() == CV_8U);

        if(runningIDX == 0)
        {
            system->randomInit(image.data, fakeTimeStamp, runningIDX);
        }
        else
        {
            //std::cout << " track ..." << std::endl;
            system->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp);
        }

        //gui.pose.assignValue(system->getCurrentPoseEstimateScale());

        runningIDX++;
        fakeTimeStamp+=0.03;

        if(fullResetRequested)
        {
            printf("FULL RESET!\n");
            delete system;

            system = new SlamSystem(w, h, K, false);
            //system->setVisualization(outputWrapper);

            fullResetRequested = false;
            runningIDX = 0;
        }
    }

    lsdDone.assignValue(true);
}
void runWithSensor(SlamSystem * system, Undistorter* undistorter, Sophus::Matrix3f K)
{
    cv::VideoCapture sensor1;
    sensor1.open(CV_CAP_OPENNI_ASUS);
    //sensor1.open(CV_CAP_OPENNI_ASUS);
    if( !sensor1.isOpened() ){
        std::cout << "Can not open capture object 1." << std::endl;
        //return -1;
    }
    else
    {
        std::cout << "sensor Open !!" << std::endl;
    }

    // get HZ
    double hz = 30;

    cv::Mat image = cv::Mat(h, w, CV_8U);
    int runningIDX=0;
    float fakeTimeStamp = 0;
    while(!stopTracking)
    {


        //          if( cv::waitKey( 30 ) == 27 )   break;//ESC to exit
        //for(unsigned int i = 0; i < numFrames; i++)
        //{
        if(lsdDone.getValue())
            break;
        cv::Mat rgb;
        cv::Mat imageDist;
        cv::Mat depth;
        sensor1.set(CV_CAP_PROP_OPENNI_REGISTRATION,1);

        if( !sensor1.grab() )
        {
            std::cout << "Sensor1 can not grab images." << std::endl;
        }
        else if( sensor1.retrieve( rgb, CV_CAP_OPENNI_BGR_IMAGE ) && sensor1.retrieve( _depth, CV_CAP_OPENNI_DEPTH_MAP ) )

            //cv::Mat imageDist = cv::Mat(h, w, CV_8U);
            //Util::displayImage( "DebugWindow DEPTH", cv::Mat(currentFrame->height(), currentFrame->width(), CV_32F, currentFrame->image())*(1/255.0f), false );

            // Util::displayImage( "DebugWindow DEPTH", depth, true );

            if(logReader)
            {
                logReader->getNext();

                cv::Mat3b img(h, w, (cv::Vec3b *)logReader->rgb);

                cv::cvtColor(img, imageDist, CV_RGB2GRAY);
            }
            else
            {
                cv::cvtColor(rgb, imageDist, CV_BGR2GRAY);
                cv::imshow("gray",imageDist);
                //imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);

                if(imageDist.rows != h_inp || imageDist.cols != w_inp)
                {
                    if(imageDist.rows * imageDist.cols == 0)
                        std::cout << "failed to load image" << std::endl;
                    //printf("failed to load image %s! skipping.\n", files[i].c_str());
                    else
                        std::cout << "image has wrong dimensions" << std::endl;
                    //printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
                    //      files[i].c_str(),
                    //    w,h,imageDist.cols, imageDist.rows);
                    continue;
                }
            }

        assert(imageDist.type() == CV_8U);

        undistorter->undistort(imageDist, image);

        assert(image.type() == CV_8U);

        if(runningIDX == 0)
        {
            system->randomInit(image.data, fakeTimeStamp, runningIDX);
        }
        else
        {
            //std::cout << " track ..." << std::endl;
            system->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp);
        }
        //         unsigned short depthValue;
        //        for (int h = 0; h < depth.rows; h++)
        //        {
        //            for (int w = 0; w < depth.cols; w++)
        //            {
        //                depthValue = depth.at<unsigned short>(h,w);
        //            }
        //        }
        //        std::cout << "--------------------------------------------------------" << std::endl;
        //        std::cout << "pose " << system->getCurrentPoseEstimateScale().rotationMatrix()(0,0) << std::endl;
        //        std::cout << "pose " << system->getCurrentPoseEstimateScale().rotationMatrix()(1,1) << std::endl;
        //        std::cout << "pose " << system->getCurrentPoseEstimateScale().rotationMatrix()(0,2) << std::endl;
        //        std::cout << "pose " << system->getCurrentPoseEstimateScale().translation()[2] << std::endl;
        //        std::cout << "matrix" << system->getCurrentPoseEstimate().matrix() << std::endl;
        camPoseCurrent = system->getCurrentPoseEstimateScale();
        poseCu = system->getCurrentPoseEstimate();
        //gui.pose.assignValue(system->getCurrentPoseEstimateScale());
        //gui.pose.assignValue();
        runningIDX++;
        fakeTimeStamp+=0.03;

        if(fullResetRequested)
        {
            printf("FULL RESET!\n");
            delete system;

            system = new SlamSystem(w, h, K, false);
            //system->setVisualization(outputWrapper);

            fullResetRequested = false;
            runningIDX = 0;
        }
    }

    lsdDone.assignValue(true);
}



int main( int argc, char** argv )
{

    source_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    prev_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    prev_cloud2.reset(new pcl::PointCloud<pcl::PointNormal>);
    _ground.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    _mc.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    _planes_cloud.clear();
    pcl::PointXYZ normal_ground;
    int cpt_voxel = 0;
    bool useSensor = false;
    std::string calibFile;
    Undistorter* undistorter = 0;
    if(Parse::arg(argc, argv, "-c", calibFile) > 0)
    {
        undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
    }
    if(undistorter == 0)
    {
        printf("need camera calibration file! (set using -c FILE)\n");
        exit(0);
    }
    w = undistorter->getOutputWidth();
    h = undistorter->getOutputHeight();
    w_inp = undistorter->getInputWidth();
    h_inp = undistorter->getInputHeight();

    float fx = undistorter->getK().at<double>(0, 0);
    float fy = undistorter->getK().at<double>(1, 1);
    float cx = undistorter->getK().at<double>(2, 0);
    float cy = undistorter->getK().at<double>(2, 1);

    Sophus::Matrix3f K;
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    Resolution::getInstance(w, h);
    Intrinsics::getInstance(fx, fy, cx, cy);

    //gui.initImages();

    //Output3DWrapper* outputWrapper = new PangolinOutput3DWrapper(w, h, gui);

    // make slam system
    SlamSystem * system = new SlamSystem(w, h, K, true);
    //system->setVisualization(outputWrapper);

    // open image files: first try to open as file.
    std::string source;
    if(!(Parse::arg(argc, argv, "-f", source) > 0))
    {
        printf("need source files! (set using -f FOLDER or KLG)\n");
        exit(0);
    }

    Bytef * decompressionBuffer = new Bytef[Resolution::getInstance().numPixels() * 2];
    IplImage * deCompImage = 0;

    if(source.substr(source.find_last_of(".") + 1) == "klg")
    {
        // std::cout << " LOL  " << std::endl;
        logReader = new RawLogReader(decompressionBuffer,
                                     deCompImage,
                                     source);

        numFrames = logReader->getNumFrames();
    }
    else
    {
        if(getdir(source, files) >= 0)
        {
            printf("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
        }
        else if(getFile(source, files) >= 0)
        {
            printf("found %d image files in file %s!\n", (int)files.size(), source.c_str());
        }
        else
        {
            useSensor = true;
            //printf("could not load file list! wrong path / file?\n");
        }

        numFrames = (int)files.size();
    }

    //boost::thread lsdThread(run, system, undistorter, outputWrapper, K);
    boost::thread* lsdThread ;
    //boost::thread lsdThread;
    if(useSensor)
    {
        lsdThread = new boost::thread (runWithSensor, system, undistorter, K);
        //lsdThread = lsdWithSensor
    }
    else
    {
        lsdThread = new boost::thread(run, system, undistorter, K);
        //lsdThread = lsd;
    }
    Camera camera;
    camera.init();

    std::vector<osg::Vec3> colors;
    init(colors);

    Osg_gui osgGui;

    osg::ref_ptr<osg::Geode> node = osg::ref_ptr<osg::Geode>(new osg::Geode());

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudU (new pcl::PointCloud<pcl::PointXYZRGBA>);

    //camera.getPointCloud(cloud);
    //    std::cout << "cloud " << cloud->size() << std::endl;
    //    camera.undistort(cloud,cloudU);
    //    addCloud(cloud,node,gui);
    osgGui.clearRoot();
    osgGui.setGeode(node);
    osgGui.setUpdate(true);

    //gui.run();

    Segmentation seg;
    boost::thread thread(&Osg_gui::run, &osgGui);
    sleep(1);
    //boost::thread threadGrab(&Camera::run, &camera);
    std::vector<Plane> planes;
    sleep(1);
    while(osgGui.getBoolIsStopped() == false)
    {
        struct timeval tbegin,tend;
        double texec=0.0;

        // Start timer
        gettimeofday(&tbegin,NULL);
        if(!osgGui.getStopMainThread())
        {

            planes.clear();


            //camera.getPointCloud(cloud);
            getPointCloud(cloud,_depth);

            if(!cloud->empty())
            {

                camera.undistort(cloud,cloudU);
                gettimeofday(&tend,NULL);
                // Compute execution time
                texec = ((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000;
                //std::cout << "time 1 " <<  texec << std::endl;
                //addCloud(cloudU,node,gui);
                camPoseSaved = camPoseCurrent;
                poseSaved = poseCu;
                //seg.doSegmentation(cloudU);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mc (new pcl::PointCloud<pcl::PointXYZRGBA>);
                //seg.getMainCloud(mc);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground (new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
                //ground = seg.getMainPlane().getPlaneCloud();
                //seg.getPlanes(planes);
                pcl::VoxelGrid< pcl::PointXYZRGBA > sor;
                sor.setInputCloud (cloudU);
                sor.setLeafSize (leaf, leaf, leaf);

                std::vector<int> indices;
                cloudU->is_dense = false;
                //pcl::removeNaNFromPointCloud(*cloudU,*cloudU, indices);
                gettimeofday(&tend,NULL);
                // Compute execution time
                texec = ((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000;
                //std::cout << "time 2 " <<  texec << std::endl;
                sor.filter (*filtered);

                gettimeofday(&tend,NULL);
                // Compute execution time
                texec = ((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000;
                //std::cout << "time 3 " <<  texec << std::endl;
                seg.doSegmentation(filtered);
                seg.getMainCloud(mc);
                ground = seg.getMainPlane().getHull();
                seg.getPlanes(planes);
                normal_ground.x = seg.getMainPlane().getNormalizedCoefficients()->values[0];
                normal_ground.y = seg.getMainPlane().getNormalizedCoefficients()->values[1];
                normal_ground.z = seg.getMainPlane().getNormalizedCoefficients()->values[2];
                //std::cout << "norm " <<  sqrt((normal_ground.x * normal_ground.x) + (normal_ground.y * normal_ground.y) + (normal_ground.z * normal_ground.z)) << std::endl;

                registerCloudWithNormals2(filtered, ground, mc, planes);
                //transformCloud(filtered, ground, mc, planes);
                //addCloudAndGroundAndHull(source_cloud,_ground,_planes_cloud, node, osgGui,colors);
                addCloudAndGroundAndHull(source_cloud,_ground,_planes_cloud, node, osgGui,colors);
                //*_ground = *ground;
                std::cout << "number of planes detected " << _planes_cloud.size() << std::endl;
                if(!source_cloud->empty())
                {
                    //                    if(cpt_voxel%5 == 0 && cpt_voxel != 0)
                    //                    {
                    //                        pcl::VoxelGrid< pcl::PointXYZRGBA > sor2;
                    //                        sor2.setInputCloud (_mc);
                    //                        sor.setLeafSize (1.0f, 1.0f, 1.0f);
                    //                        sor2.filter (*_mc);
                    //                        //                            sor2.setInputCloud (_ground);
                    //                        //                            sor.setLeafSize (0.010f, 0.010f, 0.010f);
                    //                        //                            sor2.filter (*_ground);

                    //                    }
                    //                    cpt_voxel++;
                    //seg.doSegmentation(source_cloud);
                    //seg.getMainCloud(mc);
                    //ground = seg.getMainPlane().getPlaneCloud();
                    //seg.getPlanes(planes);
                    //addCloudAndGroundAndHull(source_cloud,_ground,_planes_cloud, node, osgGui,colors);
                }
            }
            else
            {
                //addCloud(cloud,node,gui);
            }
        }

        //addCloudAndGroundAndHull(mc,ground,planes, node, gui);
        //addCloudAndGround(cloud, ground, node, gui);
        osgGui.clearRoot();
        osgGui.setGeode(node);
        osgGui.setUpdate(true);
        gettimeofday(&tend,NULL);
        // Compute execution time
        texec = ((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000;
        //boost::this_thread::sleep (boost::posix_time::seconds (0.5));
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr map(new pcl::PointCloud<pcl::PointXYZRGBA>);
    *map = *_ground;
        for(unsigned int i = 0; i < _planes_cloud.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pla(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pla = _planes_cloud[i].getHull();
            *map += *pla;
        }

    parseToWRL(_ground,normal_ground,_planes_cloud);
    std::cout << "Map generated" << std::endl;
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *map);
    //    ofstream my_file;
    //    my_file.open("test.txt");
    //    for(size_t i = 0; i < _ground->points.size(); i++)
    //    {
    //        my_file << _ground->points[i].x << " " << _ground->points[i].y << " " << _ground->points[i].z << "\n";
    //    }
    //    my_file << "normal\n";
    //    my_file << normal_ground.x << " " << normal_ground.y << " " << normal_ground.z << "\n";
    //    std::cout << "Done !" << std::endl;
    //threadTrackingGui.join();
    stopTracking = true;
    //threadGrab.join();
    thread.join();

    lsdDone.assignValue(true);
    lsdThread->join();
    delete lsdThread;
    delete system;
    delete undistorter;
    //delete outputWrapper;
    return 0;
}
