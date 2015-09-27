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
#include "osg_gui.h"
#include "DataStructures/Frame.h"
#include "SlamSystem.h"
#include "Plane.h"
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
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>


std::string &ltrim(std::string &s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}
std::string &rtrim(std::string &s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}
std::string &trim(std::string &s
                  ) {
    return ltrim(rtrim(s));
}
int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string name = std::string(dirp->d_name);

        if(name != "." && name != "..")
            files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
    for(unsigned int i=0;i<files.size();i++)
    {
        if(files[i].at(0) != '/')
            files[i] = dir + files[i];
    }

    return files.size();
}

int getFile (std::string source, std::vector<std::string> &files)
{
    std::ifstream f(source.c_str());

    if(f.good() && f.is_open())
    {
        while(!f.eof())
        {
            std::string l;
            std::getline(f,l);

            l = trim(l);

            if(l == "" || l[0] == '#')
                continue;

            files.push_back(l);
        }

        f.close();

        size_t sp = source.find_last_of('/');
        std::string prefix;
        if(sp == std::string::npos)
            prefix = "";
        else
            prefix = source.substr(0,sp);

        for(unsigned int i=0;i<files.size();i++)
        {
            if(files[i].at(0) != '/')
                files[i] = prefix + "/" + files[i];
        }

        return (int)files.size();
    }
    else
    {
        f.close();
        return -1;
    }

}
void init(std::vector<osg::Vec3> &initColors)
{
    initColors.clear();
    initColors.push_back(osg::Vec3(0, 255 , 0));
    initColors.push_back(osg::Vec3(0, 0 , 255));
    initColors.push_back(osg::Vec3(255, 255 , 0));
    initColors.push_back(osg::Vec3(255, 0 , 255));
    initColors.push_back(osg::Vec3(0, 255 , 255));
    initColors.push_back(osg::Vec3(255, 171 , 0));
    initColors.push_back(osg::Vec3(100, 255 , 0));
    initColors.push_back(osg::Vec3(0,255 , 100));
    initColors.push_back(osg::Vec3(100, 255 , 100));
    initColors.push_back(osg::Vec3(255, 50 , 100));
    initColors.push_back(osg::Vec3(100, 50 , 255));
    initColors.push_back(osg::Vec3(255, 100 , 100));
    initColors.push_back(osg::Vec3(0, 255 , 0));
    initColors.push_back(osg::Vec3(0, 0 , 255));
    initColors.push_back(osg::Vec3(255, 255 , 0));
    initColors.push_back(osg::Vec3(255, 0 , 255));
    initColors.push_back(osg::Vec3(0, 255 , 255));
    initColors.push_back(osg::Vec3(255, 0 , 0));
    initColors.push_back(osg::Vec3(0, 255 , 0));
    initColors.push_back(osg::Vec3(0, 0 , 255));
    initColors.push_back(osg::Vec3(255, 255 , 0));
    initColors.push_back(osg::Vec3(255, 0 , 255));
    initColors.push_back(osg::Vec3(0, 255 , 255));

}

bool computeErrorHeight(float height1, float height2, float error)
{
    if(std::abs(height2 - height1) <= error)
    {
        //std::cout << "error Height true" << std::endl;
        return true;
    }
    //sstd::cout << "error Height false" << std::endl;
    return false;
}
bool computeDistanceCentroid(Eigen::Vector4f &centroid1, Eigen::Vector4f &centroid2, float error)
{
    //std::cout << "Centroid Prev " << centroid1[0] << " " << centroid1[1] << " " << centroid1[2] <<std::endl;
    //std::cout << "centroid_ " << centroid2[0] << " " << centroid2[1] << " " << centroid2[2] <<std::endl;

    float res = std::sqrt(((centroid2[0] - centroid1[0]) * (centroid2[0] - centroid1[0])) + ((centroid2[1] - centroid1[1]) * (centroid2[1] - centroid1[1])) + ((centroid2[2] - centroid1[2]) * (centroid2[2] - centroid1[2])));
    //std::cout << "error is " << res << std::endl;
    if(res <= error)
    {
        //std::cout << "error Centroid true" << std::endl;
        return true;
    }
    //std::cout << "error Centroid false" << std::endl;
    return false;
}
bool computeAngleNormal(pcl::ModelCoefficients::Ptr &coeff1, pcl::ModelCoefficients::Ptr &coeff2, float error)
{

    double res = (coeff1->values[0]*coeff2->values[0]) + (coeff1->values[1]*coeff2->values[1]) + (coeff1->values[2]*coeff2->values[2]);
    res = std::acos(res)*180/M_PI;
    std::cout << "angle is " << res << std::endl;
    if(std::abs(res) < error)
    {
        return true;
    }
    else
    {
        return false;
    }
}
void writePlane(std::ofstream &file, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &plane, Eigen::Vector3f &normal_plane)
{
    file <<"Transform {\n";
    file <<"children[\n";
    file <<"Shape{\n";
    file <<"geometry IndexedFaceSet {\n";
    file <<"ccw TRUE\n";
    file <<"coord Coordinate {\n";
    file <<"point [\n";
    for(size_t i = 0; i < plane->points.size(); i++)
    {
        file << plane->points[i].x << " "<< plane->points[i].y <<" " << plane->points[i].z <<",\n";
    }
    file <<"]\n";
    file <<"}\n";
    file <<"coordIndex [\n";
    for(size_t i = 0; i < plane->points.size(); i++)
    {
        file << i << " ";
    }
    file << -1 << ",\n";
    file <<"]\n";
    file <<"normalPerVertex FALSE\n";
    file <<"normal Normal  {\n";
    file <<"vector [\n";
    file << normal_plane[0] << " " << normal_plane[1] << " "<< normal_plane[2] <<",\n";
    file <<"]\n";
    file <<"}\n";
    file <<"normalIndex [\n";
    file <<"  0 0\n";
    file <<"]\n";
    file <<"solid FALSE\n";
    file <<"}\n";
    file <<"appearance Appearance{\n";
    file <<"  material Material{\n";
    file <<"    diffuseColor 0.5 0.0 0.0\n";
    file <<"  }\n";
    file <<"}\n";
    file <<"}\n";
    file <<"]\n";
    file <<"}\n";

}
void parseToWRL(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &ground_, pcl::PointXYZ &normal_ground, std::vector<Plane> &planes_)
{
    std::ofstream my_file;
    my_file.open("map.wrl");
    my_file << "#VRML V2.0 utf8\n";
    my_file << "\n";
    my_file << "PROTO Joint [\n";
    my_file << "exposedField     SFVec3f      center              0 0 0\n";
    my_file << "exposedField     MFNode       children            []\n";
    my_file << "exposedField     MFFloat      llimit              []\n";
    my_file << "exposedField     MFFloat      lvlimit             []\n";
    my_file << "exposedField     SFRotation   limitOrientation    0 0 1 0\n";
    my_file << "exposedField     SFString     name                \"\"\n";
    my_file << "exposedField     SFRotation   rotation            0 0 1 0\n";
    my_file << "exposedField     SFVec3f      scale               1 1 1\n";
    my_file << "exposedField     SFRotation   scaleOrientation    0 0 1 0\n";
    my_file << "exposedField     MFFloat      stiffness           [ 0 0 0 ]\n";
    my_file << "exposedField     SFVec3f      translation         0 0 0\n";
    my_file << "exposedField     MFFloat      ulimit              []\n";
    my_file << "exposedField     MFFloat      uvlimit             []\n";
    my_file << "exposedField     SFString     jointType           \"\"\n";
    my_file << "exposedField     SFInt32      jointId             -1\n";
    my_file << "exposedField     SFVec3f      jointAxis           0 0 1\n";
    my_file << "\n";
    my_file << "exposedField     SFFloat      gearRatio           1\n";
    my_file << "exposedField     SFFloat      rotorInertia        0\n";
    my_file << "exposedField     SFFloat      rotorResistor       0\n";
    my_file << "exposedField     SFFloat      torqueConst         1\n";
    my_file << "exposedField     SFFloat      encoderPulse        1\n";
    my_file << "]\n";
    my_file << "{\n";
    my_file << "Transform {\n";
    my_file << "center           IS center\n";
    my_file << "children         IS children\n";
    my_file <<"rotation         IS rotation\n";
    my_file <<"scale            IS scale\n";
    my_file <<"scaleOrientation IS scaleOrientation\n";
    my_file <<"translation      IS translation\n";
    my_file <<"  }\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"PROTO Segment [\n";
    my_file <<"field           SFVec3f     bboxCenter        0 0 0\n";
    my_file <<"field           SFVec3f     bboxSize          -1 -1 -1\n";
    my_file <<"exposedField    SFVec3f     centerOfMass      0 0 0\n";
    my_file <<"exposedField    MFNode      children          [ ]\n";
    my_file <<"exposedField    SFNode      coord             NULL\n";
    my_file <<"exposedField    MFNode      displacers        [ ]\n";
    my_file <<"exposedField    SFFloat     mass              0 \n";
    my_file <<"exposedField    MFFloat     momentsOfInertia  [ 0 0 0 0 0 0 0 0 0 ]\n";
    my_file <<"exposedField    SFString    name              \"\"\n";
    my_file <<"eventIn         MFNode      addChildren\n";
    my_file <<"eventIn         MFNode      removeChildren\n";
    my_file <<"]\n";
    my_file <<"{\n";
    my_file <<"Group {\n";
    my_file <<"addChildren    IS addChildren\n";
    my_file <<"bboxCenter     IS bboxCenter\n";
    my_file <<"bboxSize       IS bboxSize\n";
    my_file <<"children       IS children\n";
    my_file <<"removeChildren IS removeChildren\n";
    my_file <<"  }\n";
    my_file <<"}\n";
    my_file <<"PROTO Humanoid [\n";
    my_file <<"field           SFVec3f    bboxCenter            0 0 0\n";
    my_file <<"field           SFVec3f    bboxSize              -1 -1 -1\n";
    my_file <<"exposedField    SFVec3f    center                0 0 0\n";
    my_file <<"exposedField    MFNode     humanoidBody          [ ]\n";
    my_file <<"exposedField    MFString   info                  [ ]\n";
    my_file <<"exposedField    MFNode     joints                [ ]\n";
    my_file <<"exposedField    SFString   name                  \"\"\n";
    my_file <<"exposedField    SFRotation rotation              0 0 1 0\n";
    my_file <<"exposedField    SFVec3f    scale                 1 1 1\n";
    my_file <<"exposedField    SFRotation scaleOrientation      0 0 1 0\n";
    my_file <<"exposedField    MFNode     segments              [ ]\n";
    my_file <<"exposedField    MFNode     sites                 [ ]\n";
    my_file <<"exposedField    SFVec3f    translation           0 0 0\n";
    my_file <<"exposedField    SFString   version               \"1.1\"\n";
    my_file <<"exposedField    MFNode     viewpoints            [ ]\n";
    my_file <<"]\n";
    my_file <<"{\n";
    my_file <<"Transform {\n";
    my_file <<"bboxCenter       IS bboxCenter\n";
    my_file <<"bboxSize         IS bboxSize\n";
    my_file <<"center           IS center\n";
    my_file <<"rotation         IS rotation\n";
    my_file <<"scale            IS scale\n";
    my_file <<"scaleOrientation IS scaleOrientation\n";
    my_file <<"translation      IS translation\n";
    my_file <<"children [\n";
    my_file <<"Group {\n";
    my_file <<"children IS viewpoints\n";
    my_file <<"}\n";
    my_file <<"Group {\n";
    my_file <<"children IS humanoidBody \n";
    my_file <<"}\n";
    my_file <<"]\n";
    my_file <<"}\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"PROTO VisionSensor [\n";
    my_file <<"  exposedField SFVec3f    translation       0 0 0\n";
    my_file <<"  exposedField SFRotation rotation          0 0 1 0\n";
    my_file <<"  exposedField MFNode     children          [ ]\n";
    my_file <<"  exposedField SFFloat    fieldOfView       0.785398\n";
    my_file <<"  exposedField SFString   name              \"\"\n";
    my_file <<"  exposedField SFFloat    frontClipDistance 0.01\n";
    my_file <<"  exposedField SFFloat    backClipDistance  10.0\n";
    my_file <<"  exposedField SFString   type              \"NONE\"\n";
    my_file <<"  exposedField SFInt32    sensorId          -1\n";
    my_file <<"  exposedField SFInt32    width             320\n";
    my_file <<"  exposedField SFInt32    height            240\n";
    my_file <<"  exposedField SFFloat    frameRate         30\n";
    my_file <<"]\n";
    my_file <<"{\n";
    my_file <<"  Transform {\n";
    my_file <<"    rotation         IS rotation\n";
    my_file <<"    translation      IS translation\n";
    my_file <<"    children         IS children\n";
    my_file <<"  }\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"\n";
    my_file <<"PROTO ForceSensor [\n";
    my_file <<"  exposedField SFVec3f    maxForce    -1 -1 -1\n";
    my_file <<"  exposedField SFVec3f    maxTorque   -1 -1 -1\n";
    my_file <<"  exposedField SFVec3f    translation 0 0 0\n";
    my_file <<"  exposedField SFRotation rotation    0 0 1 0\n";
    my_file <<"  exposedField MFNode     children          [ ]\n";
    my_file <<"  exposedField SFInt32    sensorId    -1\n";
    my_file <<"]\n";
    my_file <<"{\n";
    my_file <<"  Transform {\n";
    my_file <<"    translation IS translation\n";
    my_file <<"    rotation    IS rotation\n";
    my_file <<"    children    IS children\n";
    my_file <<"  }\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"PROTO Gyro [\n";
    my_file <<"  exposedField SFVec3f    maxAngularVelocity -1 -1 -1\n";
    my_file <<"  exposedField SFVec3f    translation        0 0 0\n";
    my_file <<"  exposedField SFRotation rotation           0 0 1 0\n";
    my_file <<"  exposedField MFNode     children          [ ]\n";
    my_file <<"  exposedField SFInt32    sensorId           -1\n";
    my_file <<"]\n";
    my_file <<"{\n";
    my_file <<"  Transform {\n";
    my_file <<"    translation IS translation\n";
    my_file <<"    rotation    IS rotation\n";
    my_file <<"    children    IS children\n";
    my_file <<"  }\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"PROTO AccelerationSensor [\n";
    my_file <<"  exposedField SFVec3f    maxAcceleration -1 -1 -1\n";
    my_file <<"  exposedField SFVec3f    translation     0 0 0\n";
    my_file <<"  exposedField SFRotation rotation        0 0 1 0\n";
    my_file <<"  exposedField MFNode     children          [ ]\n";
    my_file <<"  exposedField SFInt32    sensorId        -1\n";
    my_file <<"]\n";
    my_file <<"{\n";
    my_file <<"  Transform {\n";
    my_file <<"    translation IS translation\n";
    my_file <<"    rotation    IS rotation\n";
    my_file <<"    children    IS children\n";
    my_file <<"  }\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"PROTO RangeSensor [\n";
    my_file <<"  exposedField SFVec3f    translation     0 0 0\n";
    my_file <<"  exposedField SFRotation rotation        0 0 1 0\n";
    my_file <<"  exposedField MFNode     children        [ ]\n";
    my_file <<"  exposedField SFInt32    sensorId        -1\n";
    my_file <<"  exposedField SFFloat    scanAngle       3.14159 #[rad]\n";
    my_file <<"  exposedField SFFloat    scanStep        0.1     #[rad]\n";
    my_file <<"  exposedField SFFloat    scanRate        10      #[Hz]\n";
    my_file <<"  exposedField SFFloat    maxDistance	    10\n";
    my_file <<"]\n";
    my_file <<"{\n";
    my_file <<"  Transform {\n";
    my_file <<"    translation IS translation\n";
    my_file <<"    rotation    IS rotation\n";
    my_file <<"    children    IS children\n";
    my_file <<"  }\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"PROTO Plane [\n";
    my_file <<"  exposedField SFVec3f size 10 10 0\n";
    my_file <<"]\n";
    my_file <<"{\n";
    my_file <<"  Box {\n";
    my_file <<"    size IS size\n";
    my_file <<"  }\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"PROTO ExtraJoint [\n";
    my_file <<"  exposedField SFString 	link1Name 		\"\"\n";
    my_file <<"  exposedField SFString 	link2Name 		\"\"\n";
    my_file <<"  exposedField SFVec3f  	link1LocalPos	0 0 0\n";
    my_file <<"  exposedField SFVec3f  	link2LocalPos	0 0 0\n";
    my_file <<"  exposedField SFString 	jointType 		\"xyz\"\n";
    my_file <<"  exposedField SFVec3f	jointAxis 		1 0 0\n";
    my_file <<"]\n";
    my_file <<"{\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"NavigationInfo {\n";
    my_file <<"  avatarSize    0.5\n";
    my_file <<"  headlight     TRUE\n";
    my_file <<"  type  [\"EXAMINE\" , \"ANY\"  ]\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"Background {\n";
    my_file <<"  skyColor 0.4 0.6 0.4\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"Viewpoint {\n";
    my_file <<"  position    3 0 0.835\n";
    my_file <<"  orientation 0.5770 0.5775 0.5775 2.0935\n";
    my_file <<"}\n";
    my_file <<"\n";
    my_file <<"DEF boxes_on_floor Humanoid{\n";
    my_file <<"  humanoidBody [\n";
    my_file <<"    DEF WAIST Joint {\n";
    my_file <<"      jointType \"fixed\"\n";
    my_file <<"      ulimit [0.0 0.0 ]\n";
    my_file <<"      llimit [0.0 0.0 ]\n";
    my_file <<"      children[\n";
    my_file <<"        DEF BODY Segment{\n";
    my_file <<"          centerOfMass 0.0 0.0 0.0\n";
    my_file <<"          mass 0.5 \n";
    my_file <<"          momentsOfInertia [ 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 ]\n";
    my_file <<"          children[\n";
    my_file <<"            Transform {\n";
    my_file <<"              children[\n";
    my_file <<"                Shape{\n";
    my_file <<"                  geometry IndexedFaceSet {\n";
    my_file <<"                  ccw TRUE\n";
    my_file <<"                  coord Coordinate {\n";
    my_file <<"                  point [\n";
    for(size_t i = 0; i < ground_->points.size();i++)
    {
        my_file <<ground_->points[i].x << " " << ground_->points[i].y<<" "<<ground_->points[i].z <<",\n";
    }
    my_file <<"]\n";
    my_file <<"               }\n";
    my_file <<"               coordIndex [\n";
    for(size_t i = 0; i < ground_->points.size();i++)
    {
        my_file << i << " ";
    }
    my_file << -1 << ",\n";
    my_file <<"]\n";
    my_file <<"normalPerVertex FALSE\n";
    my_file <<"normal Normal  {\n";
    my_file <<"vector [\n";
    my_file << normal_ground.x << " " << normal_ground.y << " " << normal_ground.z <<",\n";
    my_file <<"]\n";
    my_file <<"}\n";
    my_file <<"normalIndex [\n";
    my_file <<"  0 0                   ]\n";
    my_file <<"solid FALSE\n";
    my_file <<"}\n";
    my_file <<"appearance Appearance{\n";
    my_file <<"  material Material{\n";
    my_file <<"    diffuseColor 0.9 0.9 0.7\n";
    my_file <<"  }\n";
    my_file <<"}\n";
    my_file <<"}\n";
    my_file <<"]\n";
    my_file <<"}\n";
    for(size_t i = 0 ; i < planes_.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
        plane = planes_[i].getHull();
        Eigen::Vector3f normal_plane = planes_[i].getVector();
        writePlane(my_file,plane, normal_plane);
    }
//    my_file <<"Transform {\n";
//    my_file <<"children[\n";
//    my_file <<"Shape{\n";
//    my_file <<"geometry IndexedFaceSet {\n";
//    my_file <<"ccw TRUE\n";
//    my_file <<"coord Coordinate {\n";
//    my_file <<"point [\n";
//    my_file <<"1.25 0.2 0.02,\n";
//    my_file <<"1.25 0.0 0.02,\n";
//    my_file <<"1.35 0.2 0.02,\n";
//    my_file <<"1.35 0.0 0.02,\n";
//    my_file <<"]\n";
//    my_file <<"}\n";
//    my_file <<"coordIndex [\n";
//    my_file <<"1 0 2 -1,\n";
//    my_file <<"1 2 3 -1,\n";
//    my_file <<"]\n";
//    my_file <<"normalPerVertex FALSE\n";
//    my_file <<"normal Normal  {\n";
//    my_file <<"vector [\n";
//    my_file <<"0.0 0.0 1.0,\n";
//    my_file <<"]\n";
//    my_file <<"}\n";
//    my_file <<"normalIndex [\n";
//    my_file <<"  0 0\n";
//    my_file <<"]\n";
//    my_file <<"solid FALSE\n";
//    my_file <<"}\n";
//    my_file <<"appearance Appearance{\n";
//    my_file <<"  material Material{\n";
//    my_file <<"    diffuseColor 0.5 0.0 0.0\n";
//    my_file <<"  }\n";
//    my_file <<"}\n";
//    my_file <<"}\n";
//    my_file <<"]\n";
//    my_file <<"}\n";
    my_file <<"]\n";
    my_file <<"}\n";
    my_file <<"]\n";
    my_file <<"}\n";
    my_file <<"]\n";
    my_file <<"joints [\n";
    my_file <<"USE WAIST,\n";
    my_file <<"]\n";
    my_file <<"segments [\n";
    my_file <<"USE BODY,\n";
    my_file <<"]\n";
    my_file <<"}\n";
    my_file.close();


}
void getPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, cv::Mat &depth)
{

    cloud.reset(new  pcl::PointCloud<pcl::PointXYZRGBA>);
    for (unsigned int h = 0; h < depth.rows; h++)
    {
        for (unsigned int w = 0; w < depth.cols; w++)
        {
            //            point.z = 1024 * rand () / (RAND_MAX + 1.0f); // depthValue
            //            //point.z /= 1000;
            //            point.x = 1024 * rand () / (RAND_MAX + 1.0f) ;//(w - 319.5); //321.075 1024 * rand () / (RAND_MAX + 1.0f) //(w - 319.5) * point.z * rgbFocalInvertedX;
            //            point.y = 1024
            unsigned short depthValue;
            depthValue = depth.at<unsigned short>(h,w);
            //cv::Vec3f pt3D = _bgr.at<Vec3f>(h, w);
            pcl::PointXYZRGBA point;
            point.z = depthValue; // depthValue
            point.z /= 1000;
            point.x = (w - 319.5) * point.z * (1/525.0) ;//(w - 319.5); //321.075 1024 * rand () / (RAND_MAX + 1.0f) //(w - 319.5) * point.z * rgbFocalInvertedX;
            point.y = (h - 248.919) * point.z * (1/525.0); //248.919//(h - 239.5) * point.z * rgbFocalInvertedY
            point.r = 255;
            point.g = 255;
            point.b = 255;
            cloud->push_back(point);
            //std::cout << " point x " << point.x << "  " << point.y << " " << point.z << std::endl;
        }

    }
}


void computeRotationMatrix(Eigen::Matrix4f &matrix, Sophus::Sim3f &pose)
{
    matrix = Eigen::Matrix4f::Identity();

    matrix (0,0) = pose.rotationMatrix()(0,0); //cos (theta);
    matrix (0,1) = pose.rotationMatrix()(0,1);// -sin(theta)
    matrix (0,2) = pose.rotationMatrix()(0,2);
    matrix (1,0) = pose.rotationMatrix()(1,0); // sin (theta)
    matrix (1,1) = pose.rotationMatrix()(1,1); //cos (theta)
    matrix (1,2) = pose.rotationMatrix()(1,2);
    matrix (2,0) = pose.rotationMatrix()(2,0) ;
    matrix (2,1) = pose.rotationMatrix()(2,1);
    matrix (2,2) = pose.rotationMatrix()(2,2);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    matrix (0,3) = pose.translation()[0]; //2.5
    matrix (1,3) = pose.translation()[1]; //2.5
    matrix (2,3) = pose.translation()[2]; //2.5

}

void computeTransformation(Eigen::Matrix4f &transformation, Plane &plane1)
{
    pcl::PointXYZ normalXY;
    pcl::PointXYZ normalGround;
    pcl::PointXYZ result;
    normalXY.x = 0.0;
    normalXY.y = 0.0;
    normalXY.z = 1.0;

    double normalLength = sqrt((plane1.getCoefficients()->values[0] * plane1.getCoefficients()->values[0]) + (plane1.getCoefficients()->values[1] * plane1.getCoefficients()->values[1]) + (plane1.getCoefficients()->values[2] * plane1.getCoefficients()->values[2]));
    normalGround.x = plane1.getCoefficients()->values[0] / normalLength ;
    normalGround.y = plane1.getCoefficients()->values[1] / normalLength ;
    normalGround.z = plane1.getCoefficients()->values[2] / normalLength ;

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

    transformation = Eigen::Matrix4f::Identity();

    //    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    //    //float theta = M_PI/4; // The angle of rotation in radians

    transformation (0,0) = xx; //cos (theta);
    transformation (0,1) = xy;// -sin(theta)
    transformation (0,2) = xz;
    transformation (1,0) = yx; // sin (theta)
    transformation (1,1) = yy;
    transformation (1,2) = yz;
    transformation (2,0) = zx;
    transformation (2,1) = zy;
    transformation (2,2) = zz;
}
double determinant(pcl::PointXYZRGBA &pt1, pcl::PointXYZRGBA &pt2)
{
    return (pt1.x * pt2.y - pt1.y * pt2.x);

}
bool edgeIntersection(pcl::PointXYZRGBA &pt1, pcl::PointXYZRGBA &pt2, pcl::PointXYZRGBA &pt3, pcl::PointXYZRGBA &pt4)
{
    pcl::PointXYZRGBA pt21; pt21.x = pt2.x - pt1.x;pt21.y = pt2.y - pt1.y;
    pcl::PointXYZRGBA pt43; pt43.x = pt4.x - pt3.x;pt43.y = pt4.y - pt3.y;
    pcl::PointXYZRGBA pt31; pt31.x = pt3.x - pt1.x;pt31.y = pt3.y - pt1.y;;

    double det = determinant(pt21, pt43);
    double t = determinant(pt31, pt43)/det;
    double u = determinant(pt21, pt31)/det;
    if((t < 0) || (u < 0) || (t > 1) || (u > 1))
    {
        return false;
    }
    else
        return true;
}
bool checkSamePlane(Plane &plane1, Plane &plane2)
{
    Eigen::Matrix4f tr;
    Eigen::Matrix4f tr2;
    computeTransformation(tr,plane1);
    computeTransformation(tr2,plane2);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);

    cloud1_in = plane1.getHull();
    cloud2_in = plane2.getHull();
    std::cout << "Plane 1 " << cloud1_in->points.size()<< std::endl;
    std::cout << "Plane 2 " << cloud2_in->points.size()<< std::endl;

    pcl::transformPointCloud(*cloud1_in,*cloud1,tr);
    pcl::transformPointCloud(*cloud2_in,*cloud2,tr2);

    Eigen::Vector4f centroidPlane1;// = plane1.getCentroid();
    pcl::compute3DCentroid (*cloud1, centroidPlane1);
    Eigen::Vector4f centroidPlane2;// = plane2.getCentroid();
    pcl::compute3DCentroid (*cloud2, centroidPlane2);
    std::cout << "centroid " << centroidPlane1[2] << std::endl;
    std::cout << "centroid " << centroidPlane2[2] << std::endl;
    std::cout << "Type is " << plane1.getTYPE() << "  " << plane2.getTYPE() << std::endl;
    if((centroidPlane1[2] - centroidPlane2[2]) < 0.06&& plane1.getTYPE() == VERTICAL && plane2.getTYPE() == VERTICAL )
    {
        for(int i = 0; i < cloud1->points.size(); i++)
        {
            for(int j = 0; j < cloud2->points.size(); j++)
            {
                int k,l;
                k = i;
                l = j;
                if(k+1 >= cloud1->points.size())
                {
                    k = 0;
                }
                if(l+1 >= cloud2->points.size())
                {
                    l = 0;
                }
                if(edgeIntersection(cloud1->points[k],cloud1->points[k+1], cloud1->points[l],cloud1->points[l+1]))
                {
                    std::cout << "Intersect" << std::endl;
                    return true;
                }

            }
        }
        std::cout << "Not Intersect" << std::endl;
    }
    return false;
}
void addCloudAndGroundAndHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &gr,std::vector<Plane> &planes, osg::ref_ptr<osg::Geode> &node, Osg_gui &gui_, std::vector<osg::Vec3> &initColors)
{
    node = osg::ref_ptr<osg::Geode>(new osg::Geode());

    osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());
    osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
    osg::ref_ptr<osg::Vec4Array> colorsPc (new osg::Vec4Array());

    osg::ref_ptr<osg::Geometry> geometryP (new osg::Geometry());
    osg::ref_ptr<osg::Vec3Array> verticesP (new osg::Vec3Array());
    osg::ref_ptr<osg::Vec4Array> colorsPcP (new osg::Vec4Array());
    osg::Vec3 color(0, 255 , 0);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeXY(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for(int i = 0; i < 100 ; i++)
    {
        for(int j = 0; j < 100 ; j++)
        {
            pcl::PointXYZRGBA pt;
            pt.x = 0.1*i ;
            pt.y = 0.1*j;
            pt.z = 0.0 ;
            planeXY->push_back(pt);
        }
    }
    gui_.displayPointCloud(cloud , vertices, colorsPc ,color,MAIN_CLOUD );
    //gui_.displayPointCloud(planeXY , vertices, colorsPc ,color,MAIN_CLOUD );

    geometry->setVertexArray(vertices.get());
    geometry->setColorArray (colorsPc.get());
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,cloud->points.size()));

    if(planes.size() > 0)
    {
        for(int p = 0 ; p < planes.size() ; p++)
        {

            osg::ref_ptr<osg::Geometry> geometryG (new osg::Geometry());
            osg::ref_ptr<osg::Vec3Array> verticesG (new osg::Vec3Array());
            osg::ref_ptr<osg::Vec4Array> colorsPcG (new osg::Vec4Array());

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane2( new pcl::PointCloud<pcl::PointXYZRGBA>);
            plane2 = planes[p].getHull();
            //std::cout << "plane detected " << planes.size() << " " <<plane2->points.size()<< std::endl;
            gui_.displayPointCloud(plane2 , verticesG, colorsPcG ,initColors[p],PLANES );
            geometryG->setVertexArray(verticesG.get());
            geometryG->setColorArray (colorsPcG.get());
            geometryG->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            geometryG->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,plane2->points.size()));
            node->addDrawable(geometryG.get());
            osg::StateSet* stateGroundG = geometryG->getOrCreateStateSet();
            stateGroundG->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
        }
    }


    gui_.displayPointCloud(gr , verticesP , colorsPcP ,initColors[2],GROUND );
    geometryP->setVertexArray(verticesP.get());
    geometryP->setColorArray (colorsPcP.get());
    geometryP->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geometryP->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,gr->points.size()));


    //    float radius = 10.0f;
    //    osg::Vec3 center(0.0f,0.0f,0.0f -radius*0.5);
    //    int numTilesX = 10;
    //    int numTilesY = 10;

    //    float width = 2*radius;
    //    float height = 2*radius;

    //    osg::Vec3 v000(center - osg::Vec3(width*0.5f,height*0.5f,0.0f));
    //    osg::Vec3 dx(osg::Vec3(width/((float)numTilesX),0.0,0.0f));
    //    osg::Vec3 dy(osg::Vec3(0.0f,height/((float)numTilesY),0.0f));

    //    // fill in vertices for grid, note numTilesX+1 * numTilesY+1...
    //    osg::Vec3Array* coords = new osg::Vec3Array;
    //    int iy;
    //    for(iy=0;iy<=numTilesY;++iy)
    //    {
    //        for(int ix=0;ix<=numTilesX;++ix)
    //        {
    //            coords->push_back(v000+dx*(float)ix+dy*(float)iy);
    //        }
    //    }

    //    //Just two colours - black and white.
    //    osg::Vec4Array* colors = new osg::Vec4Array;
    //    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f)); // white
    //    colors->push_back(osg::Vec4(0.0f,0.0f,0.0f,1.0f)); // black

    //    osg::ref_ptr<osg::DrawElementsUShort> whitePrimitives = new osg::DrawElementsUShort(GL_QUADS);
    //    osg::ref_ptr<osg::DrawElementsUShort> blackPrimitives = new osg::DrawElementsUShort(GL_QUADS);

    //    int numIndicesPerRow=numTilesX+1;
    //    for(iy=0;iy<numTilesY;++iy)
    //    {
    //        for(int ix=0;ix<numTilesX;++ix)
    //        {
    //            osg::DrawElementsUShort* primitives = ((iy+ix)%2==0) ? whitePrimitives.get() : blackPrimitives.get();
    //            primitives->push_back(ix    +(iy+1)*numIndicesPerRow);
    //            primitives->push_back(ix    +iy*numIndicesPerRow);
    //            primitives->push_back((ix+1)+iy*numIndicesPerRow);
    //            primitives->push_back((ix+1)+(iy+1)*numIndicesPerRow);
    //        }
    //    }

    //    // set up a single normal
    //    osg::Vec3Array* normals = new osg::Vec3Array;
    //    normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));

    //    osg::Geometry* geom = new osg::Geometry;
    //    geom->setVertexArray(coords);

    //    geom->setColorArray(colors, osg::Array::BIND_PER_PRIMITIVE_SET);

    //    geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

    //    geom->addPrimitiveSet(whitePrimitives.get());
    //    geom->addPrimitiveSet(blackPrimitives.get());


    node->addDrawable(geometry.get());
    node->addDrawable(geometryP.get());
    osg::StateSet* stateGroundP = geometryP->getOrCreateStateSet();
    stateGroundP->setMode( GL_LIGHTING,osg::StateAttribute::OFF );
    //node->addDrawable(geom);
    node->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF );
}
void computeTransformationToOrigin(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & ground, Eigen::Vector3f &normalGround, Eigen::Matrix4f &transformXY, Eigen::Matrix4f &transformTranslationOrigin)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    *gr = *ground;
    //pcl::PointXYZ normalXY;
    //pcl::PointXYZ normalGround;
    //pcl::PointXYZ result;
    Eigen::Vector3f normalXY;
    Eigen::Vector3f result;
    normalXY[0] = 0.0;
    normalXY[1] = 0.0;
    normalXY[2] = 1.0;

    double normalLength = sqrt((normalGround[0] * normalGround[0]) + (normalGround[1] * normalGround[1]) + (normalGround[2] * normalGround[2]));
    normalGround[0] = normalGround[0] / normalLength ;
    normalGround[1] = normalGround[1] / normalLength ;
    normalGround[2] = normalGround[2] / normalLength ;

    result[0] = normalGround[1] * normalXY[2] - normalXY[1] * normalGround[2];
    result[1] = normalXY[0] * normalGround[2] - normalGround[0] * normalXY[2];
    result[2] = normalGround[0] * normalXY[1] - normalGround[1] * normalXY[0];
    double resultLenght = sqrt((result[0] * result[0]) + (result[1] * result[1]) +(result[2] * result[2]));
    result[0] /=resultLenght;
    result[1] /=resultLenght;
    result[2] /=resultLenght;

    double theta = std::acos(normalGround[2]/sqrt((normalGround[0]* normalGround[0])+(normalGround[1]*normalGround[1])+(normalGround[2] * normalGround[2])));
    //std::cout << "The crossproduct is " << result.x << " "<< result.y<< " "<< result.z <<std::endl;
    std::cout << "norm " << std::sqrt((result[0] * result[0]  )+(result[1] * result[1])+ (result[2] * result[2]))<< std::endl;

    double xx = (result[0] * result[0]) + ((1 - (result[0] * result[0])) * std::cos(theta));
    double xy = (result[0] * result[1])* (1 - std::cos(theta)) - (result[2] * std::sin(theta));
    double xz = (result[0] * result[2])* (1 - std::cos(theta)) + (result[1] * std::sin(theta));
    double yx = (result[0] * result[1])* (1 - std::cos(theta)) + (result[2] * std::sin(theta));
    double yy = (result[1] * result[1]) + ((1 - (result[1] * result[1])) * std::cos(theta));
    double yz = (result[1] * result[2])* (1 - std::cos(theta)) - (result[0] * std::sin(theta));
    double zx = (result[0] * result[2])* (1 - std::cos(theta)) - (result[1] * std::sin(theta));
    double zy = (result[1] * result[2])* (1 - std::cos(theta)) + (result[0] * std::sin(theta));
    double zz = (result[2] * result[2]) + ((1 - (result[2] * result[2])) * std::cos(theta));

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


    Eigen::Matrix4f transforminverse = Eigen::Matrix4f::Identity();

    transforminverse (0,0) = -1; //cos (theta);
    transforminverse (0,1) = 0;// -sin(theta)
    transforminverse (0,2) = 0;
    transforminverse (1,0) = 0; // sin (theta)
    transforminverse (1,1) = 1;
    transforminverse (1,2) = 0;
    transforminverse (2,0) = 0;
    transforminverse (2,1) = 0;
    transforminverse (2,2) = -1;
    pcl::transformPointCloud(*gr, *gr, transforminverse);
    transformXY = transforminverse*transformXY;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*gr, centroid);
    transformTranslationOrigin(0,3) = - centroid[0];
    transformTranslationOrigin(1,3) = - centroid[1];
    transformTranslationOrigin(2,3) = - centroid[2];

}
void normalizeVector(Eigen::Vector3f &vector)
{
    float length = std::sqrt( std::pow(vector(0,0),2) + (std::pow(vector(1,0),2)) +  (std::pow(vector(2,0),2)));
    vector(0,0) = vector(0,0)/length;
    vector(1,0) = vector(1,0)/length;
    vector(2,0) = vector(2,0)/length;
}

void pointCloudToPointNormal(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &cloud2)
{
    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        pcl::PointNormal point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        cloud2->push_back(point);
    }
}

void pointNormalToPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &cloud2)
{
    for(size_t i = 0; i < cloud2->points.size(); i++)
    {
        pcl::PointXYZRGBA point;
        point.x = cloud2->points[i].x;
        point.y = cloud2->points[i].y;
        point.z = cloud2->points[i].z;
        cloud->push_back(point);
    }
}

void setLocalizationMatrix(Eigen::Matrix4f &transform, Sophus::Sim3f &poseSaved)
{
    transform (0,0) = poseSaved.rotationMatrix()(0,0); //cos (theta);
    transform (0,1) = poseSaved.rotationMatrix()(0,1);// -sin(theta)
    transform (0,2) = poseSaved.rotationMatrix()(0,2);
    transform (1,0) = poseSaved.rotationMatrix()(1,0); // sin (theta)
    transform (1,1) = poseSaved.rotationMatrix()(1,1); //cos (theta)
    transform (1,2) = poseSaved.rotationMatrix()(1,2);
    transform (2,0) = poseSaved.rotationMatrix()(2,0) ;
    transform (2,1) = poseSaved.rotationMatrix()(2,1);
    transform (2,2) = poseSaved.rotationMatrix()(2,2);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform (0,3) = poseSaved.translation()[0]; //2.5
    transform (1,3) = poseSaved.translation()[1]; //2.5
    transform (2,3) = poseSaved.translation()[2]; //2.5

}

void EvaluatePlanes(std::vector<Plane> &pl, std::vector<Plane> & _planes_cloud, bool &planeDetected,const Eigen::Matrix4f &transform_1, const Eigen::Matrix4f &_transformationICP, const Eigen::Matrix4f &_transformationXYPlane, const Eigen::Matrix4f &_transformationXYTranslation)
{


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
        //pcl::transformPointCloud (*centroidTransf, *centroidTransf, _transformationXYPlane);
        //pcl::transformPointCloud (*centroidTransf, *centroidTransf, _transformationXYTranslation);

        Eigen::Vector4f height = pl[p].getHeight();

        plTransf = pl[p].getHull();
        pcl::transformPointCloud (*plTransf, *plTransf, transform_1);
        pcl::transformPointCloud (*plTransf, *plTransf, _transformationICP);
        //pcl::transformPointCloud (*plTransf, *plTransf, _transformationXYPlane);
        //pcl::transformPointCloud (*plTransf, *plTransf, _transformationXYTranslation);

        pcl::PointCloud<pcl::PointXYZ>::Ptr coeff(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients(new  pcl::ModelCoefficients);
        coefficients = pl[p].getCoefficients();
        pcl::PointXYZ coeffPoint;
        coeffPoint.x = coefficients->values[0];
        coeffPoint.y = coefficients->values[1];
        coeffPoint.z = coefficients->values[2];
        coeff->push_back(coeffPoint);
        pcl::transformPointCloud (*coeff, *coeff, transform_1);
        //pcl::transformPointCloud (*coeff, *coeff, _transformationICP);
        //pcl::transformPointCloud (*coeff, *coeff, _transformationXYPlane);

        coefficients->values[0] = coeff->points[0].x;
        coefficients->values[1] = coeff->points[0].y;
        coefficients->values[2] = coeff->points[0].z;

        Eigen::Vector3f vectorNormalPlane = pl[p].getVector();
        vectorNormalPlane = transform_1.block(0,0,3,3) * vectorNormalPlane;
        vectorNormalPlane = _transformationICP.block(0,0,3,3) * vectorNormalPlane;
        //vectorNormalPlane = _transformationXYPlane.block(0,0,3,3) * vectorNormalPlane;

        // new plane
        Plane plane;
        plane.setCentroid(centroid);
        plane.setHull(plTransf);
        plane.setHeight(height);
        plane.setCoefficients(coefficients);
        plane.setVectorGround(vectorNormalPlane);
        plane.setType(pl[p].getTYPE());
        Eigen::Vector4f centroid_(centroidTransf->points[0].x,centroidTransf->points[0].y,centroidTransf->points[0].z, 0.0 );
        plane.setCentroidTrans(centroid_);

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

                Eigen::Vector4f centroidPrev = _planes_cloud[j].getCentroidTrans();

                pcl::ModelCoefficients::Ptr coeff_prev(new  pcl::ModelCoefficients);
                coeff_prev = _planes_cloud[j].getNormalizedCoefficients();

                // check the position of the centroid
                if((computeDistanceCentroid(centroid_, centroidPrev, 0.15 ) == true))
                {
                    _planes_cloud[j].setHeight(height);
                    _planes_cloud[j].setCentroidTrans(centroid_);
                    _planes_cloud[j].concatenateCloud(plTransf); // concatenate Hull
                    break;
                }
                else
                {
                    // std::cout << "Add new plane !!" << std::endl;
                    //double diff = std::abs(centroid_[2]) - std::abs(centroidPrev[2]);
                }
            }
        }
    }
}
void postProcessingGround(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &ground, Eigen::Vector3f &normalGround, const Eigen::Matrix4f &_transformationXYPlane,const Eigen::Matrix4f &_transformationXYTranslation)
{
    // Transform ground and normal
    pcl::transformPointCloud (*ground, *ground, _transformationXYPlane);
    pcl::transformPointCloud (*ground, *ground, _transformationXYTranslation);
    normalGround = _transformationXYPlane.block(0,0,3,3) * normalGround;
    normalizeVector(normalGround);
}

void postProcessingPlanes(std::vector<Plane> &planes_cloud,const Eigen::Matrix4f &_transformationXYPlane,const Eigen::Matrix4f &_transformationXYTranslation)
{
    std::vector<Plane> planes;
    planes.clear();
    for(int i = 0 ; i < planes_cloud.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plTransf(new pcl::PointCloud<pcl::PointXYZRGBA>);
        plTransf = planes_cloud[i].getHull();

        pcl::transformPointCloud (*plTransf, *plTransf, _transformationXYPlane);
        pcl::transformPointCloud (*plTransf, *plTransf, _transformationXYTranslation);

        Eigen::Vector3f vectorNormalPlane = planes_cloud[i].getVector();
        vectorNormalPlane = _transformationXYPlane.block(0,0,3,3) * vectorNormalPlane;
        Plane plane;
        plane.setHull(plTransf);
        plane.setVectorGround(vectorNormalPlane);
        planes.push_back(plane);
    }

    planes_cloud.clear();
    planes_cloud = std::vector<Plane>();

    planes_cloud = planes;


}
