#ifndef OSG_GUI_H_
#define OSG_GUI_H_

//#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/thread/thread.hpp>

#include <osg/Node>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/GUIEventHandler>



enum TYPES_POINTCLOUD
{
    MAIN_CLOUD = 0,
    PLANES = 1,
    GROUND = 2
};
class KeyHandler : public osgGA::GUIEventHandler
{
public:
    bool _stopWriting;
    KeyHandler()
    {
        _stopWriting = false;
    }

public:
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
    {
        switch(ea.getEventType())
        {
        case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            switch(ea.getKey())
            {
            case 'w':
                std::cout << " stop main Thread " << std::endl;
                _stopWriting = true;
                return false;
                break;
            default:
                return false;
            }

        }

        default:
            return false;
        }
    }
    bool getStopWriting(){ return _stopWriting;}
    virtual void accept(osgGA::GUIEventHandlerVisitor& v)
    {
        v.visit(*this);
    }

};
class Osg_gui
{
public:
    Osg_gui();
    ~Osg_gui();
    void displayPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, osg::ref_ptr<osg::Vec3Array> &vertices, osg::ref_ptr<osg::Vec4Array> &colors,  osg::Vec3 &colorCluster, TYPES_POINTCLOUD c);
    void displayGround(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, osg::ref_ptr<osg::Vec3Array> &vertices, osg::ref_ptr<osg::Vec4Array> &colors,  osg::Vec3 &colorCluster);

    void run();
    void setUpdate(bool update);
    void setGeode(osg::ref_ptr<osg::Geode> geodeToSet);
    void clearRoot();
    void setRoot();
    osgViewer::Viewer getViewer();
    bool getUpdate() const;
    bool getBoolIsStopped() const;
    bool getStopMainThread() const;
private:
    osgViewer::Viewer _viewer;
    osg::ref_ptr<osg::Group> _root;
    //osg::Group* root;
    osg::ref_ptr<osg::Geode> _geode;
    bool _update;
    bool _isStopped;
    bool _firstFrame;
    bool _stopMainThread;

};
#endif /* GUI_H_ */
