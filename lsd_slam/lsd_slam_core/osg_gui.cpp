#include "osg_gui.h"


Osg_gui::Osg_gui()
{
    _root = new osg::Group();
    _viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    _viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    _update = false;
    _isStopped = false;
    _firstFrame = false;
    _stopMainThread = false;
}

Osg_gui::~Osg_gui()
{
    // delete root;
}
void Osg_gui::displayPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, osg::ref_ptr<osg::Vec3Array> &vertices, osg::ref_ptr<osg::Vec4Array> &colors,  osg::Vec3 &colorCluster, TYPES_POINTCLOUD c)
{

    for (unsigned int i = 0; i < cloud->points.size(); i++)
    {
        vertices->push_back (osg::Vec3 (cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
        uint32_t rgb_val_;
        memcpy(&rgb_val_, &(cloud->points[i].rgb), sizeof(uint32_t));

        uint32_t red,green,blue;
        blue = rgb_val_ & 0x000000ff;
        rgb_val_ = rgb_val_ >> 8;
        green = rgb_val_ & 0x000000ff;
        rgb_val_ = rgb_val_ >> 8;
        red = rgb_val_ & 0x000000ff;
        if (c == MAIN_CLOUD)
        {
            //colors->push_back (osg::Vec4f ((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f));
            colors->push_back (osg::Vec4f ((float)255/255.0f, (float)255/255.0f, (float)255/255.0f,1.0f));
        }
        else if (c == PLANES)
        {
            colors->push_back (osg::Vec4f ((float)colorCluster[0]/255.0f, (float)colorCluster[1]/255.0f,(float) colorCluster[2]/255.0f,1.0f));
        }
//        else if(c == 2)
//        {
//            colors->push_back (osg::Vec4f ((float)colorCluster[0]/255.0f, (float)colorCluster[1]/255.0f,(float) colorCluster[2]/255.0f,1.0f));

//        }
        else
        {
            colors->push_back (osg::Vec4f ((float)1.0, (float)0.0,(float) 0.0,1.0f));
        }
        //colors->push_back (osg::Vec4f ((float)255/255.0f, (float)255/255.0f, (float)255/255.0f,1.0f));

    }


}
void Osg_gui::displayGround(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, osg::ref_ptr<osg::Vec3Array> &vertices, osg::ref_ptr<osg::Vec4Array> &colors,  osg::Vec3 &colorCluster)
{

    for (unsigned int i = 0; i < cloud->points.size(); i++)
    {
        vertices->push_back (osg::Vec3 (cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
        //        uint32_t rgb_val_;
        //    /*    memcpy(&rgb_val_, &(cloud->points[i].rgb), sizeof(uint32_t));

        //        uint32_t red,green,blue;
        //        blue = rgb_val_ & 0x000000ff;
        //        rgb_val_ = rgb_val_ >> 8;
        //        green = rgb_val_ & 0x000000ff;
        //        rgb_val_ = rgb_val_ >> 8;
        //        red = rgb_val_ & 0x000000ff*/;

        colors->push_back (osg::Vec4f ((float)255/255.0f, (float)0/255.0f, (float)0/255.0f,1.0f));

    }


}
void Osg_gui::setUpdate(bool update)
{
    _update = update;
}
bool Osg_gui::getUpdate() const
{
    return _update;
}
void Osg_gui::setGeode(osg::ref_ptr<osg::Geode> geodeToSet)
{
    _geode = geodeToSet;
}
void Osg_gui::setRoot()
{
    _root->addChild(_geode);
    _viewer.setSceneData( _root );
}
void Osg_gui::clearRoot()
{
    _root->removeChild(_geode);
}
osgViewer::Viewer Osg_gui::getViewer()
{
    return _viewer;
}
bool Osg_gui::getBoolIsStopped() const
{
    return _isStopped;
}
bool Osg_gui::getStopMainThread() const
{
    return _stopMainThread;
}
void Osg_gui::run()
{
    KeyHandler* keyEvent = new KeyHandler();
    _viewer.addEventHandler(keyEvent);
    _viewer.realize();

    while (!_viewer.done())
    {
        if (_update == true && _geode != NULL)
        {
            _root->addChild(_geode);
            if (_firstFrame == false)
            {
                _firstFrame = true;
                _viewer.setSceneData( _root );
            }
        }

        _update = false;
        _viewer.frame();
        if(keyEvent->getStopWriting())
        {
            _stopMainThread = true;
        }
        //boost::this_thread::sleep (boost::posix_time::seconds (0.5));
    }
    std::cout << "osg GUI stopped" << std::endl;
    _isStopped = true;
}
