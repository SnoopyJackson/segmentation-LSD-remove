#include "osg_gui.h"

/*
* Default Constructor.
*/
Osg_gui::Osg_gui()
{
    _root = new osg::Group();
    _viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    _viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    _update = false;
    _isStopped = false;
    _firstFrame = false;
    _stopMainThread = false;
    tx = 0.0;
    ty = 0.0;
    tz = 0.0;
}
/*
* Destructor.
*/
Osg_gui::~Osg_gui()
{
    // delete root;
}
/*
* Set Translation parameters.
* params[in]: x, y, z
*/
void Osg_gui::setTranslation(double x, double y , double z)
{
    tx = x;
    ty = y;
    tz = z;
}

/*
* Display point cloud.
* params[in]: cloud, vertices, colors, colorCLuster, types
*/
void Osg_gui::displayPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, osg::ref_ptr<osg::Vec3Array> &vertices, osg::ref_ptr<osg::Vec4Array> &colors,  osg::Vec3 &colorCluster, TYPES_POINTCLOUD c)
{

    for (unsigned int i = 0; i < cloud->points.size(); i++)
    {
        vertices->push_back (osg::Vec3 (cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
//        uint32_t rgb_val_;
//        memcpy(&rgb_val_, &(cloud->points[i].rgb), sizeof(uint32_t));

//        uint32_t red,green,blue;
//        blue = rgb_val_ & 0x000000ff;
//        rgb_val_ = rgb_val_ >> 8;
//        green = rgb_val_ & 0x000000ff;
//        rgb_val_ = rgb_val_ >> 8;
//        red = rgb_val_ & 0x000000ff;
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
/*
* Set update bool.
* params[in]: update bool
*/
void Osg_gui::setUpdate(bool update)
{
    _update = update;
}
/*
* Get update bool.
* return update bool
*/
bool Osg_gui::getUpdate() const
{
    return _update;
}
/*
* Set Geode.
* params[in]: Geode to set
*/
void Osg_gui::setGeode(osg::ref_ptr<osg::Geode> geodeToSet)
{
    _geode = geodeToSet;
}
/*
* Set Root.
*/
void Osg_gui::setRoot()
{
    _root->addChild(_geode);
    _viewer.setSceneData( _root );
}
/*
* Clear root.
*/
void Osg_gui::clearRoot()
{
    _root->removeChild(_geode);
}
/*
* Get viewer.
* params[out]: get Viewer
*/
osgViewer::Viewer Osg_gui::getViewer()
{
    return _viewer;
}

/*
* Get the isStopped bool.
* return the isStopped bool
*/
bool Osg_gui::getBoolIsStopped() const
{
    return _isStopped;
}
/*
* Get stopMainThread bool.
* return stopMainThread bool
*/
bool Osg_gui::getStopMainThread() const
{
    return _stopMainThread;
}
/*
* Create 3 Axis X, Y, Z.
* return the Geometry
*/
osg::Geometry* createAxis()
{
    osg::Geometry* pyramidGeometry = new osg::Geometry();
    osg::Vec3Array* pyramidVertices = new osg::Vec3Array;
    pyramidVertices->push_back( osg::Vec3( 0, 0, 0) ); // front left
    pyramidVertices->push_back( osg::Vec3(1, 0, 0) ); // front right
    pyramidVertices->push_back( osg::Vec3(0,1, 0) ); // back right
    pyramidVertices->push_back( osg::Vec3( 0,0, 1) ); // back left
    pyramidGeometry->setVertexArray( pyramidVertices );
    osg::DrawElementsUInt* AxeX = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);
    AxeX->push_back(1);
    AxeX->push_back(0);
    pyramidGeometry->addPrimitiveSet(AxeX);
    osg::DrawElementsUInt* AxeY = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);
    AxeY->push_back(2);
    AxeY->push_back(0);
    pyramidGeometry->addPrimitiveSet(AxeY);
    osg::DrawElementsUInt* AxeZ = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);
    AxeZ->push_back(3);
    AxeZ->push_back(0);
    pyramidGeometry->addPrimitiveSet(AxeZ);
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) ); //index 0 red
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) ); //index 1 green
    colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) ); //index 2 blue
    colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); //index 3 white

    pyramidGeometry->setColorArray(colors);
    pyramidGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    return pyramidGeometry;
}
/*
* Run Function.
*/
void Osg_gui::run()
{
    KeyHandler* keyEvent = new KeyHandler();
    _viewer.addEventHandler(keyEvent);
    _viewer.realize();

    while (!_viewer.done())
    {
        if (_update == true && _geode != NULL)
        {
//            osg::Geometry* geo =  createAxis();
//             osg::Geometry* geo2 =  createAxis();
//             osg::Geode* pyramidGeode = new osg::Geode();
//             pyramidGeode->addDrawable(geo);
//             _root->addChild(pyramidGeode);
//            //_geode->addDrawable(geo);
//           _geode->addDrawable(geo2);
//              osg::PositionAttitudeTransform* pyramidTwoXForm =
//                 new osg::PositionAttitudeTransform();

//           // Use the 'addChild' method of the osg::Group class to
//           // add the transform as a child of the root node and the
//           // pyramid node as a child of the transform.
//              _root->addChild(pyramidTwoXForm);
//              pyramidTwoXForm->addChild(pyramidGeode);

//           // Declare and initialize a Vec3 instance to change the
//           // position of the tank model in the scene
//              osg::Vec3 pyramidTwoPosition(tx,ty,tz);
//              pyramidTwoXForm->setPosition( pyramidTwoPosition );
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
