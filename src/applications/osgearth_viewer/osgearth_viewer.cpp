/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Graticule>
#include <osgEarthUtil/SkyNode>

using namespace osgEarth::Util;
#define USE_EXTRA_CAMERA

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << "USAGE: osgearth_viewer [--graticule] [--autoclip] file.earth" << std::endl;
    OE_NOTICE << "   --graticule     : displays a lat/long grid in geocentric mode" << std::endl;
    OE_NOTICE << "   --sky           : activates the atmospheric model" << std::endl;
    OE_NOTICE << "   --animateSky    : animates the sun across the sky" << std::endl;
    OE_NOTICE << "   --autoclip      : activates the auto clip-plane handler" << std::endl;
        
    return -1;
}

struct AnimateSunCallback : public osg::NodeCallback
{
    void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        SkyNode* skyNode = static_cast<SkyNode*>(node);
        double hours = fmod( osg::Timer::instance()->time_s()/4.0, 24.0 );
        skyNode->setDateTime( 2011, 6, 6, hours );
        OE_INFO << "TIME: " << hours << std::endl;
    }
};

struct ToggleStereoHandler : public osgGA::GUIEventHandler 
{
	bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
	{
		if ( ea.getEventType() == ea.KEYDOWN )
		{
			if(ea.getKey() == 'b')
			{
				bool stereo = osg::DisplaySettings::instance()->getStereo();
				osg::DisplaySettings::instance()->setStereo(stereo ? false : true);
			}
		}
		return false;
	}
};


// Callback that checks other cameras
int s_skyDomePreRenderCameraNum = -3;

#ifdef _WIN32
#define FAST_EXTERNAL_DIR "/work/trac/external"
#define FAST_TEST_DIR "/work/trac/test"
#else
#define FAST_EXTERNAL_DIR "/local/work/fastprotect/external"
#define FAST_TEST_DIR "/local/work/fastprotect/test"
#endif
#define FAST_OSG_DATA_DIR FAST_EXTERNAL_DIR "/OpenSceneGraph/data/"

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    bool useGraticule = arguments.read( "--graticule" );
    bool useAutoClip  = arguments.read( "--autoclip" );
    bool animateSky   = arguments.read( "--animateSky");
    bool useSky       = arguments.read( "--sky" ) || animateSky;

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
        return usage( "Unable to load earth model." );

    osgViewer::Viewer viewer(arguments);

    osg::Group* root = new osg::Group();

    root->addChild( earthNode );
	osg::Node * cow = osgDB::readNodeFile(FAST_OSG_DATA_DIR "cow.osg");
	root->addChild( cow );

    // create a graticule and clip plane handler.
    Graticule* graticule = 0L;
    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if ( mapNode )
    {
        if ( mapNode->getMap()->isGeocentric() )
        {
            // the AutoClipPlaneHandler will automatically adjust the near/far clipping
            // planes based on your view of the horizon. This prevents near clipping issues
            // when you are very close to the ground. If your app never brings a user very
            // close to the ground, you may not need this.
            if ( useAutoClip )
                viewer.addEventHandler( new AutoClipPlaneHandler );

            // the Graticule is a lat/long grid that overlays the terrain. It only works
            // in a round-earth geocentric terrain.
            if ( useGraticule )
            {
                graticule = new Graticule( mapNode->getMap() );
                root->addChild( graticule );
            }

            if ( useSky )
            {
                SkyNode* sky = new SkyNode( mapNode->getMap() );
                sky->setDateTime( 2011, 1, 6, 17.0 );
                //sky->setSunPosition( osg::Vec3(0,-1,0) );
                sky->attach( &viewer );
#ifdef USE_EXTRA_CAMERA
				osg::Camera* mainCamera = viewer.getCamera();
				// Set the camera to not clear the color buffer.
				if (mainCamera->getClearMask() & GL_COLOR_BUFFER_BIT)
					mainCamera->setClearMask(mainCamera->getClearMask() & ~GL_COLOR_BUFFER_BIT);

				osg::Camera* camera = new osg::Camera;
				// set to -100 by default
				camera->setRenderOrder(osg::Camera::PRE_RENDER, s_skyDomePreRenderCameraNum);
				// Hopefully this will be the first pre-render camera, so it should
				// clear the color and depth buffers (this skydome is not guaranteed
				// to fill the whole view at all times).
				camera->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
				// The dome hangs under m_skyTransform.
				camera->addChild(sky);

				// Make sure the main viewer camera only clears the depth buffer
				// otherwise it will clear our skydome...
				//root->setCullCallback(new CheckCamerasCallback);
				root->addChild( camera );
#else
                root->addChild( sky );
#endif
                if (animateSky)
                {
                    sky->setUpdateCallback( new AnimateSunCallback());
                }
            }
        }
    }

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    viewer.setSceneData( root );

    EarthManipulator* manip = new EarthManipulator();
	//osgGA::TrackballManipulator* manip = new osgGA::TrackballManipulator();
	//manip->setNode(cow);
    viewer.setCameraManipulator( manip );

	// add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));
	viewer.addEventHandler(new ToggleStereoHandler());

    return viewer.run();
}
