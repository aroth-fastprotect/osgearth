/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/AutoTransform>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/MapNode>
#include <osgEarth/FindNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ElevationManager>
#include <osgEarthDrivers/tms/TMSOptions>
#include <sstream>

#include <osgDB/ReaderWriter>
#include <osgDB/WriteFile>
#include <osgGA/FirstPersonManipulator>
#include <osgEarth/Registry>

using namespace osgEarth::Drivers;

static
osg::MatrixTransform* createFlag()
{
    osg::Cylinder* c = new osg::Cylinder( osg::Vec3d(0,0,0), 2.0f, 250.f );
    osg::Geode* g = new osg::Geode();
    g->addDrawable( new osg::ShapeDrawable( c ) );
    osgText::Text* text = new osgText::Text();
    text->setCharacterSizeMode( osgText::Text::SCREEN_COORDS );
    text->setCharacterSize( 72.f );
    text->setBackdropType( osgText::Text::OUTLINE );
    text->setText( "00000000000000" );
    text->setAutoRotateToScreen( true );
    text->setPosition( osg::Vec3d( 0, 0, 125 ) );
    text->setDataVariance( osg::Object::DYNAMIC );
    g->addDrawable( text );
    osg::AutoTransform* at = new osg::AutoTransform();
    at->setAutoScaleToScreen( true );
    at->addChild( g );
    at->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
    osg::MatrixTransform* xf = new osg::MatrixTransform();
    xf->addChild( at );
    xf->setDataVariance( osg::Object::DYNAMIC );
    return xf;
}

static void
updateFlag( osg::MatrixTransform* xf, const osg::Matrix& mat, double lat, double lon, double elev )
{
    osg::Geode* g = static_cast<osg::Geode*>( xf->getChild(0)->asGroup()->getChild(0) );
    std::stringstream buf;
	buf << lat << ", " << lon << '\n' << elev;
	std::string bufStr;
	bufStr = buf.str();
    static_cast<osgText::Text*>( g->getDrawable(1) )->setText( bufStr );
    xf->setMatrix( mat );
}

// An event handler that will print out the elevation at the clicked point
struct QueryElevationHandler : public osgGA::GUIEventHandler 
{
    QueryElevationHandler( osgEarthUtil::ElevationManager* elevMan, osg::MatrixTransform* flag )
        :_elevMan(elevMan), _flag(flag), _mouseDown(false) { }

    void update( float x, float y, osgViewer::View* view )
    {
        osgUtil::LineSegmentIntersector::Intersections results;
        if ( view->computeIntersections( x, y, results, 0x01 ) )
        {
            // find the first hit under the mouse:
            osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
            osg::Vec3d point = first.getWorldIntersectPoint();
            
            // transform it to map coordinates:
            const SpatialReference* srs = _elevMan->getMap()->getProfile()->getSRS();
            double lat_rad, lon_rad, height;
            srs->getEllipsoid()->convertXYZToLatLongHeight( point.x(), point.y(), point.z(), lat_rad, lon_rad, height );
            
            // query the elevation at the map point:
            double lat_deg = osg::RadiansToDegrees( lat_rad );
            double lon_deg = osg::RadiansToDegrees( lon_rad );
            osg::Matrixd out_mat;
            double query_resolution = 0.1; // 1/10th of a degree
            double out_elevation = 0.0;
            double out_resolution = 0.0;

            if ( _elevMan->getPlacementMatrix(
                lon_deg, lat_deg, 0,
                query_resolution, NULL,
                out_mat, out_elevation, out_resolution ) )
            {
                updateFlag( _flag.get(), out_mat, lat_deg, lon_deg, out_elevation );
            }
            else
            {
                OE_NOTICE
                    << "getElevation FAILED! at (" << lat_deg << ", " << lon_deg << ")" << std::endl;
            }

        }
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
		osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
        if ( ea.getEventType() == osgGA::GUIEventAdapter::MOVE )
        {
            update( ea.getX(), ea.getY(), view );
        }
		else if(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		{
			switch(ea.getKey())
			{
			case 'e': 
				{
					update( ea.getX(), ea.getY(), view );
				}
				break;
			}
		}

        return false;
    }

    bool _mouseDown;
    osg::ref_ptr<osgEarthUtil::ElevationManager> _elevMan;
    osg::ref_ptr<osg::MatrixTransform> _flag;
};

osg::Vec3d toGPSCoordinate(const osg::EllipsoidModel & ellipsoid, const osg::Vec3d & v)
{
	osg::Vec3d ret;
	double lon;
	double lat;
	ellipsoid.convertXYZToLatLongHeight(v.x(), v.y(), v.z(), lat, lon, ret.z());
	ret.x() = osg::RadiansToDegrees(lat);
	ret.y() = osg::RadiansToDegrees(lon);
	return ret;
}

bool writeHeightField(const osg::EllipsoidModel & ellipsoid, const GeoExtent& extent, const osg::HeightField * hf, const std::string & filename)
{
	if(!hf)
		return false;

	std::ofstream file(filename.c_str());


	osg::Vec3d origin = toGPSCoordinate(ellipsoid, hf->getOrigin());
	double xmin, ymin, xmax, ymax;
	extent.getBounds(xmin, ymin, xmax, ymax);

	file << "ncols " << hf->getNumColumns() << std::endl;
	file << "nrows " << hf->getNumRows() << std::endl;

	file << "xllcorner " << xmin << std::endl;
	file << "yllcorner " << ymin << std::endl;
	file << "cellsize " << hf->getXInterval() << std::endl;
	file << "nodata_value -9999" << std::endl;

	unsigned numRows = hf->getNumRows();
	for(unsigned r = 0; r < numRows; r++)
	{
		for(unsigned c = 0; c < hf->getNumColumns(); c++)
		{
			float elev = hf->getHeight(c, numRows - r - 1);
			file << elev << ' ';
		}
		file << std::endl;
	}
	file.close();
	return true;
}

class MyReadFileCallback : public osgDB::ReadFileCallback
{
public:

	virtual osgDB::ReaderWriter::ReadResult openArchive(const std::string& filename,osgDB::ReaderWriter::ArchiveStatus status, unsigned int indexBlockSizeHint, const osgDB::Options* useObjectCache)
	{
		std::cout << "openArchive " << filename << std::endl;
		return osgDB::ReadFileCallback::openArchive(filename, status, indexBlockSizeHint, useObjectCache);
	}

	virtual osgDB::ReaderWriter::ReadResult readObject(const std::string& filename, const osgDB::Options* options)
	{
		std::cout << "readObject " << filename << std::endl;
		return osgDB::ReadFileCallback::readObject(filename, options);
	}

	virtual osgDB::ReaderWriter::ReadResult readImage(const std::string& filename, const osgDB::Options* options)
	{
		std::cout << "readImage " << filename << std::endl;
		return osgDB::ReadFileCallback::readImage(filename, options);
	}

	virtual osgDB::ReaderWriter::ReadResult readHeightField(const std::string& filename, const osgDB::Options* options)
	{
		std::cout << "readHeightField " << filename << std::endl;
		return osgDB::ReadFileCallback::readHeightField(filename, options);
	}

	virtual osgDB::ReaderWriter::ReadResult readNode(const std::string& filename, const osgDB::Options* options)
	{
		std::cout << "readNode " << filename << std::endl;
		return osgDB::ReadFileCallback::readNode(filename, options);
	}

	virtual osgDB::ReaderWriter::ReadResult readShader(const std::string& filename, const osgDB::Options* options)
	{
		std::cout << "readShader " << filename << std::endl;
		return osgDB::ReadFileCallback::readShader(filename, options);
	}

protected:
	virtual ~MyReadFileCallback() {}
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

	osgDB::Registry::instance()->setReadFileCallback(new MyReadFileCallback);

    osgViewer::Viewer viewer(arguments);

    // install the programmable manipulator.
    osgEarthUtil::EarthManipulator* manip = new osgEarthUtil::EarthManipulator();
	//osgGA::FirstPersonManipulator * manip = new osgGA::FirstPersonManipulator();
    viewer.setCameraManipulator( manip );

	osgEarth::MapNode* mapNode = NULL;

	osg::Node* loadedNode = osgDB::readNodeFiles( arguments );
	if (!loadedNode)
	{
		// load up a map with an elevation layer:
		osgEarth::Map *map = new osgEarth::Map();

		// Add some imagery
		{
			TMSOptions* tms = new TMSOptions();
			tms->url() = "http://demo.pelicanmapping.com/rmweb/data/bluemarble-tms/tms.xml";
			map->addMapLayer( new osgEarth::ImageMapLayer( "BLUEMARBLE", tms ) );
		}

		// Add some elevation
		{
			TMSOptions* tms = new TMSOptions();
			tms->url() = "http://demo.pelicanmapping.com/rmweb/data/srtm30_plus_tms/tms.xml";
			map->addMapLayer( new osgEarth::HeightFieldMapLayer( "SRTM", tms ) );
		}
		mapNode = new osgEarth::MapNode( map );
	}
	else
	{
		mapNode = findTopMostNodeOfType<osgEarth::MapNode>( loadedNode );
	}
	osgEarth::Map *map = mapNode->getMap();

	map->getImageMapLayers();

	// save the current osgEarth configuration to a file for checking
	EarthFile earthFile(map, mapNode->getMapEngineProperties());
	earthFile.writeXML("C:\\tmp\\elev1.earth");


	mapNode->getEllipsoidModel();

	osg::ref_ptr<TileKey> key;
	osg::ref_ptr<osg::HeightField> hf;
	osg::ref_ptr<osgTerrain::TerrainTile> tile;

	double x = 11.566667, y = 48.133333;
	for(int level = 0; level < 12; level++)
	{
		char sz[256];
		sprintf(sz, "C:\\tmp\\munich_%i.asc", level);

		// get the tilekey corresponding to the tile we need:
		key = map->getProfile()->createTileKey( x, y, level );
		const GeoExtent& extent = key->getGeoExtent();

		hf = map->createHeightField( key.get(), true, osgEarth::INTERP_BILINEAR);

		if(hf.valid())
		{
			double map_x = x, map_y = y;
			double xInterval = extent.width()  / (double)(hf->getNumColumns()-1);
			double yInterval = extent.height() / (double)(hf->getNumRows()-1);

			double out_elevation = (double) HeightFieldUtils::getHeightAtLocation( hf.get(), map_x, map_y, extent.xMin(), extent.yMin(), xInterval, yInterval );

			std::cerr << "createTileKey(" << x << ", " << y << " , " << level << ") " << key->str() << " " 
				<< extent.toString() << " elev " << out_elevation << std::endl;

			bool ok = writeHeightField(*mapNode->getEllipsoidModel(), extent, hf.get(), sz);
		}
		else
		{
			std::cerr << "createTileKey(" << x << ", " << y << " , " << level << ") " << key->str() << " " 
				<< extent.toString() << " elev <nohightfield>" << std::endl;
			unlink(sz);
		}
	}

    osg::Group* root = new osg::Group();

    // The MapNode will render the Map object in the scene graph.
    mapNode->setNodeMask( 0x01 );
    root->addChild( mapNode );

    // A flag so we can see where we clicked
    osg::MatrixTransform* flag = createFlag();
    flag->setNodeMask( 0x02 );
    root->addChild( flag );

    viewer.setSceneData( root );

    // AN elevation manager that is tied to the map node:
    osgEarthUtil::ElevationManager* elevMan = new osgEarthUtil::ElevationManager( mapNode->getMap() );
    elevMan->setTechnique( osgEarthUtil::ElevationManager::TECHNIQUE_PARAMETRIC );
    elevMan->setMaxTilesToCache( 10 );

	double out_elev, out_res;
	elevMan->getElevation(11.566667, 48.133333, 0.5, NULL, out_elev, out_res);
	std::cerr << "getElevation(11.566667, 48.133333, 0.5, NULL, out_elev, out_res) " << out_elev << " " << out_res << std::endl;

	osgDB::writeNodeFile(*root, "C:\\tmp\\evas.osg");

    // An event handler that will respond to mouse clicks:
    viewer.addEventHandler( new QueryElevationHandler( elevMan, flag ) );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
