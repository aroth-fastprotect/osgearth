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
#include <limits.h>

#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/ImageUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace osgEarth::Threading;


/****************************************************************/

TileSourceOptions::TileSourceOptions( const PluginOptions* po ) :
DriverOptions( po ),
_tileSize( 256 ),
_noDataValue( (float)SHRT_MIN ),
_noDataMinValue( -FLT_MAX ),
_noDataMaxValue( FLT_MAX )
{
    if ( po )
        fromConfig( po->config() );
}

TileSourceOptions::TileSourceOptions( const Config& conf ) :
DriverOptions( conf ),
_tileSize( 256 ),
_noDataValue( (float)SHRT_MIN ),
_noDataMinValue( -FLT_MAX ),
_noDataMaxValue( FLT_MAX )
{
    fromConfig( conf );
}

void
TileSourceOptions::fromConfig( const Config& conf )
{
    config().getIfSet( "tile_size", _tileSize );
    config().getIfSet( "nodata_value", _noDataValue );
    config().getIfSet( "nodata_min", _noDataMinValue );
    config().getIfSet( "nodata_max", _noDataMaxValue );
    config().getIfSet( "blacklist_filename", _blacklistFilename);
    
    // special handling of default tile size:
    if ( !tileSize().isSet() )
        config().getIfSet( "default_tile_size", _tileSize );
    // remove it now so it does not get serialized
    config().remove( "default_tile_size" );
}

Config
TileSourceOptions::toConfig() const
{ 
    Config conf = DriverOptions::toConfig();
    conf.updateIfSet( "tile_size", _tileSize );
    conf.updateIfSet( "nodata_value", _noDataValue );
    conf.updateIfSet( "nodata_min", _noDataMinValue );
    conf.updateIfSet( "nodata_max", _noDataMaxValue );
    conf.updateIfSet( "blacklist_filename", _blacklistFilename);
    return conf;
}

/****************************************************************/  

TileBlacklist::TileBlacklist()
{
    //NOP
}

void
TileBlacklist::add(const osgTerrain::TileID &tile)
{
    ScopedWriteLock lock(_mutex);
    _tiles.insert(tile);
    OE_DEBUG << "Added " << tile.level << " (" << tile.x << ", " << tile.y << ") to blacklist" << std::endl;
}

void
TileBlacklist::remove(const osgTerrain::TileID &tile)
{
    ScopedWriteLock lock(_mutex);
    _tiles.erase(tile);
    OE_DEBUG << "Removed " << tile.level << " (" << tile.x << ", " << tile.y << ") from blacklist" << std::endl;
}

void
TileBlacklist::clear()
{
    ScopedWriteLock lock(_mutex);
    _tiles.clear();
    OE_DEBUG << "Cleared blacklist" << std::endl;
}

bool
TileBlacklist::contains(const osgTerrain::TileID &tile) const
{
    ScopedReadLock lock(const_cast<TileBlacklist*>(this)->_mutex);
    return _tiles.find(tile) != _tiles.end();
}

unsigned int
TileBlacklist::size() const
{
    ScopedReadLock lock(const_cast<TileBlacklist*>(this)->_mutex);
    return _tiles.size();
}

TileBlacklist*
TileBlacklist::read(std::istream &in)
{
    osg::ref_ptr< TileBlacklist > result = new TileBlacklist();


    while (!in.eof())
    {
        std::string line;
        std::getline(in, line);
        if (!line.empty())
        {
            int z, x, y;
            if (sscanf(line.c_str(), "%d %d %d", &z, &x, &y) == 3)
            {
                result->add(osgTerrain::TileID(z, x, y ));
            }

        }
    }

    return result.release();
}

TileBlacklist*
TileBlacklist::read(const std::string &filename)
{
    if (osgDB::fileExists(filename) && (osgDB::fileType(filename) == osgDB::REGULAR_FILE))
    {
        std::ifstream in( filename.c_str() );
        return read( in );
    }
    return NULL;
}

void
TileBlacklist::write(const std::string &filename) const
{ 
    std::string path = osgDB::getFilePath(filename);
    if (!osgDB::fileExists(path) && !osgDB::makeDirectory(path))
    {
        OE_NOTICE << "Couldn't create path " << std::endl;
        return;
    }
    std::ofstream out(filename.c_str());
    write(out);
}

void
TileBlacklist::write(std::ostream &output) const
{
    ScopedReadLock lock(const_cast<TileBlacklist*>(this)->_mutex);
    for (BlacklistedTiles::const_iterator itr = _tiles.begin(); itr != _tiles.end(); ++itr)
    {
        output << itr->level << " " << itr->x << " " << itr->y << std::endl;
    }
}

/****************************************************************/  

TileSource::TileSource(const PluginOptions* options)
{
    this->setThreadSafeRefUnref( true );

    _settings = dynamic_cast<const TileSourceOptions*>( options );
    if ( !_settings.valid() )
        _settings = new TileSourceOptions( options );

    _memCache = new MemCache();

    if (_settings->blacklistFilename().isSet())
    {
        _blacklistFilename = _settings->blacklistFilename().value();
    }

    
    if (!_blacklistFilename.empty() && osgDB::fileExists(_blacklistFilename))
    {
        _blacklist = TileBlacklist::read(_blacklistFilename);
        if (_blacklist.valid())
        {
            OE_INFO << "Read blacklist from file" << std::endl;
        }
    }

    if (!_blacklist.valid())
    {
        //Initialize the blacklist if we couldn't read it.
        _blacklist = new TileBlacklist();
    }
}

TileSource::~TileSource()
{
    if (_blacklist.valid() && !_blacklistFilename.empty())
    {
        _blacklist->write(_blacklistFilename);
    }
}

int
TileSource::getPixelsPerTile() const
{
    return _settings->tileSize().value();
}

osg::Image*
TileSource::getImage( const TileKey* key,
                      ProgressCallback* progress )
{
	//Try to get it from the memcache fist
    osg::ref_ptr<osg::Image> image = NULL;
	if (_memCache.valid())
	{
		image = _memCache->getImage( key,"","");
	}

	if (!image.valid())
	{
		image = createImage( key, progress );
		if (image.valid() && _memCache.valid())
		{
			_memCache->setImage( key, "", "",image.get() );
		}
	}
	return image.release();
}

osg::HeightField*
TileSource::getHeightField( const TileKey* key,
                            ProgressCallback* progress
                           )
{
    osg::ref_ptr<osg::HeightField> hf = NULL;
	if (_memCache.valid())
	{
		hf = _memCache->getHeightField( key, "", "" );
	}

	if (!hf.valid())
	{
		hf = createHeightField( key, progress );
		if (hf.valid() && _memCache.valid())
		{
			_memCache->setHeightField( key, "", "", hf.get() );
		}
	}
	return hf.release();
}

osg::HeightField*
TileSource::createHeightField( const TileKey* key,
                               ProgressCallback* progress)
{
    osg::ref_ptr<osg::Image> image = createImage(key, progress);
    osg::HeightField* hf = 0;
    if (image.valid())
    {
        ImageToHeightFieldConverter conv;
        hf = conv.convert( image.get() );
    }      
    return hf;
}

bool
TileSource::isOK() const 
{
    return getProfile() != NULL;
}

void
TileSource::setProfile( const Profile* profile )
{
    _profile = profile;
}

const Profile*
TileSource::getProfile() const
{
    return _profile.get();
}

unsigned int
TileSource::getMaxDataLevel() const
{
    //If we have no data extents, just use INT_MAX
    if (_dataExtents.size() == 0) return INT_MAX;

    unsigned int maxDataLevel = INT_MIN;
    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if (itr->getMaxLevel() > maxDataLevel) maxDataLevel = itr->getMaxLevel();
    }
    return maxDataLevel;
}

unsigned int
TileSource::getMinDataLevel() const
{
    //If we have no data extents, just use 0
    if (_dataExtents.size() == 0) return 0;

    unsigned int minDataLevel = INT_MAX;
    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if (itr->getMinLevel() < minDataLevel) minDataLevel = itr->getMinLevel();
    }
    return minDataLevel;
}

bool
TileSource::hasData(const osgEarth::TileKey* key)
{
    //osg::Timer_t start = osg::Timer::instance()->tick();
    const osgEarth::GeoExtent& keyExtent = key->getGeoExtent();
    bool intersectsData = false;
    if (_dataExtents.size() == 0) intersectsData = true;
    else
    {
        for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
        {
            if (keyExtent.intersects( *itr ) && key->getLevelOfDetail() >= itr->getMinLevel() && key->getLevelOfDetail() <= itr->getMaxLevel())
            {
                intersectsData = true;
            }
        }
    }
    //osg::Timer_t end = osg::Timer::instance()->tick();
    //OE_NOTICE << "hasData took " << osg::Timer::instance()->delta_m(start, end) << " ms to check " << _dataExtents.size() << " areas " << std::endl;
    return intersectsData;
}

bool
TileSource::supportsPersistentCaching() const
{
    return true;
}

TileBlacklist*
TileSource::getBlacklist()
{
    return _blacklist.get();
}

const TileBlacklist*
TileSource::getBlacklist() const
{
    return _blacklist.get();
}