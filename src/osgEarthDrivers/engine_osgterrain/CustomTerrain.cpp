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
#include "CustomTerrain"
#include "CustomTerrainTechnique"
#include "TransparentLayer"

#include <osgEarth/Registry>
#include <osgEarth/Locators>
#include <osgEarth/Map>
#include <osgEarth/FindNode>

#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Texture2D>
#include <osgGA/EventVisitor>

#include <OpenThreads/ScopedLock>


using namespace osgEarth;
using namespace OpenThreads;

#define LC "[CustomTerrain] "

// setting this will enable "fast GL object release" - the engine will activity
// track tiles that expire from the scene graph, and will explicity force them
// to deallocate their GL objects (instead of waiting for OSG to "lazily" 
// release them). This is helpful for freeing up memory more quickly when 
// aggresively navigating a map.
#define QUICK_RELEASE_GL_OBJECTS 1

//#define PREEMPTIVE_DEBUG 1

//----------------------------------------------------------------------------

// this progress callback checks to see whether the request being serviced is 
// out of date with respect to the task service that is running it. It checks
// for a disparity in frame stamps, and reports that the request should be
// canceled if it appears the request has been abandoned by the Tile that
// originally scheduled it.
struct StampedProgressCallback : ProgressCallback
{
public:
    StampedProgressCallback(TaskRequest* request, TaskService* service):
      _request(request),
      _service(service)
    {
    }

    //todo: maybe we should pass TaskRequest in as an argument 
    bool reportProgress(double current, double total)
    {
        //Check to see if we were marked cancelled on a previous check
        if (_canceled) return _canceled;

        _canceled = (_service->getStamp() - _request->getStamp() > 2);
        return _canceled;
    }

    TaskRequest* _request;
    TaskService* _service;
};

//----------------------------------------------------------------------------

// NOTE: Task requests run in background threads. So we pass in a map frame and
// make a clone of it to use in that thread. Each Task must have its own MapFrame
// so it's operating in its own sandbox.

struct TileLayerRequest : public TaskRequest
{
    TileLayerRequest( const TileKey& key, const MapFrame& mapf, OSGTileFactory* tileFactory )
        : _key( key ), 
          _mapf(mapf, "osgterrain.TileLayerRequest"), 
          _tileFactory(tileFactory), 
          _numTries(0), 
          _maxTries(3) { }

    TileKey _key;
    MapFrame _mapf;
    //osg::ref_ptr<Map> _map;
    osg::ref_ptr<OSGTileFactory> _tileFactory;
	unsigned int _numTries;
	unsigned int _maxTries;
};

struct TileColorLayerRequest : public TileLayerRequest
{
    TileColorLayerRequest( const TileKey& key, const MapFrame& mapf, OSGTileFactory* tileFactory, unsigned int layerId )
        : TileLayerRequest( key, mapf, tileFactory ), _layerId(layerId) { }

    void operator()( ProgressCallback* progress )
    {
        osg::ref_ptr<ImageLayer> imageLayer = 0L;

        for( ImageLayerVector::const_iterator i = _mapf.imageLayers().begin(); i != _mapf.imageLayers().end(); ++i )
        {
            if ( i->get()->getId() == _layerId )
                imageLayer = i->get();
        }

        if ( imageLayer.valid() )
        {
            _result = _tileFactory->createImageLayer( _mapf.getMapInfo(), imageLayer.get(), _key, progress );
			if (!wasCanceled())
			{
			  _numTries++;
			}
        }
    }
    unsigned int _layerId;
};

struct TileElevationLayerRequest : public TileLayerRequest
{
    TileElevationLayerRequest( const TileKey& key, const MapFrame& mapf, OSGTileFactory* tileFactory )
        : TileLayerRequest( key, mapf, tileFactory )
    {
    }

    void operator()( ProgressCallback* progress )
    {
        _result = _tileFactory->createHeightFieldLayer( _mapf, _key, true ); //exactOnly=true
		_numTries++;
    }
};

struct TileElevationPlaceholderLayerRequest : public TileLayerRequest
{
    TileElevationPlaceholderLayerRequest( const TileKey& key, const MapFrame& mapf, OSGTileFactory* tileFactory, GeoLocator* keyLocator )
        : TileLayerRequest( key, mapf, tileFactory ),
          _keyLocator(keyLocator),
          _parentKey( key.createParentKey() )
    {
        //nop
    }

    void setParentHF( osg::HeightField* parentHF )
    {
        _parentHF = parentHF; 
    }

    void setNextLOD( int nextLOD )
    {
        _nextLOD = nextLOD;
    }

    void operator()( ProgressCallback* progress )
    {
        if ( !progress->isCanceled() )
        {
            _result = _tileFactory->createPlaceholderHeightfieldLayer(
                _parentHF.get(),
                _parentKey,
                _key,
                _keyLocator.get() );
        }
    }

    osg::ref_ptr<osg::HeightField> _parentHF;
    TileKey _parentKey;
    osg::ref_ptr<GeoLocator>    _keyLocator;
    int _nextLOD;
};

// A task request that rebuilds a tile's terrain technique in the background. It
// re-compiles the geometry but does NOT apply the updates (since this constitutes
// altering the scene graph and must therefore be done in the update traversal).
struct TileGenRequest : public TaskRequest
{
    TileGenRequest( CustomTile* tile, const TileUpdate& update ) :
        _tile( tile ), _update(update) { }

    void operator()( ProgressCallback* progress )
    {
        if (_tile.valid())
        {
            CustomTerrainTechnique* tech = dynamic_cast<CustomTerrainTechnique*>( _tile->getTerrainTechnique() );
            if (tech)
            {
                tech->compile( _update, progress );
            }
        }

        //We don't need the tile anymore
        _tile = NULL;
    }

    osg::ref_ptr< CustomTile > _tile;
    TileUpdate _update;
};


/*****************************************************************************/

// family tile indicies in the _family vector
#define PARENT 0
#define WEST   1
#define NORTH  2
#define EAST   3
#define SOUTH  4


CustomTile::CustomTile( const TileKey& key, GeoLocator* keyLocator, bool quickReleaseGLObjects ) :
_key( key ),
_keyLocator( keyLocator ),
_useLayerRequests( false ),       // always set this to false here; use setUseLayerRequests() to enable
_terrainRevision( -1 ),
_tileRevision( 0 ),
_requestsInstalled( false ),
_elevationLayerDirty( false ),
_colorLayersDirty( false ),
_usePerLayerUpdates( false ),     // only matters when _useLayerRequests==true
_elevationLayerUpToDate( true ),
_elevationLOD( key.getLevelOfDetail() ),
_hasBeenTraversed(false),
_useTileGenRequest( true ),
//_tileGenNeeded( false ),
_verticalScale(1.0f),
_quickReleaseGLObjects( quickReleaseGLObjects )
{
    this->setThreadSafeRefUnref( true );

    this->setTileID( key.getTileId() );

    // because the lowest LOD (1) is always loaded fully:
    _elevationLayerUpToDate = _key.getLevelOfDetail() <= 1;

    // initially bump the update requirement so that this tile will receive an update
    // traversal the first time through. It is on the first update traversal that we
    // know the tile is in the scene graph and that it can be registered with the terrain.
    adjustUpdateTraversalCount( 1 );
}

CustomTile::~CustomTile()
{
    //OE_NOTICE << "Destroying CustomTile " << this->getKey()->str() << std::endl;
}

void
CustomTile::adjustUpdateTraversalCount( int delta )
{
    int oldCount = this->getNumChildrenRequiringUpdateTraversal();
    if ( oldCount + delta >= 0 )
    {
        this->setNumChildrenRequiringUpdateTraversal(
            (unsigned int)(oldCount + delta) );
    }
    else
    {
        OE_NOTICE << "WARNING, tile (" 
            << _key.str() << ") tried to set a negative NCRUT"
            << std::endl;
    }
}

bool
CustomTile::cancelRequests()
{
    // This method ensures that all requests owned by this object are stopped and released
    // by the corresponding task service prior to destructing the tile. Called by
    // CustomTerrain::updateTileTable().

    bool done = true;

    // Cancel all active requests
    if ( _requestsInstalled )
    {
        for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
        {
            i->get()->cancel();
        }

        if ( _elevRequest.valid() )
        {
            _elevRequest->cancel();
        }

        if (_elevPlaceholderRequest.valid())
        {
            _elevPlaceholderRequest->cancel();
        }

        if (_tileGenRequest.valid())
        {
            _tileGenRequest->cancel();
        }
    }

    return done;
}


Threading::ReadWriteMutex&
CustomTile::getTileLayersMutex()
{
    return _tileLayersMutex;
}

const TileKey&
CustomTile::getKey() const
{
    return _key;
}

void
CustomTile::setElevationLOD( int lod )
{
    _elevationLOD = lod;
    _elevationLayerUpToDate = _elevationLOD == _key.getLevelOfDetail();

    //Should probably just reset the placeholder requests
    //if (_elevPlaceholderRequest.valid()) _elevPlaceholderRequest->setState( TaskRequest::STATE_IDLE );
    //if (_elevRequest.valid()) _elevRequest->setState( TaskRequest::STATE_IDLE );
    //resetElevationRequests();
}

int
CustomTile::getElevationLOD() const
{
    return _elevationLOD;
}

bool
CustomTile::getHasBeenTraversed() const
{
    return _hasBeenTraversed;
}

CustomTerrain*
CustomTile::getCustomTerrain()
{
    if ( !_CustomTerrain.valid() )
        _CustomTerrain = static_cast<CustomTerrain*>(getTerrain());
    return _CustomTerrain.get();
}

const CustomTerrain*
CustomTile::getCustomTerrain() const
{
    return const_cast<CustomTile*>(this)->getCustomTerrain();
}

void
CustomTile::setUseLayerRequests( bool value )
{
    _useLayerRequests = value;
}

int
CustomTile::getTerrainRevision() const
{
    return _terrainRevision;
}

void
CustomTile::setTerrainRevision( int revision )
{
    _terrainRevision = revision;
}

bool
CustomTile::isInSyncWithTerrain() const
{
    return _terrainRevision == getCustomTerrain()->getRevision();
}

int
CustomTile::getTileRevision() const
{
    return _tileRevision;
}

void
CustomTile::incrementTileRevision()
{
    _tileRevision++;
}

void
CustomTile::setHasElevationHint( bool hint ) 
{
    _hasElevation = hint;
}

bool
CustomTile::isElevationLayerUpToDate() const 
{
    return _elevationLayerUpToDate;
}

bool
CustomTile::getUseTileGenRequest() const
{
    return _useTileGenRequest;
}

float
CustomTile::getVerticalScale() const
{
    return _verticalScale;
}

void
CustomTile::setVerticalScale(float verticalScale)
{
    if (_verticalScale != verticalScale)
    {
        _verticalScale = verticalScale;
        dirtyBound();
    }
}

osg::BoundingSphere
CustomTile::computeBound() const
{
    //Overriden computeBound that takes into account the vertical scale.
    //OE_NOTICE << "CustomTile::computeBound verticalScale = " << _verticalScale << std::endl;

    osg::BoundingSphere bs;

    if (_elevationLayer.valid())
    {        
        if (!_elevationLayer->getLocator()) return bs;

        osg::BoundingBox bb;
        unsigned int numColumns = _elevationLayer->getNumColumns();
        unsigned int numRows = _elevationLayer->getNumRows();
        for(unsigned int r=0;r<numRows;++r)
        {
            for(unsigned int c=0;c<numColumns;++c)
            {
                float value = 0.0f;
                bool validValue = _elevationLayer->getValidValue(c,r, value);
                if (validValue) 
                {
                    //Multiply by the vertical scale.
                    value *= _verticalScale;
                    osg::Vec3d ndc, v;
                    ndc.x() = ((double)c)/(double)(numColumns-1), 
                        ndc.y() = ((double)r)/(double)(numRows-1);
                    ndc.z() = value;

                    if (_elevationLayer->getLocator()->convertLocalToModel(ndc, v))
                    {
                        bb.expandBy(v);
                    }
                }
            }
        }
        bs.expandBy(bb);

    }
    else
    {
        for(Layers::const_iterator itr = _colorLayers.begin();
            itr != _colorLayers.end();
            ++itr)
        {
            if (itr->valid()) bs.expandBy((*itr)->computeBound(false));
        }
    }

    return bs;
    
}

// returns TRUE if it's safe for this tile to load its next elevation data layer.
bool
CustomTile::readyForNewElevation()
{
    bool ready = true;

    if ( _elevationLOD == _key.getLevelOfDetail() )
    {
        ready = false;
    }
    else if ( _family[PARENT].elevLOD < 0 )
    {
        ready = false;
    }
    else
    {
        for( int i=PARENT; i<=SOUTH; i++) 
        {
            if ( _family[i].expected && _family[i].elevLOD >= 0 && _family[i].elevLOD < _elevationLOD )
            {
                ready = false;
                break;
            }
        }

        // if the next LOD is not the final, but our placeholder is up to date, we're not ready.
        if ( ready && _elevationLOD+1 < _key.getLevelOfDetail() && _elevationLOD == _family[PARENT].elevLOD )
        {
            ready = false;
        }
    }

#ifdef PREEMPTIVE_DEBUG
    OE_NOTICE
        << "Tile (" << _key.str() << ") at (" << _elevationLOD << "), parent at ("
        << _family[PARENT].elevLOD << "), sibs at (";
    if ( _family[WEST].expected ) osg::notify( osg::NOTICE ) << "W=" << _family[WEST].elevLOD << " ";
    if ( _family[NORTH].expected ) osg::notify( osg::NOTICE ) << "N=" << _family[NORTH].elevLOD << " ";
    if ( _family[EAST].expected ) osg::notify( osg::NOTICE ) << "E=" << _family[EAST].elevLOD << " ";
    if ( _family[SOUTH].expected ) osg::notify( osg::NOTICE ) << "S=" << _family[SOUTH].elevLOD << " ";
    OE_NOTICE << "), ready = " << (ready? "YES" : "no") << std::endl;
#endif

    return ready;
}



// returns TRUE if it's safe for this tile to load its next elevation data layer.
bool
CustomTile::readyForNewImagery(ImageLayer* layer, int currentLOD)
{
    bool ready = true;

    if ( currentLOD == _key.getLevelOfDetail() )
    {
        ready = false;
    }
    else if ( _family[PARENT].getImageLOD(layer->getId()) < 0 )
    {
        ready = false;
    }
    else
    {
        for( int i=PARENT; i<=SOUTH; i++) 
        {
            if ( _family[i].expected && _family[i].getImageLOD(layer->getId()) >= 0 && _family[i].getImageLOD(layer->getId()) < currentLOD )
            {
                ready = false;
                break;
            }
        }

        // if the next LOD is not the final, but our placeholder is up to date, we're not ready.
        if ( ready && currentLOD+1 < _key.getLevelOfDetail() && currentLOD == _family[PARENT].getImageLOD(layer->getId()) )
        {
            ready = false;
        }
    }

    return ready;
}


#define PRI_IMAGE_OFFSET 0.1f // priority offset of imagery relative to elevation
#define PRI_LAYER_OFFSET 0.1f // priority offset of image layer(x) vs. image layer(x+1)

void
CustomTile::installRequests( const MapFrame& mapf, int stamp )
{
    CustomTerrain* terrain = getCustomTerrain();

    //Map* map = terrain->getMap();
    //Threading::ScopedReadLock lock( map->getMapDataMutex() );

    OSGTileFactory* tileFactory = terrain->getTileFactory();
    //MapEngine* engine = terrain->getEngine();

    bool hasElevationLayer;
    int numColorLayers;
    {
        Threading::ScopedReadLock lock( _tileLayersMutex );
        hasElevationLayer = this->getElevationLayer() != NULL;
        numColorLayers = this->getNumColorLayers();
    }

    if ( hasElevationLayer )
    {
        resetElevationRequests( mapf );     
    }

    // safely loop through the map layers and update the imagery for each:
    //ImageLayerVector imageLayers;
    //map->getImageLayers( imageLayers );

    for( int layerIndex = 0; layerIndex < numColorLayers; layerIndex++ )
    {
        if ( layerIndex < mapf.imageLayers().size() )
        {
            updateImagery( mapf.imageLayers()[layerIndex]->getId(), mapf, tileFactory );
        }
    }
    _requestsInstalled = true;
}

void
CustomTile::resetElevationRequests( const MapFrame& mapf )
{
    if (_elevRequest.valid() && _elevRequest->isRunning()) _elevRequest->cancel();
    if (_elevPlaceholderRequest.valid() && _elevPlaceholderRequest->isRunning()) _elevPlaceholderRequest->cancel();

    // this request will load real elevation data for the tile:
    _elevRequest = new TileElevationLayerRequest(_key, mapf, getCustomTerrain()->getTileFactory());
    float priority = (float)_key.getLevelOfDetail();
    _elevRequest->setPriority( priority );
    std::stringstream ss;
    ss << "TileElevationLayerRequest " << _key.str() << std::endl;
	std::string ssStr;
	ssStr = ss.str();
    _elevRequest->setName( ssStr );

    // this request will load placeholder elevation data for the tile:
    _elevPlaceholderRequest = new TileElevationPlaceholderLayerRequest(
        _key, mapf, getCustomTerrain()->getTileFactory(), _keyLocator.get() );
    _elevPlaceholderRequest->setPriority( priority );
    ss.str("");
    ss << "TileElevationPlaceholderLayerRequest " << _key.str() << std::endl;
	ssStr = ss.str();
    _elevPlaceholderRequest->setName( ssStr );
}


// called from installRequests (cull traversal) or terrainengine (main thread) ... so be careful!
void
CustomTile::updateImagery(unsigned int layerId, const MapFrame& mapf, OSGTileFactory* tileFactory)
{
    //NOTE: the following comment is OBE -gw
    //NOTE: the map data mutex is held upon entering this method

    CustomTerrain* terrain = getCustomTerrain();

    ImageLayer* mapLayer = NULL;
    unsigned int layerIndex = -1;
    for (unsigned int i = 0; i < mapf.imageLayers().size(); ++i)
    {
        if (mapf.imageLayers()[i]->getId() == layerId)
        {
            mapLayer = mapf.imageLayers()[i].get();
            layerIndex = i;
            break;
        }
    }

    if (!mapLayer)
    {
        OE_NOTICE << "updateImagery could not find MapLayer with id=" << layerId << std::endl;
        return;
    }

    //if ( mapLayer->isKeyValid( _key.get() ) )
    {
        unsigned int layerId = mapLayer->getId();
        // imagery is slighty higher priority than elevation data
        TaskRequest* r = new TileColorLayerRequest( _key, mapf, tileFactory, layerId );
        std::stringstream ss;
        ss << "TileColorLayerRequest " << _key.str() << std::endl;
		std::string ssStr;
		ssStr = ss.str();
        r->setName( ssStr );
        r->setState( osgEarth::TaskRequest::STATE_IDLE );

        // in image-sequential mode, we want to prioritize lower-LOD imagery since it
        // needs to come in before higher-resolution stuff. 
        if ( getCustomTerrain()->getLoadingPolicy().mode() == LoadingPolicy::MODE_SEQUENTIAL )
        {
            r->setPriority( -(float)_key.getLevelOfDetail() + PRI_IMAGE_OFFSET );
        }
        // in image-preemptive mode, the highest LOD should get higher load priority:
        else // MODE_PREEMPTIVE
        {
            r->setPriority( PRI_IMAGE_OFFSET + (float)_key.getLevelOfDetail());
        }

        r->setProgressCallback( new StampedProgressCallback( r, terrain->getImageryTaskService( layerIndex ) ));

        //If we already have a request for this layer, remove it from the list and use the new one
        for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
        {
            TileColorLayerRequest* r2 = static_cast<TileColorLayerRequest*>( i->get() );
            if (r2->_layerId == layerId)
            {
                _requests.erase( i );
                break;
            }
        }
        //Add the new imagery request
        _requests.push_back( r );
    }
}

// This method is called from the CULL TRAVERSAL, from TileImageBackfillCallback in OSGTileFactory.cpp.
void
CustomTile::servicePendingImageRequests( const MapFrame& mapf, int stamp )
{       
    //Don't do anything until we have been added to the scene graph
    if (!_hasBeenTraversed) return;

    // install our requests if they are not already installed:
    if ( !_requestsInstalled )
    {
        // since we're in the CULL thread, use the cull thread map frame:
        installRequests( mapf, stamp );
    }

    for( TaskRequestList::iterator i = _requests.begin(); i != _requests.end(); ++i )
    {
        TileColorLayerRequest* r = static_cast<TileColorLayerRequest*>( i->get() );

        //If a request has been marked as IDLE, the TaskService has tried to service it
        //and it was either deemed out of date or was cancelled, so we need to add it again.
        if ( r->isIdle() )
        {
            //OE_NOTICE << "Queuing IR (" << _key.str() << ")" << std::endl;
            r->setStamp( stamp );
            getCustomTerrain()->getImageryTaskService(r->_layerId)->add( r );
        }
        else if ( !r->isCompleted() )
        {
            r->setStamp( stamp );
        }
    }    
}

Relative*
CustomTile::getFamily() {
    return _family;
}

// This method is called from the UPDATE TRAVERSAL, from CustomTerrain::traverse.
void
CustomTile::servicePendingElevationRequests( const MapFrame& mapf, int stamp, bool tileTableLocked )
{
    //Don't do anything until we have been added to the scene graph
    if (!_hasBeenTraversed) return;


    // install our requests if they are not already installed:
    if ( !_requestsInstalled )
    {
        installRequests( mapf, stamp );
    }

    if ( _hasElevation && !_elevationLayerUpToDate && _elevRequest.valid() && _elevPlaceholderRequest.valid() )
    {  
        CustomTerrain* terrain = getCustomTerrain();

        // update the main elevation request if it's running:
        if ( !_elevRequest->isIdle() )
        {
#ifdef PREEMPTIVE_DEBUG
            OE_NOTICE << "Tile (" << _key.str() << ") .. ER not idle" << std::endl;
#endif
            
            if ( !_elevRequest->isCompleted() )
            {
                _elevRequest->setStamp( stamp );
            }
        }

        // update the placeholder request if it's running:
        else if ( !_elevPlaceholderRequest->isIdle() )
        {
#ifdef PREEMPTIVE_DEBUG
            OE_NOTICE << "Tile (" << _key.str() << ") .. PR not idle" << std::endl;
#endif
            if ( !_elevPlaceholderRequest->isCompleted() )
            {
               _elevPlaceholderRequest->setStamp( stamp );
            }
        }

        // otherwise, see if it is legal yet to start a new request:
        else if ( readyForNewElevation() )
        {
            if ( _elevationLOD + 1 == _key.getLevelOfDetail() )
            {
                _elevRequest->setStamp( stamp );
                _elevRequest->setProgressCallback( new ProgressCallback() );
                terrain->getElevationTaskService()->add( _elevRequest.get() );
#ifdef PREEMPTIVE_DEBUG
                OE_NOTICE << "..queued FE req for (" << _key.str() << ")" << std::endl;
#endif
            }
            
            else if ( _family[PARENT].elevLOD > _elevationLOD )
            {
                osg::ref_ptr<CustomTile> parentTile;
                terrain->getCustomTile( _family[PARENT].tileID, parentTile, !tileTableLocked );

                if ( _elevationLOD < _family[PARENT].elevLOD && parentTile.valid() )
                {
                    TileElevationPlaceholderLayerRequest* er = static_cast<TileElevationPlaceholderLayerRequest*>(_elevPlaceholderRequest.get());

                    er->setStamp( stamp );
                    er->setProgressCallback( new ProgressCallback() );
                    float priority = (float)_key.getLevelOfDetail();
                    er->setPriority( priority );
                    //TODO: should there be a read lock here when accessing the parent tile's elevation layer? GW
                    osgTerrain::HeightFieldLayer* hfLayer = static_cast<osgTerrain::HeightFieldLayer*>(parentTile->getElevationLayer());
                    er->setParentHF( hfLayer->getHeightField() );
                    er->setNextLOD( _family[PARENT].elevLOD );
                    terrain->getElevationTaskService()->add( er );
#ifdef PREEMPTIVE_DEBUG
                    OE_NOTICE << "..queued PH req for (" << _key.str() << ")" << std::endl;
#endif
                }

                else 
                {
#ifdef PREEMPTIVE_DEBUG
                    OE_NOTICE << "...tile (" << _key.str() << ") ready, but nothing to do." << std::endl;
#endif
                }
            }
        }
    }
}

void
CustomTile::queueTileUpdate( TileUpdate::Action action, int index )
{
    if ( _useTileGenRequest )
    {
        _tileUpdates.push( TileUpdate(action, index) );
    }
    else
    {
        this->setDirty( true );
    }
}

// called from the UPDATE TRAVERSAL, because this method can potentially alter
// the scene graph.
bool
CustomTile::serviceCompletedRequests( const MapFrame& mapf, bool tileTableLocked )
{
    //Don't do anything until we have been added to the scene graph
    if (!_hasBeenTraversed) return false;

    bool tileModified = false;

    if ( !_requestsInstalled )
        return false;

    // First service the tile generator:
    if ( _tileGenRequest.valid() && _tileGenRequest->isCompleted() )
    {
        CustomTerrainTechnique* tech = dynamic_cast<CustomTerrainTechnique*>( getTerrainTechnique() );
        if ( tech )
        {
            //TODO: consider waiting to apply if there are still more tile updates in the queue.
            if ( _tileUpdates.size() == 0 )
                tileModified = tech->applyTileUpdates();
        }
        _tileGenRequest = 0L;
    }


    // now deal with imagery.
    const LoadingPolicy& lp = getCustomTerrain()->getLoadingPolicy();

    //Check each layer independently.
    for (unsigned int i = 0; i < mapf.imageLayers().size(); ++i)
    {
        ImageLayer* imageLayer = mapf.imageLayers()[i].get();

        bool checkForFinalImagery = false;

        if (i < getNumColorLayers())
        {
            TransparentLayer* layer = dynamic_cast<TransparentLayer*>(getColorLayer( i ));
            if (layer)
            {
                if ( lp.mode() == LoadingPolicy::MODE_PREEMPTIVE )
                {
                    // in preemptive mode, always check for the final imagery - there are no intermediate
                    // placeholders.
                    checkForFinalImagery = true;
                }
                else if (
                    lp.mode() == LoadingPolicy::MODE_SEQUENTIAL && 
                    layer && 
                    readyForNewImagery(imageLayer, layer->getLevelOfDetail()) )
                {
                    // in sequential mode, we have to incrementally increase imagery resolution by
                    // creating placeholders based of parent tiles, one LOD at a time.
                    if ( layer->getLevelOfDetail()+1 < _key.getLevelOfDetail() )
                    {
                        if ( _family[PARENT].getImageLOD(layer->getId()) > layer->getLevelOfDetail() )
                        {
                            osg::ref_ptr<CustomTile> parentTile;
                            getCustomTerrain()->getCustomTile( _family[PARENT].tileID, parentTile, !tileTableLocked );

                            //Get the parent color layer
                            osg::ref_ptr<osgTerrain::Layer> parentColorLayer;
                            {
                                Threading::ScopedReadLock l2( parentTile->getTileLayersMutex() );
                                if (i < parentTile->getNumColorLayers())
                                {
                                    parentColorLayer = parentTile->getColorLayer(i);
                                }
                            }

                            //Set the parent color layer
                            {
                                Threading::ScopedWriteLock lock( getTileLayersMutex() );
                                if (parentColorLayer.valid())
                                {
                                    setColorLayer(i, parentColorLayer.get());
                                }
                            }

                            queueTileUpdate( TileUpdate::UPDATE_IMAGE_LAYER, i );
                            //markTileForRegeneration();
                        }
                    }
                    else
                    {
                        // we've gone as far as we can with placeholders; time to check for the
                        // final imagery tile.
                        checkForFinalImagery = true;
                    }
                }
            }
        }

        if ( checkForFinalImagery )
        {
            // Then the image requests:
            for( TaskRequestList::iterator itr = _requests.begin(); itr != _requests.end(); )
            {
                bool increment = true;
                TileColorLayerRequest* r = static_cast<TileColorLayerRequest*>( itr->get() );
                //We only care about the current layer we are checking
                if (r->_layerId == imageLayer->getId())
                {
                    if ( itr->get()->isCompleted() )
                    {
                        if ( r->wasCanceled() )
                        {
                            //Reset the cancelled task to IDLE and give it a new progress callback.
                            r->setState( TaskRequest::STATE_IDLE );
                            r->setProgressCallback( new StampedProgressCallback(
                                r, getCustomTerrain()->getImageryTaskService(r->_layerId)));
                            r->reset();
                        }
                        else // success..
                        {
                            int index = -1;
                            {
                                // Lock the map data mutex, since we are querying the map model:
                                //NO need for this due to MapFrame
                                //Threading::ScopedReadLock mapDataLock( map->getMapDataMutex() );

                                //See if we even care about the request
                                for (unsigned int j = 0; j < mapf.imageLayers().size(); ++j)
                                {
                                    if (mapf.imageLayers()[j]->getId() == r->_layerId)
                                    {
                                        index = j;
                                        break;
                                    }
                                }
                            }

                            if (index < 0)
                            {
                                //The maplayer was probably deleted
                                OE_DEBUG << "Layer " << r->_layerId << " no longer exists, ignoring TileColorLayerRequest " << std::endl;
                                itr = _requests.erase(itr);
                                increment = false;
                            }
                            else
                            {
                                osg::ref_ptr<osgTerrain::ImageLayer> newImgLayer = static_cast<osgTerrain::ImageLayer*>( r->getResult() );
                                if ( newImgLayer.valid() )
                                {
                                    // update the color layer safely:
                                    {
                                        Threading::ScopedWriteLock layerLock( getTileLayersMutex() );
                                        this->setColorLayer( index, newImgLayer.get() );
                                    }

                                    queueTileUpdate( TileUpdate::UPDATE_IMAGE_LAYER, index );

                                    //OE_NOTICE << "Complete IR (" << _key.str() << ") layer=" << r->_layerId << std::endl;

                                    // remove from the list (don't reference "r" after this!)
                                    itr = _requests.erase( itr );
                                    increment = false;
                                }
                                else
                                {  
									if (r->_numTries > r->_maxTries)
									{
										osg::ref_ptr< TransparentLayer > oldLayer = dynamic_cast<TransparentLayer*>(this->getColorLayer(index));
										if (oldLayer)
										{
											TransparentLayer* newLayer = new TransparentLayer(oldLayer->getImage(), oldLayer->getMapLayer());
											newLayer->setLocator( oldLayer->getLocator() );
											newLayer->setName( oldLayer->getName() );
											newLayer->setLevelOfDetail(_key.getLevelOfDetail());
											// update the color layer safely:
											{
												Threading::ScopedWriteLock layerLock( getTileLayersMutex() );
												this->setColorLayer( index, newLayer );
											}

											//static_cast<osgEarth::TransparentLayer*>(this->getColorLayer(index))->setLevelOfDetail( _key.getLevelOfDetail());										
											itr = _requests.erase( itr );
											increment = false;
											OE_DEBUG << "Tried (" << _key.str() << ") (layer " << r->_layerId << "), too many times, moving on...." << std::endl;
										}
									}
									else
									{
										OE_DEBUG << "IReq error (" << _key.str() << ") (layer " << r->_layerId << "), retrying" << std::endl;

										//The color layer request failed, probably due to a server error. Reset it.
										r->setState( TaskRequest::STATE_IDLE );
										r->reset();
									}
                                }
                            }
                        }
                    }
                }

                if ( increment )
                    ++itr;
            }
        }
    }

    // Finally, the elevation requests:
    if ( _hasElevation && !_elevationLayerUpToDate && _elevRequest.valid() && _elevPlaceholderRequest.valid() )
    {
        // First, check is the Main elevation request is done. If so, we will now have the final HF data
        // and can shut down the elevation requests for this tile.
        if ( _elevRequest->isCompleted() )
        {
            if ( _elevRequest->wasCanceled() )
            {
                // If the request was canceled, reset it to IDLE and reset the callback. On the next
                _elevRequest->setState( TaskRequest::STATE_IDLE );
                _elevRequest->setProgressCallback( new ProgressCallback() );            
                _elevRequest->reset();
            }
            else // success:
            {
                // if the elevation request succeeded, install the new elevation layer!
                TileElevationLayerRequest* r = static_cast<TileElevationLayerRequest*>( _elevRequest.get() );
                osg::ref_ptr<osgTerrain::HeightFieldLayer> newHFLayer = static_cast<osgTerrain::HeightFieldLayer*>( r->getResult() );
                if ( newHFLayer.valid() && newHFLayer->getHeightField() != NULL )
                {
                    newHFLayer->getHeightField()->setSkirtHeight( 
                        getCustomTerrain()->getTileFactory()->getTerrainOptions().heightFieldSkirtRatio().get()
                        * this->getBound().radius() );

                    // need to write-lock the layer data since we'll be changing it:
                    {
                        Threading::ScopedWriteLock lock( _tileLayersMutex );
                        this->setElevationLayer( newHFLayer.get() );
                        this->dirtyBound();
                    }

                    // the tile needs rebuilding. This will kick off a TileGenRequest.
                    queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                    
                    // finalize the LOD marker for this tile, so other tiles can see where we are.
                    //setElevationLOD( _key.getLevelOfDetail() );
                    _elevationLOD = _key.getLevelOfDetail();

    #ifdef PREEMPTIVE_DEBUG
                    OE_NOTICE << "Tile (" << _key.str() << ") final HF, LOD (" << _elevationLOD << ")" << std::endl;
    #endif
                    // this was the final elev request, so mark elevation as DONE.
                    _elevationLayerUpToDate = true;
                    
                    // GW- just reset these and leave them alone and let cancelRequests() take care of cleanup later.
                    // done with our Elevation requests!
                    //_elevRequest = 0L;
                    //_elevPlaceholderRequest = 0L;
                }
                else
                {
                    _elevRequest->setState( TaskRequest::STATE_IDLE );
                    _elevRequest->reset();
                }
            }
        }

        else if ( _elevPlaceholderRequest->isCompleted() )
        {
            TileElevationPlaceholderLayerRequest* r = 
                static_cast<TileElevationPlaceholderLayerRequest*>(_elevPlaceholderRequest.get());

            if ( r->wasCanceled() )
            {
                r->setState( TaskRequest::STATE_IDLE );
                r->setProgressCallback( new ProgressCallback() );
                r->reset();
            }
            else // success:
            {
                osg::ref_ptr<osgTerrain::HeightFieldLayer> newPhLayer = static_cast<osgTerrain::HeightFieldLayer*>( r->getResult() );
                if ( newPhLayer.valid() && newPhLayer->getHeightField() != NULL )
                {
                    // install the new elevation layer.
                    {
                        Threading::ScopedWriteLock lock( _tileLayersMutex );
                        this->setElevationLayer( newPhLayer.get() );
                        this->dirtyBound();
                    }

                    // tile needs to be recompiled.
                    queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                    //markTileForRegeneration();

                    // update the elevation LOD for this tile, now that the new HF data is installed. This will
                    // allow other tiles to see where this tile's HF data is.
                    _elevationLOD = r->_nextLOD;
                    //setElevationLOD( r->_nextLOD );

    #ifdef PREEMPTIVE_DEBUG
                    OE_NOTICE << "..tile (" << _key.str() << ") is now at (" << _elevationLOD << ")" << std::endl;
    #endif
                }
                _elevPlaceholderRequest->setState( TaskRequest::STATE_IDLE );
                _elevPlaceholderRequest->reset();
            }
        }
    }

    // if we have a new TileGenRequest, queue it up now.
    if ( _tileUpdates.size() > 0 && !_tileGenRequest.valid() ) // _tileGenNeeded && !_tileGenRequest.valid())
    {
        _tileGenRequest = new TileGenRequest( this, _tileUpdates.front() );
        _tileUpdates.pop();
        //OE_NOTICE << "tile (" << _key.str() << ") queuing new tile gen" << std::endl;
        getCustomTerrain()->getTileGenerationTaskSerivce()->add( _tileGenRequest.get() );
    }

    return tileModified;
}

void
CustomTile::traverse( osg::NodeVisitor& nv )
{
    if ( !_hasBeenTraversed && nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        Threading::ScopedWriteLock lock( this->_tileLayersMutex );
        {
            if ( !_hasBeenTraversed && getCustomTerrain() )
            {
                _hasBeenTraversed = true;

                // we constructed this tile with an update traversal count of 1 so it would get
                // here and we could register the tile. Now we can decrement it back to normal.
                // this MUST be called from the UPDATE traversal.
                adjustUpdateTraversalCount( -1 );                
            }
        }
    }
   
    osgTerrain::TerrainTile::traverse( nv );
}

void
CustomTile::releaseGLObjects(osg::State* state) const
{
    Group::releaseGLObjects(state);

    if ( _quickReleaseGLObjects && _terrainTechnique.valid() )
    {
        //NOTE: crashes sometimes if OSG_RELEASE_DELAY is set -gw
        _terrainTechnique->releaseGLObjects( state );
    }
}

//----------------------------------------------------------------------------

namespace
{
    /**
     * A draw callback to calls another, nested draw callback.
     */
    struct NestingDrawCallback : public osg::Camera::DrawCallback
    {
        NestingDrawCallback( osg::Camera::DrawCallback* next ) : _next(next) { }

        virtual void operator()( osg::RenderInfo& renderInfo ) const
        {
            dispatch( renderInfo );
        }

        void dispatch( osg::RenderInfo& renderInfo ) const
        {
            if ( _next )
                _next->operator ()( renderInfo );
        }

        osg::ref_ptr<osg::Camera::DrawCallback> _next;
    };


    // a simple draw callback, to be installed on a Camera, that tells all CustomTerrains to
    // release GL memory on any expired tiles.
    struct QuickReleaseGLCallback : public NestingDrawCallback
    {
	    typedef std::vector< osg::observer_ptr< CustomTerrain > > ObserverTerrainList;

        QuickReleaseGLCallback( CustomTerrain* terrain, osg::Camera::DrawCallback* next )
            : NestingDrawCallback(next), _terrain(terrain) { }

        virtual void operator()( osg::RenderInfo& renderInfo ) const
        {
            osg::ref_ptr<CustomTerrain> terrainSafe = _terrain.get();
            if ( terrainSafe.valid() )
            {
                terrainSafe->releaseGLObjectsForTiles( renderInfo.getState() );
            }
            dispatch( renderInfo );
        }

        osg::observer_ptr<CustomTerrain> _terrain;
    };
}

#undef  LC
#define LC "[CustomTerrain] "

//TODO:  Register with the callback when we are first created...
// immediately release GL memory for any expired tiles.
// called from the DRAW thread
void
CustomTerrain::releaseGLObjectsForTiles(osg::State* state)
{
    Threading::ScopedReadLock lock( _tilesMutex );

    while( _tilesToRelease.size() > 0 )
    {
        _tilesToRelease.front()->releaseGLObjects( state );
        _tilesToRelease.pop();
    }
}

CustomTerrain::CustomTerrain(const MapFrame& update_mapf, 
                             const MapFrame& cull_mapf, 
                             OSGTileFactory* tileFactory,
                             bool            quickReleaseGLObjects ) :
_update_mapf( update_mapf ),
_cull_mapf( cull_mapf ),
_tileFactory( tileFactory ),
_revision(0),
_numLoadingThreads( 0 ),
_registeredWithReleaseGLCallback( false ),
_quickReleaseGLObjects( quickReleaseGLObjects ),
_quickReleaseCallbackInstalled( false ),
_onDemandDelay( 2 )
{
    this->setThreadSafeRefUnref( true );

    _loadingPolicy = _tileFactory->getTerrainOptions().loadingPolicy().get();

    if ( _loadingPolicy.mode() != LoadingPolicy::MODE_STANDARD )
    {
        setNumChildrenRequiringUpdateTraversal( 1 );
        const char* env_numTaskServiceThreads = getenv("OSGEARTH_NUM_PREEMPTIVE_LOADING_THREADS");
        if ( env_numTaskServiceThreads )
        {
            _numLoadingThreads = ::atoi( env_numTaskServiceThreads );
        }
        else
        if ( _loadingPolicy.numLoadingThreads().isSet() )
        {
            _numLoadingThreads = osg::maximum( 1, _loadingPolicy.numLoadingThreads().get() );
        }
        else
        {
            _numLoadingThreads = (int)osg::maximum( 1.0f, _loadingPolicy.numLoadingThreadsPerCore().get() * (float)GetNumberOfProcessors() );
        }

        OE_INFO << LC << "Using a total of " << _numLoadingThreads << " loading threads " << std::endl;
    }
    
    // undo the setting in osgTerrain::Terrain
    setNumChildrenRequiringUpdateTraversal( 0 );

    // register for events in order to support ON_DEMAND frame scheme
    setNumChildrenRequiringEventTraversal( 1 );
}

CustomTerrain::~CustomTerrain()
{
    //nop
}

void
CustomTerrain::incrementRevision()
{
    // no need to lock; if we miss it, we'll get it the next time around
    _revision++;
}

int
CustomTerrain::getRevision() const
{
    // no need to lock; if we miss it, we'll get it the next time around
    return _revision;
}

void
CustomTerrain::getCustomTile( const osgTerrain::TileID& tileID,
                                   osg::ref_ptr<CustomTile>& out_tile,
                                   bool lock )
{
    if ( lock )
    {
        Threading::ScopedReadLock lock( _tilesMutex );
        TileTable::iterator i = _tiles.find( tileID );
        out_tile = i != _tiles.end()? i->second.get() : 0L;
    }
    else
    {
        TileTable::iterator i = _tiles.find( tileID );
        out_tile = i != _tiles.end()? i->second.get() : 0L;
    }
}

void
CustomTerrain::getCustomTiles( TileList& out_list )
{
    Threading::ScopedReadLock lock( _tilesMutex );
    for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); i++ )
        out_list.push_back( i->second.get() );
}

void
CustomTerrain::getTerrainTiles( TerrainTileList& out_list )
{
    Threading::ScopedReadLock lock( _tilesMutex );
    for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); i++ )
    {
        out_list.push_back( i->second.get() );
    }
}

const LoadingPolicy&
CustomTerrain::getLoadingPolicy() const
{
    return _loadingPolicy;
}

// This method is called by CustomTerrain::traverse() in the UPDATE TRAVERSAL.
void
CustomTerrain::refreshFamily(const MapInfo& mapInfo,
                             const osgTerrain::TileID& tileId,
                             Relative* family,
                             bool tileTableLocked )
{
    // geocentric maps wrap around in the X dimension.
    bool wrapX = mapInfo.isGeocentric();
    unsigned int tilesX, tilesY;
    mapInfo.getProfile()->getNumTiles( tileId.level, tilesX, tilesY );

    // parent
    {
        family[PARENT].expected = true;
        family[PARENT].elevLOD = -1;
        family[PARENT].imageLODs.clear();
        family[PARENT].tileID = osgTerrain::TileID( tileId.level-1, tileId.x/2, tileId.y/2 );

        osg::ref_ptr<CustomTile> parent;
        getCustomTile( family[PARENT].tileID, parent, !tileTableLocked );
        if ( parent.valid() ) {
            family[PARENT].elevLOD = parent->getElevationLOD();
            for (unsigned int i = 0; i < parent->getNumColorLayers(); ++i)
            {
                TransparentLayer* layer = dynamic_cast<TransparentLayer*>(parent->getColorLayer(i));
                if (layer)
                {
                    family[PARENT].imageLODs[layer->getId()] = layer->getLevelOfDetail();
                }
            }
        }
    }

    // west
    {
        family[WEST].expected = tileId.x > 0 || wrapX;
        family[WEST].elevLOD = -1;
        family[WEST].imageLODs.clear();
        //family[WEST].imageryLOD = -1;
        family[WEST].tileID = osgTerrain::TileID( tileId.level, tileId.x > 0? tileId.x-1 : tilesX-1, tileId.y );
        osg::ref_ptr<CustomTile> west;
        getCustomTile( family[WEST].tileID, west, !tileTableLocked );
        if ( west.valid() ) {
            family[WEST].elevLOD = west->getElevationLOD();
            //family[WEST].imageryLOD = west->getImageryLOD();
            for (unsigned int i = 0; i < west->getNumColorLayers(); ++i)
            {
                TransparentLayer* layer = dynamic_cast<TransparentLayer*>(west->getColorLayer(i));
                if (layer)
                {
                    family[WEST].imageLODs[layer->getId()] = layer->getLevelOfDetail();
                }
            }
        }
    }

    // north
    {
        family[NORTH].expected = tileId.y < tilesY-1;
        family[NORTH].elevLOD = -1;
        //family[NORTH].imageryLOD = -1;
        family[NORTH].imageLODs.clear();
        family[NORTH].tileID = osgTerrain::TileID( tileId.level, tileId.x, tileId.y < tilesY-1 ? tileId.y+1 : 0 );
        osg::ref_ptr<CustomTile> north;
        getCustomTile( family[NORTH].tileID, north, !tileTableLocked );
        if ( north.valid() ) {
            family[NORTH].elevLOD = north->getElevationLOD();
            //family[NORTH].imageryLOD = north->getImageryLOD();
            for (unsigned int i = 0; i < north->getNumColorLayers(); ++i)
            {
                TransparentLayer* layer = dynamic_cast<TransparentLayer*>(north->getColorLayer(i));
                if (layer)
                {
                    family[NORTH].imageLODs[layer->getId()] = layer->getLevelOfDetail();
                }
            }
        }
    }

    // east
    {
        family[EAST].expected = tileId.x < tilesX-1 || wrapX;
        family[EAST].elevLOD = -1;
        //family[EAST].imageryLOD = -1;
        family[EAST].imageLODs.clear();
        family[EAST].tileID = osgTerrain::TileID( tileId.level, tileId.x < tilesX-1 ? tileId.x+1 : 0, tileId.y );
        osg::ref_ptr<CustomTile> east;
        getCustomTile( family[EAST].tileID, east, !tileTableLocked );
        if ( east.valid() ) {
            family[EAST].elevLOD = east->getElevationLOD();
            //family[EAST].imageryLOD = east->getImageryLOD();
            for (unsigned int i = 0; i < east->getNumColorLayers(); ++i)
            {
                TransparentLayer* layer = dynamic_cast<TransparentLayer*>(east->getColorLayer(i));
                if (layer)
                {
                    family[EAST].imageLODs[layer->getId()] = layer->getLevelOfDetail();
                }
            }
        }
    }

    // south
    {
        family[SOUTH].expected = tileId.y > 0;
        family[SOUTH].elevLOD = -1;
        //family[SOUTH].imageryLOD = -1;
        family[SOUTH].imageLODs.clear();
        family[SOUTH].tileID = osgTerrain::TileID( tileId.level, tileId.x, tileId.y > 0 ? tileId.y-1 : tilesY-1 );
        osg::ref_ptr<CustomTile> south;
        getCustomTile( family[SOUTH].tileID, south, !tileTableLocked );
        if ( south.valid() ) {
            family[SOUTH].elevLOD = south->getElevationLOD();
            //family[SOUTH].imageryLOD = south->getImageryLOD();
            for (unsigned int i = 0; i < south->getNumColorLayers(); ++i)
            {
                TransparentLayer* layer = dynamic_cast<TransparentLayer*>(south->getColorLayer(i));
                if (layer)
                {
                    family[SOUTH].imageLODs[layer->getId()] = layer->getLevelOfDetail();
                }
            }
        }
    }
}

OSGTileFactory*
CustomTerrain::getTileFactory() {
    return _tileFactory.get();
}

void
CustomTerrain::addTerrainCallback( TerrainCallback* cb )
{
    _terrainCallbacks.push_back( cb );
}

void
CustomTerrain::registerTile( CustomTile* newTile )
{
    Threading::ScopedWriteLock lock( _tilesMutex );
    //Register the new tile immediately, but also add it to the queue so that
    _tiles[ newTile->getTileID() ] = newTile;
    _tilesToAdd.push( newTile );
    //OE_NOTICE << "Registered " << newTile->getKey()->str() << " Count=" << _tiles.size() << std::endl;
}

unsigned int
CustomTerrain::getNumTasksRemaining() const
{
    ScopedLock<Mutex> lock(const_cast<CustomTerrain*>(this)->_taskServiceMutex );
    unsigned int total = 0;
    for (TaskServiceMap::const_iterator itr = _taskServices.begin(); itr != _taskServices.end(); ++itr)
    {
        total += itr->second->getNumRequests();
    }
    return total;
}

void
CustomTerrain::traverse( osg::NodeVisitor &nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        // if the terrain engine requested "quick release", install the quick release
        // draw callback now.
        if ( _quickReleaseGLObjects && !_quickReleaseCallbackInstalled )
        {
            osg::Camera* cam = findFirstParentOfType<osg::Camera>( this );
            if ( cam )
            {
                cam->setPostDrawCallback( new QuickReleaseGLCallback( this, cam->getPostDrawCallback() ) );
                _quickReleaseCallbackInstalled = true;
            }
        }

        // this stamp keeps track of when requests are dispatched. If a request's stamp gets too
        // old, it is considered "expired" and subject to cancelation
        int stamp = nv.getFrameStamp()->getFrameNumber();
        
        TerrainTileList _updatedTiles;

        // update the internal Tile table.
        {
            Threading::ScopedWriteLock lock( _tilesMutex );

            int oldSize = _tiles.size();

            for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); )
            {
                if ( i->second.valid() && i->second->referenceCount() == 1 && i->second->getHasBeenTraversed() )
                {
                    //OE_NOTICE << "Tile (" << i->second->getKey()->str() << ") expired..." << std::endl;

                    _tilesToShutDown.push_back( i->second.get() );
                    TileTable::iterator j = i;
                    ++i;
                    _tiles.erase( j );
                }
                else
                {
                    ++i;
                }
            }
            
            //OE_NOTICE << "Shutting down " << _tilesToShutDown.size() << " tiles." << std::endl;

            for( TileList::iterator i = _tilesToShutDown.begin(); i != _tilesToShutDown.end(); )
            {
                if ( i->get()->cancelRequests() )
                {
                    // if quick release is activated, queue up the tile for release.
                    if ( _quickReleaseGLObjects && _quickReleaseCallbackInstalled )
                    {
                        _tilesToRelease.push( i->get() );
                    }

                    i = _tilesToShutDown.erase( i );
                }
                else
                    ++i;
            }

            // Add any newly registered tiles to the table. If a tile is in the _tilesToAdd
            // queue, we know it is already in the scene graph.
            while( _tilesToAdd.size() > 0 )
            {
                //_tiles[ _tilesToAdd.front()->getTileID() ] = _tilesToAdd.front().get();
                if ( _terrainCallbacks.size() > 0 )
                    _updatedTiles.push_back( _tilesToAdd.front().get() );
                _tilesToAdd.pop();
            }

            //if ( _tiles.size() != oldSize )
            //{
            //    OE_NOTICE << "Tiles registered = " << _tiles.size() << std::endl;
            //}
        }

        // update the frame stamp on the task services. This is necessary to support 
        // automatic request cancelation for image requests.
        {
            ScopedLock<Mutex> lock( _taskServiceMutex );
            for (TaskServiceMap::iterator i = _taskServices.begin(); i != _taskServices.end(); ++i)
            {
                i->second->setStamp( stamp );
            }
        }

        // should probably find a way to not hold this mutex during the loop.. oh well
        {
            Threading::ScopedReadLock lock( _tilesMutex );

            for( TileTable::iterator i = _tiles.begin(); i != _tiles.end(); ++i )
            {
                if ( i->second.valid() )
                {
                    refreshFamily( _update_mapf.getMapInfo(), i->first, i->second->getFamily(), true );

                    if ( i->second->getUseLayerRequests() )
                    {                        
                        i->second->servicePendingElevationRequests( _update_mapf, stamp, true );

                        //if ( _updatedTiles.size() < 1 && stamp % 10 == 0 )
                        {
                            bool tileModified = i->second->serviceCompletedRequests( _update_mapf, true );
                            if ( tileModified && _terrainCallbacks.size() > 0 )
                            {
                                _updatedTiles.push_back( i->second.get() );
                            }
                        }
                    }
                }
            }

            // notify listeners of tile modifications.
            if ( _updatedTiles.size() > 0 )
            {
                for( TerrainCallbackList::iterator n = _terrainCallbacks.begin(); n != _terrainCallbacks.end(); ++n )
                {
                    n->get()->onTerrainTilesUpdated( _updatedTiles );
                }
            }
        }
    }
    
    else if ( nv.getVisitorType() == osg::NodeVisitor::EVENT_VISITOR )
    {
        // in OSG's "ON_DEMAND" frame scheme, OSG runs the event visitor as part of the
        // test to see if a frame is needed. In sequential/preemptive mode, we need to 
        // check whether there are any pending tasks running. 

        // In addition, once the tasks run out, we continue to delay on-demand rendering
        // for another full frame so that the event dispatchers can catch up.

        int numTasks = getNumTasksRemaining();

        if ( numTasks > 0 )
            _onDemandDelay = 2;

        //OE_INFO << "Tasks = " << numTasks << std::endl;

        if ( _onDemandDelay > 0 )
        {
            osgGA::EventVisitor* ev = dynamic_cast<osgGA::EventVisitor*>( &nv );
            ev->getActionAdapter()->requestRedraw();
            _onDemandDelay--;
        }
    }

    osgTerrain::Terrain::traverse( nv );
}

TaskService*
CustomTerrain::createTaskService( const std::string& name, int id, int numThreads )
{
    ScopedLock<Mutex> lock( _taskServiceMutex );

    // first, double-check that the service wasn't created during the locking process:
    TaskServiceMap::iterator itr = _taskServices.find(id);
    if (itr != _taskServices.end())
        return itr->second.get();

    // ok, make a new one
    TaskService* service =  new TaskService( name, numThreads );
    _taskServices[id] = service;
    return service;
}

TaskService*
CustomTerrain::getTaskService(int id)
{
    ScopedLock<Mutex> lock( _taskServiceMutex );
    TaskServiceMap::iterator itr = _taskServices.find(id);
    if (itr != _taskServices.end())
    {
        return itr->second.get();
    }
    return NULL;
}

#define ELEVATION_TASK_SERVICE_ID 9999
#define TILE_GENERATION_TASK_SERVICE_ID 10000

TaskService*
CustomTerrain::getElevationTaskService()
{
    TaskService* service = getTaskService( ELEVATION_TASK_SERVICE_ID );
    if (!service)
    {
        service = createTaskService( "elevation", ELEVATION_TASK_SERVICE_ID, 1 );
    }
    return service;
}


TaskService*
CustomTerrain::getImageryTaskService(int layerId)
{
    TaskService* service = getTaskService( layerId );
    if (!service)
    {
        std::stringstream buf;
        buf << "layer " << layerId;
        std::string bufStr = buf.str();
        service = createTaskService( bufStr, layerId, 1 );
    }
    return service;
}

TaskService*
CustomTerrain::getTileGenerationTaskSerivce()
{
    TaskService* service = getTaskService( TILE_GENERATION_TASK_SERVICE_ID );
    if (!service)
    {
        int numCompileThreads = 
            _loadingPolicy.numCompileThreads().isSet() ? osg::maximum( 1, _loadingPolicy.numCompileThreads().value() ) :
            (int)osg::maximum( 1.0f, _loadingPolicy.numCompileThreadsPerCore().value() * (float)GetNumberOfProcessors() );

        service = createTaskService( "tilegen", TILE_GENERATION_TASK_SERVICE_ID, numCompileThreads );
    }
    return service;
}

void
CustomTerrain::updateTaskServiceThreads( const MapFrame& mapf )
{
    //Get the maximum elevation weight
    float elevationWeight = 0.0f;
    for (ElevationLayerVector::const_iterator itr = mapf.elevationLayers().begin(); itr != mapf.elevationLayers().end(); ++itr)
    {
        ElevationLayer* layer = itr->get();
        float w = layer->getTerrainLayerOptions().loadingWeight().value();
        if (w > elevationWeight) elevationWeight = w;
    }

    float totalImageWeight = 0.0f;
    for (ImageLayerVector::const_iterator itr = mapf.imageLayers().begin(); itr != mapf.imageLayers().end(); ++itr)
    {
        totalImageWeight += itr->get()->getTerrainLayerOptions().loadingWeight().value();
    }

    float totalWeight = elevationWeight + totalImageWeight;

    if (elevationWeight > 0.0f)
    {
        //Determine how many threads each layer gets
        int numElevationThreads = (int)osg::round((float)_numLoadingThreads * (elevationWeight / totalWeight ));
        OE_INFO << LC << "Elevation Threads = " << numElevationThreads << std::endl;
        getElevationTaskService()->setNumThreads( numElevationThreads );
    }

    for (ImageLayerVector::const_iterator itr = mapf.imageLayers().begin(); itr != mapf.imageLayers().end(); ++itr)
    {
        const TerrainLayerOptions& opt = itr->get()->getTerrainLayerOptions();
        int imageThreads = (int)osg::round((float)_numLoadingThreads * (opt.loadingWeight().value() / totalWeight ));
        OE_INFO << LC << "Image Threads for " << itr->get()->getName() << " = " << imageThreads << std::endl;
        getImageryTaskService( itr->get()->getId() )->setNumThreads( imageThreads );
    }
}
