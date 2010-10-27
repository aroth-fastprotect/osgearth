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
#include "OSGTerrainEngineNode"
#include "SinglePassTerrainTechnique"
#include "CustomTerrain"
#include "MultiPassTerrainTechnique"
#include "TransparentLayer"

#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/ShaderComposition>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>

#define LC "[OSGTerrainEngine] "

using namespace osgEarth;

//------------------------------------------------------------------------

// adapter that lets OSGTerrainEngineNode listen to Map events
struct OSGTerrainEngineNodeMapCallbackProxy : public MapCallback
{
    OSGTerrainEngineNodeMapCallbackProxy(OSGTerrainEngineNode* node) : _node(node) { }
    osg::observer_ptr<OSGTerrainEngineNode> _node;

    void onMapProfileEstablished( const Profile* profile ) {
        _node->onMapProfileEstablished(profile);
    }

    void onMapModelChanged( const MapModelChange& change ) {
        _node->onMapModelChanged( change );
    }
};

//---------------------------------------------------------------------------

//static
static OpenThreads::ReentrantMutex s_engineNodeCacheMutex;
static unsigned int s_engineNodeID = 0;
//Caches the MapNodes that have been created
typedef std::map<unsigned int, osg::observer_ptr<OSGTerrainEngineNode> > EngineNodeCache;

static
EngineNodeCache& getEngineNodeCache()
{
    static EngineNodeCache s_cache;
    return s_cache;
}

void
OSGTerrainEngineNode::registerEngine(OSGTerrainEngineNode* engineNode)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_engineNodeCacheMutex);
    getEngineNodeCache()[engineNode->_id] = engineNode;
    OE_INFO << LC << "Registered engine " << engineNode->_id << std::endl;
}

void
OSGTerrainEngineNode::unregisterEngine(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_engineNodeCacheMutex);
    EngineNodeCache::iterator k = getEngineNodeCache().find( id);
    if (k != getEngineNodeCache().end())
    {
        getEngineNodeCache().erase(k);
        OE_INFO << LC << "Unregistered engine " << id << std::endl;
    }
}

OSGTerrainEngineNode*
OSGTerrainEngineNode::getEngineById(unsigned int id)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(s_engineNodeCacheMutex);
    EngineNodeCache::const_iterator k = getEngineNodeCache().find( id);
    if (k != getEngineNodeCache().end()) return k->second.get();
    return 0;
}

unsigned int
OSGTerrainEngineNode::getId() const
{
    return _id;
}

//------------------------------------------------------------------------

OSGTerrainEngineNode::OSGTerrainEngineNode( const OSGTerrainEngineNode& rhs, const osg::CopyOp& op ) :
TerrainEngineNode( rhs, op ),
_terrain( 0L ),
_update_mapf( 0L ),
_cull_mapf( 0L )
{
    //nop
}

OSGTerrainEngineNode::~OSGTerrainEngineNode()
{
    unregisterEngine( _id );

    if ( _update_mapf )
        delete _update_mapf;

    if ( _cull_mapf )
        delete _cull_mapf;
}

void
OSGTerrainEngineNode::initialize( Map* map, const TerrainOptions& terrainOptions )
{
    TerrainEngineNode::initialize( map, terrainOptions );

    // Initialize the map frames. We need one for the update thread and one for the
    // cull thread. Someday we can detect whether these are actually the same thread
    // (depends on the viewer's threading mode).
    _update_mapf = new MapFrame( map, Map::TERRAIN_LAYERS, "osgterrain-update" );
    _cull_mapf   = new MapFrame( map, Map::TERRAIN_LAYERS, "osgterrain-cull" );

    // merge in the custom options:
    _terrainOptions.merge( terrainOptions );

    // genearte a new unique mapnode ID
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock( s_engineNodeCacheMutex );
        _id = s_engineNodeID++;
    }
    //_map->setId( _id );

    // handle an already-established map profile:
    if ( _update_mapf->getProfile() )
    {
        onMapProfileEstablished( _update_mapf->getProfile() );
    }

    if ( _terrain )
    {
        _update_mapf->sync();

        unsigned int index = 0;
        for( ElevationLayerVector::const_iterator i = _update_mapf->elevationLayers().begin(); i != _update_mapf->elevationLayers().end(); i++ )
        {
            addElevationLayer( i->get() );
        }

        index = 0;
        for( ImageLayerVector::const_iterator j = _update_mapf->imageLayers().begin(); j != _update_mapf->imageLayers().end(); j++ )
        {
            addImageLayer( j->get() );
        }
    }

    // install a layer callback for processing further map actions:
    map->addMapCallback( new OSGTerrainEngineNodeMapCallbackProxy(this) );

    // register me.
    registerEngine( this );
}

osg::BoundingSphere
OSGTerrainEngineNode::computeBound() const
{
    if ( _terrain )
        return _terrain->getBound();
    else
        return TerrainEngineNode::computeBound();
}

void
OSGTerrainEngineNode::onMapProfileEstablished( const Profile* mapProfile )
{
    OE_INFO << LC << "Map profile established" << std::endl;

    // set up the ellipsoid
    this->setCoordinateSystem( mapProfile->getSRS()->getInitString() );
    this->setFormat( mapProfile->getSRS()->getInitType() );
    if ( !mapProfile->getSRS()->isProjected() )
        this->setEllipsoidModel( new osg::EllipsoidModel( *mapProfile->getSRS()->getEllipsoid() ) );

    // create a factory for creating actual tile data
    _tileFactory = new OSGTileFactory( _id, *_cull_mapf, _terrainOptions );

    // go through and build the root nodesets.
    _terrain = new CustomTerrain(
        *_update_mapf, *_cull_mapf, _tileFactory.get(), *_terrainOptions.quickReleaseGLObjects() );

    this->addChild( _terrain );

    // set the initial properties from the options structure:
    _terrain->setVerticalScale( _terrainOptions.verticalScale().value() );
    _terrain->setSampleRatio( _terrainOptions.heightFieldSampleRatio().value() );

    OE_INFO << LC << "Sample ratio = " << _terrainOptions.heightFieldSampleRatio().value() << std::endl;

    // install the proper layer composition technique:
    _texCompositor = new TextureCompositor( _terrainOptions.compositingTechnique().value() );

    if ( _texCompositor->getTechnique() == TerrainOptions::COMPOSITING_MULTIPASS )
    {
        _terrain->setTerrainTechniquePrototype( new MultiPassTerrainTechnique() );
        // not going to use it:
        _texCompositor = 0L;
        OE_INFO << LC << "Compositing technique = MULTIPASS" << std::endl;
    }

    else 
    {
        CustomTerrainTechnique* tech = new SinglePassTerrainTechnique( _texCompositor.get() );

        // prepare the interpolation technique for generating triangles:
        if ( _terrainOptions.elevationInterpolation() == INTERP_TRIANGULATE )
            tech->setOptimizeTriangleOrientation( false );

        _terrain->setTerrainTechniquePrototype( tech );
    }

    // install the shader program, if applicable:
    installShaders();

    // apply any pending callbacks:
#if 0
    for( TerrainCallbackList::iterator c = _pendingTerrainCallbacks.begin(); c != _pendingTerrainCallbacks.end(); ++c )
    {
        terrain->addTerrainCallback( c->get() );
    }
    _pendingTerrainCallbacks.clear();
#endif

    // collect the tile keys comprising the root tiles of the terrain.
    std::vector< TileKey > keys;
    _update_mapf->getProfile()->getRootKeys( keys );

    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        // always load the root tiles completely; no deferring. -gw
        bool loadNow = true; //!_terrainOptions.getPreemptiveLOD();

        osg::Node* node = _tileFactory->createSubTiles( *_update_mapf, _terrain, keys[i], loadNow );
        if (node)
        {
            _terrain->addChild(node);
        }
        else
        {
            OE_WARN << LC << "Couldn't make tile for root key: " << keys[i].str() << std::endl;
        }
    }

    // we just added the root tiles, so mark the bound in need of recomputation.
    dirtyBound();
}

void
OSGTerrainEngineNode::onMapModelChanged( const MapModelChange& change )
{
    _update_mapf->sync();

    // dispatch the change handler
    if ( change.getLayer() )
    {
        switch( change.getAction() )
        {
        case MapModelChange::ADD_IMAGE_LAYER:
            addImageLayer( change.getImageLayer() ); break;
        case MapModelChange::REMOVE_IMAGE_LAYER:
            removeImageLayer( change.getFirstIndex() ); break;
        case MapModelChange::ADD_ELEVATION_LAYER:
            addElevationLayer( change.getElevationLayer() ); break;
        case MapModelChange::REMOVE_ELEVATION_LAYER:
            removeElevationLayer( change.getFirstIndex() ); break;
        case MapModelChange::MOVE_IMAGE_LAYER:
            moveImageLayer( change.getFirstIndex(), change.getSecondIndex() ); break;
        case MapModelChange::MOVE_ELEVATION_LAYER:
            moveElevationLayer( change.getFirstIndex(), change.getSecondIndex() ); break;
        }
    }

    // update the terrain revision in threaded mode
    if ( _terrainOptions.loadingPolicy()->mode() != LoadingPolicy::MODE_STANDARD )
    {
        getTerrain()->incrementRevision();
        getTerrain()->updateTaskServiceThreads( *_update_mapf );
    }
}

void
OSGTerrainEngineNode::addImageLayer( ImageLayer* layer )
{
    if ( !layer || !layer->getTileSource() )
        return;

    // visit all existing terrain tiles and inform each one of the new image layer:
    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        Threading::ScopedWriteLock tileLock(tile->getTileLayersMutex());

        //Create a TileKey from the TileID
        osgTerrain::TileID tileId = tile->getTileID();
        TileKey key( TileKey::getLOD(tileId), tileId.x, tileId.y, _update_mapf->getProfile() );

        GeoImage geoImage;

        bool needToUpdateImagery = false;
        int imageLOD = -1;

        // establish the initial image for this tile.
        //if (( _options.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD ) ||
        //   (( _options.loadingPolicy()->mode() == LoadingPolicy::MODE_SEQUENTIAL) && key.getLevelOfDetail() == 1))

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD ||
            key.getLevelOfDetail() == 1)
        {
            // in standard mode, or at the first LOD in seq/pre mode, fetch the image immediately.
            geoImage = _tileFactory->createValidGeoImage( layer, key );
            imageLOD = key.getLevelOfDetail();
        }
        else
        {
            // in seq/pre mode, set up a placeholder and mark the tile as dirty.
            geoImage = GeoImage(ImageUtils::createEmptyImage(), key.getExtent() );
            needToUpdateImagery = true;
        }

        if (geoImage.valid())
        {
            const MapInfo& mapInfo = _update_mapf->getMapInfo();

            double img_min_lon, img_min_lat, img_max_lon, img_max_lat;
            geoImage.getExtent().getBounds(img_min_lon, img_min_lat, img_max_lon, img_max_lat);

            //Specify a new locator for the color with the coordinates of the TileKey that was actually used to create the image
            osg::ref_ptr<GeoLocator> img_locator = key.getProfile()->getSRS()->createLocator( 
                img_min_lon, img_min_lat, img_max_lon, img_max_lat, 
                !mapInfo.isGeocentric() );
            
            //Set the CS to geocentric if we are dealing with a geocentric map
            if ( mapInfo.isGeocentric() )
            {
                img_locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
            }

            // Create a layer wrapper that supports opacity.
            // TODO: review this; the Transparent layer holds a back-reference to the actual MapLayer
            TransparentLayer* img_layer = new TransparentLayer( 
                geoImage.getImage(), 
                _update_mapf->imageLayers()[ _update_mapf->imageLayers().size()-1 ] );

            img_layer->setLevelOfDetail(imageLOD);
            img_layer->setLocator( img_locator.get());
            img_layer->setMinFilter( layer->getImageLayerOptions().minFilter().value() );
            img_layer->setMagFilter( layer->getImageLayerOptions().magFilter().value() );

            unsigned int newLayer = _update_mapf->imageLayers().size() - 1;
            tile->setColorLayer( newLayer, img_layer );

            if (needToUpdateImagery)
            {
                tile->updateImagery( layer->getId(), *_update_mapf, _tileFactory.get());
            }
        }
        else
        {
            // this can happen if there's no data in the new layer for the given tile.
            // we will rely on the driver to dump out a warning if this is an error.

            //OE_INFO << LC << 
            //    "Adding layer " << layer->getName()
            //    << ": Could not create geoimage for tile " << key.str() << std::endl;
        }

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
            tile->setDirty(true);
        else
            tile->queueTileUpdate( TileUpdate::UPDATE_ALL_IMAGE_LAYERS );
    }

    updateTextureCombining();
}

void
OSGTerrainEngineNode::updateElevation(CustomTile* tile)
{
    Threading::ScopedWriteLock tileLock( tile->getTileLayersMutex() );

    const TileKey& key = tile->getKey();

    bool hasElevation;
    {
        hasElevation = _update_mapf->elevationLayers().size() > 0;
    }    

    //Update the elevation hint
    tile->setHasElevationHint( hasElevation );

    osgTerrain::HeightFieldLayer* heightFieldLayer = dynamic_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    if (heightFieldLayer)
    {
        //In standard mode, just load the elevation data and dirty the tile.

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
        {
            osg::ref_ptr<osg::HeightField> hf;
            if (hasElevation)
            {
                hf = _update_mapf->createHeightField( key, true, _terrainOptions.elevationInterpolation().value());
            }
            if (!hf.valid()) hf = OSGTileFactory::createEmptyHeightField( key );
            heightFieldLayer->setHeightField( hf.get() );
            hf->setSkirtHeight( tile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );
            tile->setDirty(true);
        }
        else
        {
            //In preemptive mode, if there is no elevation, just clear out all the elevation on the tiles
            if (!hasElevation)
            {
                osg::ref_ptr<osg::HeightField> hf = OSGTileFactory::createEmptyHeightField( key );
                heightFieldLayer->setHeightField( hf.get() );
                hf->setSkirtHeight( tile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );
                tile->setElevationLOD( key.getLevelOfDetail() );
                tile->resetElevationRequests( *_update_mapf );
                tile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
            }
            else
            {
                //Always load the first LOD so the children tiles can have something to use for placeholders
                if (tile->getKey().getLevelOfDetail() == 1)
                {
                    osg::ref_ptr<osg::HeightField> hf = _update_mapf->createHeightField( key, true, _terrainOptions.elevationInterpolation().value());
                    if (!hf.valid()) hf = OSGTileFactory::createEmptyHeightField( key );
                    heightFieldLayer->setHeightField( hf.get() );
                    hf->setSkirtHeight( tile->getBound().radius() * _terrainOptions.heightFieldSkirtRatio().value() );
                    tile->setElevationLOD(tile->getKey().getLevelOfDetail());
                    tile->queueTileUpdate( TileUpdate::UPDATE_ELEVATION );
                }
                else
                {
                    //Set the elevation LOD to -1
                    tile->setElevationLOD(-1);
                    tile->resetElevationRequests( *_update_mapf );
                }
            }
        }
    }
}


void
OSGTerrainEngineNode::addElevationLayer( ElevationLayer* layer )
{
    if ( !layer || !layer->getTileSource() )
        return;

    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );

    OE_DEBUG << LC << "Found " << tiles.size() << std::endl;

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        updateElevation( tile );
    }
}

void
OSGTerrainEngineNode::removeImageLayer( unsigned int index )
{
    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        Threading::ScopedWriteLock tileLock(tile->getTileLayersMutex());

        //An image layer was removed, so reorganize the color layers in the tiles to account for it's removal
        std::vector< osg::ref_ptr< osgTerrain::Layer > > layers;
        for (unsigned int i = 0; i < itr->get()->getNumColorLayers(); ++i)
        {   
            //Skip the layer that is being removed
            if (i != index)
            {
                osgTerrain::Layer* imageLayer = itr->get()->getColorLayer(i);
                if (imageLayer)
                {
                    layers.push_back(imageLayer);
                }
            }
            //Set the current value to NULL
            itr->get()->setColorLayer( i, NULL);
        }

        //Reset the color layers to the correct order
        for (unsigned int i = 0; i < layers.size(); ++i)
        {
            itr->get()->setColorLayer( i, layers[i].get() );
        }

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
            tile->setDirty( true );
        else
            tile->queueTileUpdate( TileUpdate::UPDATE_ALL_IMAGE_LAYERS );
    }
    
    updateTextureCombining();

    OE_DEBUG << "[osgEarth::Map::removeImageSource] end " << std::endl;  
}

void
OSGTerrainEngineNode::removeElevationLayer( unsigned int index )
{
    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        updateElevation( tile );
    }
}

void
OSGTerrainEngineNode::moveImageLayer( unsigned int oldIndex, unsigned int newIndex )
{
    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        Threading::ScopedWriteLock tileLock(tile->getTileLayersMutex());

        //Collect the current color layers
        std::vector< osg::ref_ptr< osgTerrain::Layer > > layers;

        for (unsigned int i = 0; i < itr->get()->getNumColorLayers(); ++i)
        {              
            layers.push_back(itr->get()->getColorLayer(i));
        }

        //Swap the original position
        osg::ref_ptr< osgTerrain::Layer > layer = layers[oldIndex];
        layers.erase(layers.begin() + oldIndex);
        layers.insert(layers.begin() + newIndex, layer.get());

        for (unsigned int i = 0; i < layers.size(); ++i)
        {
            itr->get()->setColorLayer( i, layers[i].get() );
        }

        if ( _terrainOptions.loadingPolicy()->mode() == LoadingPolicy::MODE_STANDARD )
            tile->setDirty( true );
        else
            tile->queueTileUpdate( TileUpdate::UPDATE_ALL_IMAGE_LAYERS );
    }     

    updateTextureCombining();
}

void
OSGTerrainEngineNode::moveElevationLayer( unsigned int oldIndex, unsigned int newIndex )
{
    TerrainTileList tiles;
    _terrain->getTerrainTiles( tiles );
    OE_DEBUG << "Found " << tiles.size() << std::endl;

    for (TerrainTileList::iterator itr = tiles.begin(); itr != tiles.end(); ++itr)
    {
        CustomTile* tile = static_cast< CustomTile* >( itr->get() );
        updateElevation(tile);
    }
}

void
OSGTerrainEngineNode::validateTerrainOptions( TerrainOptions& options )
{
    TerrainEngineNode::validateTerrainOptions( options );
    
    //nop for now.
    //note: to validate plugin-specific features, we would create an OSGTerrainOptions
    // and do the validation on that. You would then re-integrate it by calling
    // options.mergeConfig( osgTerrainOptions ).
}

void
OSGTerrainEngineNode::traverse( osg::NodeVisitor& nv )
{
    // Note: no need to sync _update_mapf here, since it gets sync'd whenever there's a 
    // map callback.
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        _cull_mapf->sync();
    }

    else if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        // detect and respond to changes in the system shader library:
        ShaderFactory* sf = osgEarth::Registry::instance()->getShaderFactory();
        if ( sf->outOfSyncWith( _shaderLibRev ) )
        {
            OE_INFO << LC << "Detected shader factory change; updating." << std::endl;
            this->installShaders();
            sf->sync( _shaderLibRev );
        }
    }

    TerrainEngineNode::traverse( nv );
}

void
OSGTerrainEngineNode::installShaders()
{
    // This method installs a default shader setup on the engine node itself. The texture compositor
    // can then override parts of the program by using a VirtualProgram on the _terrain node. We do
    // it this way so that the developer has the option of removing this top-level shader program,
    // replacing it, or migrating it higher up the scene graph if necessary.

    if ( _texCompositor.valid() && _texCompositor->usesShaderComposition() )
    {
        const ShaderFactory* lib = Registry::instance()->getShaderFactory();

        int numLayers = osg::maximum( 1, (int)_update_mapf->imageLayers().size() );

        VirtualProgram* vp = new VirtualProgram();        

        vp->setShader( "osgearth_vert_main",     lib->createVertexShaderMain() );
        vp->setShader( "osgearth_vert_lighting", lib->createDefaultLightingVertexShader() );
        vp->setShader( "osgearth_vert_texture",  lib->createDefaultTextureVertexShader( numLayers ) );

        vp->setShader( "osgearth_frag_main",     lib->createFragmentShaderMain() );
        vp->setShader( "osgearth_frag_lighting", lib->createDefaultLightingFragmentShader() );
        vp->setShader( "osgearth_frag_texture",  lib->createDefaultTextureFragmentShader( numLayers ) );

        getOrCreateStateSet()->setAttributeAndModes( vp, osg::StateAttribute::ON );
    }
}

void
OSGTerrainEngineNode::updateTextureCombining()
{
    if ( _texCompositor.valid() )
    {
        int numImageLayers = _update_mapf->imageLayers().size();
        osg::StateSet* terrainStateSet = _terrain->getOrCreateStateSet();

        if ( _texCompositor->usesShaderComposition() )
        {
            // Creates or updates the shader components that are generated by the texture compositor.
            // These components reside in the CustomTerrain's stateset, and override the components
            // installed in the VP on the engine-node's stateset in installShaders().

            VirtualProgram* vp = dynamic_cast<VirtualProgram*>( terrainStateSet->getAttribute(osg::StateAttribute::PROGRAM) );

            if ( !vp )
            {
                // create and add it the first time around..
                vp = new VirtualProgram();
                terrainStateSet->setAttributeAndModes( vp, osg::StateAttribute::ON );
            }

            // first, update the default shader components based on the new layer count:
            const ShaderFactory* lib = Registry::instance()->getShaderFactory();
            vp->setShader( "osgearth_vert_texture",  lib->createDefaultTextureVertexShader( numImageLayers ) );

            // not this one, because the compositor always generates a new one.
            //vp->setShader( "osgearth_frag_texture",  lib.createDefaultTextureFragmentShader( numImageLayers ) );
        }

        // next, inform the compositor that it needs to update based on a new layer count:
        _texCompositor->updateGlobalStateSet( terrainStateSet, numImageLayers );
    }
}
