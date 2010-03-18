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

#include <osgEarth/ModelSource>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/Styling>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#include "FeatureGeomModelOptions"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

#define PROP_HEIGHT_OFFSET "height_offset"

//#define USE_SYMBOLOGY
#ifdef USE_SYMBOLOGY
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/GeometrySymbol>
#include <osgEarthSymbology/GeometrySymbolizer>
#include <osgEarthSymbology/GeometryInput>
#include <osgEarthSymbology/SymbolicNode>

class FeatureGeomModelSource : public FeatureModelSource
{
public:
    FeatureGeomModelSource( const PluginOptions* options, int sourceId ) : FeatureModelSource( options ),
        _sourceId( sourceId )
    {
        _options = dynamic_cast<const FeatureGeomModelOptions*>( options );
        if ( !_options )
            _options = new FeatureGeomModelOptions( options );
    }

    //override
    void initialize( const std::string& referenceURI, const osgEarth::Map* map )
    {
        FeatureModelSource::initialize( referenceURI, map );
        _map = map;
    }

    //override
    osg::Node* renderFeaturesForStyle( const Style& style, FeatureList& features, osg::Referenced* data, osg::Node** out_newNode )
    {
        // A processing context to use with the filters:
        FilterContext context;
        context.profile() = getFeatureSource()->getFeatureProfile();

        // Transform them into the map's SRS:
        TransformFilter xform( _map->getProfile()->getSRS(), _map->isGeocentric() );
        xform.heightOffset() = _options->heightOffset().value();
        context = xform.push( features, context );

        // Make the symbolic node:
        osgEarth::Symbology::SymbolicNode* symNode = new osgEarth::Symbology::SymbolicNode;

        osgEarth::Symbology::Style* sstyle = new osgEarth::Symbology::Style;
        sstyle->addSymbol(new osgEarth::Symbology::PointSymbol);
        sstyle->addSymbol(new osgEarth::Symbology::LineSymbol);
        sstyle->addSymbol(new osgEarth::Symbology::PolygonSymbol);

        symNode->setStyle(sstyle);
        symNode->setSymbolizer(new osgEarth::Symbology::GeometrySymbolizer);

        osgEarth::Symbology::GeometryInput* geoms = new osgEarth::Symbology::GeometryInput;
        symNode->setDataSet(geoms);

        for (FeatureList::iterator it = features.begin(); it != features.end(); ++it)
        {
            Feature* feature = it->get();
            if ( feature )
            {
                Geometry* geometry = feature->getGeometry();
                if ( geometry )
                {
                    // this is temporary until we finish migrating Geometry to ::Symbology
                    geoms->getGeometryList().push_back( 
                        osgEarth::Symbology::Geometry::create(
                            osgEarth::Symbology::Geometry::Type((int)(feature->getGeometry()->getType())), geometry));
                }
            }
        }

        osg::Node* result = symNode;

        // If the context specifies a reference frame, apply it to the resulting model.
        // Q: should this be here, or should the reference frame matrix be passed to the Symbolizer?
        // ...probably the latter.
        if ( context.hasReferenceFrame() )
        {
            osg::MatrixTransform* delocalizer = new osg::MatrixTransform(
                context.inverseReferenceFrame() );
            delocalizer->addChild( result );
            result = delocalizer;
        }

        // set the output node if necessary:
        if ( out_newNode )
            *out_newNode = result;

        return result;
    }

protected:
    osg::ref_ptr<const FeatureGeomModelOptions> _options;
    osg::ref_ptr<osgEarth::Symbology::SymbolicNode> _symbolic;
    int _sourceId;
    osg::ref_ptr<const Map> _map;
};


#else
class FeatureGeomModelSource : public FeatureModelSource
{
public:
    FeatureGeomModelSource( const PluginOptions* options, int sourceId ) : FeatureModelSource( options ),
        _sourceId( sourceId )
    {
        _options = dynamic_cast<const FeatureGeomModelOptions*>( options );
        if ( !_options )
            _options = new FeatureGeomModelOptions( options );
    }

    //override
    void initialize( const std::string& referenceURI, const osgEarth::Map* map )
    {
        FeatureModelSource::initialize( referenceURI, map );
        _map = map;
    }

    //override
    osg::Node* renderFeaturesForStyle( const Style& style, FeatureList& features, osg::Referenced* data, osg::Node** out_newNode )
    {
        // A processing context to use with the filters:
        FilterContext context;
        context.profile() = getFeatureSource()->getFeatureProfile();

        // Transform them into the map's SRS:
        TransformFilter xform( _map->getProfile()->getSRS(), _map->isGeocentric() );
        xform.heightOffset() = _options->heightOffset().value();
        context = xform.push( features, context );

        // Build geometry:
        BuildGeometryFilter build; 
        if ( _options->geometryTypeOverride().isSet() )
            build.geomTypeOverride() = _options->geometryTypeOverride().value();

        // apply the style rule if we have one:
        osg::ref_ptr<osg::Node> result;
        build.style() = style;
        context = build.push( features, result, context );
        
        // Apply an LOD if required:
        if ( _options->minRange().isSet() || _options->maxRange().isSet() )
        {
            osg::LOD* lod = new osg::LOD();
            lod->addChild( result.get(), _options->minRange().value(), _options->maxRange().value() );
            result = lod;
        }

        if ( out_newNode ) *out_newNode = result.get();
        return result.release();
    }

private:
    osg::ref_ptr<const FeatureGeomModelOptions> _options;
    int _sourceId;
    osg::ref_ptr<const Map> _map;
};

#endif

class FeatureGeomModelSourceFactory : public osgDB::ReaderWriter
{
public:
    FeatureGeomModelSourceFactory()
    {
        supportsExtension( "osgearth_model_feature_geom", "osgEarth feature geom plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Geom Model Plugin";
    }

    FeatureGeomModelSource* create( const PluginOptions* options )
    {
        ScopedLock<Mutex> lock( _sourceIdMutex );
        FeatureGeomModelSource* obj = new FeatureGeomModelSource( options, _sourceId );
        if ( obj ) _sourceMap[_sourceId++] = obj;
        return obj;
    }

    FeatureGeomModelSource* get( int sourceId )
    {
        ScopedLock<Mutex> lock( _sourceIdMutex );
        return _sourceMap[sourceId].get();
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        FeatureGeomModelSourceFactory* nonConstThis = const_cast<FeatureGeomModelSourceFactory*>(this);
        return nonConstThis->create( static_cast<const PluginOptions*>(options) );
    }

    // NOTE: this doesn't do anything, yet. it's a template for recursing into the
    // plugin during pagedlod traversals.
    virtual ReadResult readNode(const std::string& fileName, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( fileName )))
            return ReadResult::FILE_NOT_HANDLED;
   
        std::string stripped = osgDB::getNameLessExtension( fileName );
        int sourceId = 0;
        sscanf( stripped.c_str(), "%d", &sourceId );

        FeatureGeomModelSourceFactory* nonConstThis = const_cast<FeatureGeomModelSourceFactory*>(this);
        return ReadResult( nonConstThis->get( sourceId ) );
    }

protected:
    Mutex _sourceIdMutex;
    int _sourceId;
    std::map<int, osg::ref_ptr<FeatureGeomModelSource> > _sourceMap;
};

REGISTER_OSGPLUGIN(osgearth_model_feature_geom, FeatureGeomModelSourceFactory) 
