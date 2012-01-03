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
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/FeatureSourceMeshConsolidator>
#include <osgEarthSymbology/TextSymbol>
#include <osgEarthSymbology/PointSymbol>
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/PolygonSymbol>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarth/ECEF>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/LineStipple>
#include <osg/Point>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/ClusterCullingCallback>
#include <osgText/Text>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>
#include <osgDB/WriteFile>
#include <osg/Version>

#define LC "[BuildGeometryFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

namespace
{
    void applyLineSymbology( osg::StateSet* stateSet, const LineSymbol* lineSymbol )
    {
        if ( lineSymbol )
        {
            float width = *lineSymbol->stroke()->width();
            stateSet->setAttributeAndModes(new osg::LineWidth(width), 1);
            if ( lineSymbol->stroke()->stipple().isSet() )
            {
                stateSet->setAttributeAndModes(
                    new osg::LineStipple(1, *lineSymbol->stroke()->stipple()) );
            }
        }
    }
}


BuildGeometryFilter::BuildGeometryFilter( const Style& style ) :
_style        ( style ),
_maxAngle_deg ( 5.0 ),
_geoInterp    ( GEOINTERP_RHUMB_LINE ),
_mergeGeometry( false )
{
    reset();
}

void
BuildGeometryFilter::reset()
{
    _result = 0L;
    _geode = new osg::Geode();
    _featureNode = new FeatureSourceMultiNode;
    _hasLines = false;
    _hasPoints = false;
    _hasPolygons = false;
}

bool
BuildGeometryFilter::process( FeatureList& features, const FilterContext& context )
{
    bool makeECEF = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* mapSRS = 0L;

    if ( context.isGeoreferenced() )
    {
        makeECEF   = context.getSession()->getMapInfo().isGeocentric();
        featureSRS = context.extent()->getSRS();
        mapSRS     = context.getSession()->getMapInfo().getProfile()->getSRS();
    }

    for( FeatureList::iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();

        GeometryIterator parts( input->getGeometry(), false );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();

            const Style& myStyle = input->style().isSet() ? *input->style() : _style;

            bool  setLinePropsHere   = input->style().isSet(); // otherwise it will be set globally, we assume
            float width              = 1.0f;
            bool  hasPolyOutline     = false;

            const LineSymbol*    lineSymbol = myStyle.get<LineSymbol>();
            const PolygonSymbol* polySymbol = myStyle.get<PolygonSymbol>();

            // resolve the geometry type from the component type and the symbology:
            Geometry::Type renderType;

            if ((polySymbol != 0L) ||
                (polySymbol == 0L && lineSymbol == 0L && part->getType() == Geometry::TYPE_POLYGON))
            {
                renderType = Geometry::TYPE_POLYGON;
            }

            else if (
                (lineSymbol != 0L && polySymbol == 0L && !part->isLinear()))
            {
                renderType = Geometry::TYPE_RING;
            }

            else if (
                (lineSymbol != 0L && polySymbol == 0L && part->isLinear()))
            {
                renderType = part->getType();
            }

            else
            {
                renderType = part->getType();
            }

            // resolve the color:
            osg::Vec4f primaryColor =
                polySymbol ? polySymbol->fill()->color() :
                lineSymbol ? lineSymbol->stroke()->color() :
                osg::Vec4f(1,1,1,1);

#if 0
            switch( part->getType() )
            {
            case Geometry::TYPE_POINTSET:
                {
                    _hasPoints = true;
                    primMode = osg::PrimitiveSet::POINTS;
                    const PointSymbol* point = myStyle.getSymbol<PointSymbol>();
                    if (point)
                    {
                        primaryColor = point->fill()->color();
                    }
                }
                break;

            case Geometry::TYPE_LINESTRING:
                {
                    _hasLines = true;
                    primMode = osg::PrimitiveSet::LINE_STRIP;
                    if ( line )
                    {
                        primaryColor = line->stroke()->color();
                        width = line->stroke()->width().isSet() ? *line->stroke()->width() : 1.0f;
                    }
                }
                break;

            case Geometry::TYPE_RING:
                {
                    _hasLines = true;
                    primMode = osg::PrimitiveSet::LINE_LOOP;
                    if ( line )
                    {
                        primaryColor = line->stroke()->color();
                        width = line->stroke()->width().isSet() ? *line->stroke()->width() : 1.0f;
                    }
                }
                break;

            case Geometry::TYPE_POLYGON:
                {
                    primMode = osg::PrimitiveSet::LINE_LOOP; // loop will tessellate into polys
                    const PolygonSymbol* poly = myStyle.getSymbol<PolygonSymbol>();
                    if (poly)
                    {
                        _hasPolygons = true;
                        primaryColor = poly->fill()->color();
                    }

                    if (line)
                    {
                        // if we have a line symbol and no polygon symbol, draw as an outline.
                        _hasLines = true;
                        width = line->stroke()->width().isSet() ? *line->stroke()->width() : 1.0f;
                    }
                }
                break;
		    default:
			    break;
            }
#endif
            
            osg::Geometry* osgGeom = new osg::Geometry();

            if ( _featureNameExpr.isSet() )
            {
                const std::string& name = input->eval( _featureNameExpr.mutable_value(), &context );
                osgGeom->setName( name );
            }

            // build the geometry:
            osg::Vec3Array* allPoints = 0L;

            if ( renderType == Geometry::TYPE_POLYGON )
            {
                buildPolygon(part, featureSRS, mapSRS, makeECEF, osgGeom);
                allPoints = static_cast<osg::Vec3Array*>( osgGeom->getVertexArray() );
            }
            else
            {
                // line geometry
                GLenum primMode = 
                    renderType == Geometry::TYPE_LINESTRING ? GL_LINE_STRIP :
                    renderType == Geometry::TYPE_RING       ? GL_LINE_LOOP :
                    GL_POINTS;
                allPoints = new osg::Vec3Array();
                transformAndLocalize( part->asVector(), featureSRS, allPoints, mapSRS, _world2local, makeECEF );
                osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, 0, part->size() ) );
                osgGeom->setVertexArray( allPoints );

                applyLineSymbology( osgGeom->getOrCreateStateSet(), lineSymbol );
            }

            // assign the primary color:
            osg::Vec4Array* colors = new osg::Vec4Array( allPoints->size() );
            for(unsigned c=0; c<colors->size(); ++c)
                (*colors)[c] = primaryColor;
            osgGeom->setColorArray( colors );
            osgGeom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

            // subdivide the mesh if necessary to conform to an ECEF globe:
            if ( makeECEF && renderType != Geometry::TYPE_POINTSET )
            {
                double threshold = osg::DegreesToRadians( *_maxAngle_deg );

                MeshSubdivider ms( _world2local, _local2world );
                //ms.setMaxElementsPerEBO( INT_MAX );
                if ( input->geoInterp().isSet() )
                    ms.run( *osgGeom, threshold, *input->geoInterp() );
                else
                    ms.run( *osgGeom, threshold, *_geoInterp );
            }

            // add the part to the geode.
            _featureNode->addDrawable(osgGeom, input->getFID());
            _geode->addDrawable( osgGeom );

            // build secondary geometry, if necessary (polygon outlines)
            if ( renderType == Geometry::TYPE_POLYGON && lineSymbol )
            {
                osg::Geometry*  outline = new osg::Geometry();
                osg::Vec3Array* outlineVerts = new osg::Vec3Array();
                outline->setVertexArray(outlineVerts);

                transformAndLocalize( part->asVector(), featureSRS, outlineVerts, mapSRS, _world2local, makeECEF );
                outline->addPrimitiveSet( new osg::DrawArrays( GL_LINE_LOOP, 0, part->size() ) );
                
                osg::Vec4Array* outlineColors = new osg::Vec4Array();
                outline->setColorArray(outlineColors);
                outline->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

                osg::Vec4f outlineColor = lineSymbol->stroke()->color();
                outlineColors->reserve( colors->size() + part->size() + 1 );
                for( unsigned c=0; c < part->size() + 1; ++c )
                    outlineColors->push_back( outlineColor );

                // subdivide if necessary.
                if ( makeECEF )
                {
                    double threshold = osg::DegreesToRadians( *_maxAngle_deg );
                    MeshSubdivider ms( _world2local, _local2world );
                    if ( input->geoInterp().isSet() )
                        ms.run( *outline, threshold, *input->geoInterp() );
                    else
                        ms.run( *outline, threshold, *_geoInterp );
                }

                applyLineSymbology( outline->getOrCreateStateSet(), lineSymbol );

                _geode->addDrawable( outline );
            }

        }
    }
    
    return true;
}

// builds and tessellates a polygon (with or without holes)
void
BuildGeometryFilter::buildPolygon(Geometry*               ring,
                                  const SpatialReference* featureSRS,
                                  const SpatialReference* mapSRS,
                                  bool                    makeECEF,
                                  osg::Geometry*          osgGeom)
{
    int totalPoints = ring->getTotalPointCount();
    osg::Vec3Array* allPoints = new osg::Vec3Array();
    transformAndLocalize( ring->asVector(), featureSRS, allPoints, mapSRS, _world2local, makeECEF );

    osgGeom->addPrimitiveSet( new osg::DrawArrays( GL_LINE_LOOP, 0, ring->size() ) );

    Polygon* poly = dynamic_cast<Polygon*>(ring);
    if ( poly )
    {
        int offset = ring->size();

        for( RingCollection::const_iterator h = poly->getHoles().begin(); h != poly->getHoles().end(); ++h )
        {
            Geometry* hole = h->get();
            if ( hole->isValid() )
            {
                transformAndLocalize( hole->asVector(), featureSRS, allPoints, mapSRS, _world2local, makeECEF );

                osgGeom->addPrimitiveSet( new osg::DrawArrays( GL_LINE_LOOP, offset, hole->size() ) );
                offset += hole->size();
            }
        }
    }
    osgGeom->setVertexArray( allPoints );

    osgUtil::Tessellator tess;
    tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
    tess.retessellatePolygons( *osgGeom );
}


osg::Node*
BuildGeometryFilter::push( FeatureList& input, FilterContext& context )
{
    reset();

    computeLocalizers( context );

    bool ok = process( input, context );

    // convert all geom to triangles and consolidate into minimal set of Geometries
    if ( !_featureNameExpr.isSet() )
    {
        FeatureSourceMeshConsolidator::run( *_geode.get(), _featureNode );
    }

    osg::Node* result = 0L;

    if ( ok )
    {
        if ( !_style.empty() && _geode.valid() )
        {
            // could optimize this to only happen is lines or points were created ..
            const LineSymbol* lineSymbol = _style.getSymbol<LineSymbol>();
            float size = 1.0;
            if (lineSymbol)
                size = lineSymbol->stroke()->width().value();

            _geode->getOrCreateStateSet()->setAttribute( new osg::Point(size), osg::StateAttribute::ON );
            _geode->getOrCreateStateSet()->setAttribute( new osg::LineWidth(size), osg::StateAttribute::ON );

            const PointSymbol* pointSymbol = _style.getSymbol<PointSymbol>();
            if ( pointSymbol && pointSymbol->size().isSet() )
                _geode->getOrCreateStateSet()->setAttribute( 
                    new osg::Point( *pointSymbol->size() ), osg::StateAttribute::ON );
        }

        _featureNode->addChild(_geode.release());

        result = delocalize( _featureNode.release() );
    }
    else
    {
        result = 0L;
    }

    return result;
}
