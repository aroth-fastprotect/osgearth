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
#if 0
#include <osgEarth/TextureCompositorTex3D>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osg/Texture3D>
#include <vector>

using namespace osgEarth;

#define LC "[TextureCompositorTex3D] "

//------------------------------------------------------------------------

namespace
{
    // Records the information about a layer's texture space
    struct LayerTexRegion
    {
        LayerTexRegion() :
            _px(0), _py(0),
            _pw(256), _ph(256),
            _tx(0.0f), _ty(0.0f),
            _tw(1.0f), _th(1.0f),
            _xoffset(0.0f), _yoffset(0.0f),
            _xscale(1.0f), _yscale(1.0f)
        {
            //nop
        }

        // pixel coordinates of layer in the composite image:
        int _px, _py, _pw, _ph;

        // texture coordinates of layer in the composite image:
        float _tx, _ty, _tw, _th;

        // texture scale and offset for this region:
        float _xoffset, _yoffset, _xscale, _yscale;
    };
    typedef std::vector<LayerTexRegion> LayerTexRegionList;
}

//------------------------------------------------------------------------

static char s_source_vertMain[] =

    "varying vec3 normal, lightDir, halfVector; \n"

    "void main(void) \n"
    "{ \n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0; \n"
    "    gl_Position = ftransform(); \n"
    "} \n";

//------------------------------------------------------------------------

#if 0
static char s_source_fragMain[] =

    "uniform float osgearth_region[256]; \n"
    "uniform int   osgearth_region_count; \n"
    "uniform sampler3D tex0; \n"

    "uniform float osgearth_imagelayer_opacity[128]; \n"

    "void main(void) \n"
    "{ \n"
    "    vec3 color = vec3(1,1,1); \n"
    "    for(int i=0; i<osgearth_region_count; i++) \n"
    "    { \n"
    "        int j = 8*i; \n"
    "        float tx   = osgearth_region[j];   \n"
    "        float ty   = osgearth_region[j+1]; \n"
    "        float tw   = osgearth_region[j+2]; \n"
    "        float th   = osgearth_region[j+3]; \n"
    "        float xoff = osgearth_region[j+4]; \n"
    "        float yoff = osgearth_region[j+5]; \n"
    "        float xsca = osgearth_region[j+6]; \n"
    "        float ysca = osgearth_region[j+7]; \n"

    "        float opac = osgearth_imagelayer_opacity[i]; \n"

    "        float u = tx + ( xoff + xsca * gl_TexCoord[0].s ) * tw; \n"
    "        float v = ty + ( yoff + ysca * gl_TexCoord[0].t ) * th; \n"

    "        float w = (float)i/(float)osgearth_region_count; \n"
    "        vec4 texel = texture3D( tex0, vec3(u,v,w) ); \n"
    "        color = mix(color, texel.rgb, texel.a * opac); \n"
    "    } \n"
    "    gl_FragColor = vec4(color, 1); \n"
    "} \n";
#endif

static std::string
s_createFragShader( int numImageLayers )
{
    std::stringstream buf;

    buf << "uniform float osgearth_region[" << numImageLayers*8 << "]; \n"
        << "uniform sampler3D tex0; \n"
        << "uniform float osgearth_imagelayer_opacity[" << numImageLayers << "]; \n"

        << "void main(void) \n"
        << "{ \n"
        <<     "vec3 color = vec3(1,1,1); \n"
        <<     "float u, v, w; \n"
        <<     "vec4 texel; \n";

    for(int i=0; i<numImageLayers; ++i)
    {
        int j = i*8;
        buf << "u = osgearth_region["<< j <<"] + (osgearth_region["<< j+4 <<"] + osgearth_region["<< j+6 <<"] * gl_TexCoord[0].s) * osgearth_region["<< j+2 << "]; \n"
            << "v = osgearth_region["<< j+1 <<"] + (osgearth_region["<< j+5 <<"] + osgearth_region["<< j+7 <<"] * gl_TexCoord[0].t) * osgearth_region["<< j+3 << "]; \n"
            << "texel = texture3D( tex0, vec3(u,v,(float)"<< i <<"/(float)"<< numImageLayers <<") ); \n"
            << "color = mix(color, texel.rgb, texel.a * osgearth_imagelayer_opacity["<< i << "]); \n";
    }

    buf <<     "gl_FragColor = vec4(color,1); \n"
        << "} \n";

    std::string str = buf.str();
    return str;
}


//------------------------------------------------------------------------

osg::StateSet*
TextureCompositorTex3D::createStateSet( const GeoImageVector& layerImages, const GeoExtent& tileExtent ) const
{
    osg::StateSet* stateSet = new osg::StateSet();

    // Composite all the image layer images into a single composite image.
    //
    // NOTE!
    // This should work if images are different sizes, BUT it will NOT work if they use
    // different locators. In other words, this will only work if the texture coordinate
    // pair (u,v) is the SAME across all image layers for a given vertex. That's because
    // GLSL will only support one tex-coord pair per texture unit, and we are doing the
    // compositing so we only need to use one texture unit.

    LayerTexRegionList regions;

    int texWidth = 0, texHeight = 0;

    typedef std::vector<osg::ref_ptr<osg::Image> > ImageVec;
    ImageVec images;
    images.reserve( layerImages.size() );

    int layerNum = 0;
    for( GeoImageVector::const_iterator i = layerImages.begin(); i != layerImages.end(); ++i, ++layerNum )
    {
        const GeoImage& geoImage = *i;
        osg::ref_ptr<osg::Image> image = geoImage.getImage();

        // Because all tex2darray layers must be identical in format:
        if ( image->getPixelFormat() != GL_RGBA )
            image = ImageUtils::convertToRGBA( image );

        // TODO: reconsider.. perhaps grow the tex to the max layer size instead?
        if ( image->s() != 256 || image->t() != 256 )
            image = ImageUtils::resizeImage( image, 256, 256 );

        // add the layer image to the composite.
        images.push_back( image.get() );

        // TODO: optimize this away
        LayerTexRegion region;
        region._px = 0;
        region._py = 0;
        region._pw = image->s();
        region._ph = image->t();

        // track the maximum texture size
        if ( image->s() > texWidth )
            texWidth = image->s();

        if ( image->t() > texHeight )
            texHeight = image->t();
            
        // record the proper texture offset/scale for this layer. this accounts for subregions that
        // are used when referencing lower LODs.
        const GeoExtent& layerExtent = geoImage.getExtent();

        region._xoffset = (tileExtent.xMin() - layerExtent.xMin()) / layerExtent.width();
        region._yoffset = (tileExtent.yMin() - layerExtent.yMin()) / layerExtent.height();

        region._xscale = tileExtent.width() / layerExtent.width();
        region._yscale = tileExtent.height() / layerExtent.height();
            
        regions.push_back( region );
    }

    // build an image stack:
    osg::Image* image = new osg::Image();
    image->allocateImage( texWidth, texHeight, layerImages.size(), GL_RGBA, GL_UNSIGNED_BYTE );
    int r=0;
    for( ImageVec::const_iterator i=images.begin(); i != images.end(); ++i, ++r )
        ImageUtils::copyAsSubImage( i->get(), image, 0, 0, r );

    // put it in a 3D texture:
    osg::Texture3D* texture = new osg::Texture3D( image );

    // no mipmapping for a 3D texture
    texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );

    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);

    // build the uniforms.
    //    
    // The uniform array contains 8 floats for each region:
    //   tx, ty : origin texture coordinates in the composite-image space
    //   tw, th : width and height in composite-image space
    //   xoff, yoff : x- and y- offsets within texture space
    //   xsca, ysca : x- and y- scale factors within texture space
    osg::Uniform* texInfoArray = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_region", regions.size() * 8 );
    int p=0;
    for( unsigned int i=0; i<regions.size(); ++i )
    {
        LayerTexRegion& region = regions[i];

        // next calculate the texture space extents and store those in uniforms.
        // (GW: there is no actual reason to store these in the region structure)
        region._tx = (float)region._px/(float)texWidth;
        region._ty = (float)region._py/(float)texHeight;
        region._tw = (float)region._pw/(float)texWidth;
        region._th = (float)region._ph/(float)texHeight;

        texInfoArray->setElement( p++, region._tx );
        texInfoArray->setElement( p++, region._ty );
        texInfoArray->setElement( p++, region._tw );
        texInfoArray->setElement( p++, region._th );
        texInfoArray->setElement( p++, region._xoffset );
        texInfoArray->setElement( p++, region._yoffset );
        texInfoArray->setElement( p++, region._xscale );
        texInfoArray->setElement( p++, region._yscale );

        //OE_NOTICE << LC
        //    << "Region " << i << ": size=(" << region._pw << ", " << region._ph << ")" << std::endl;
    }

    stateSet->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON ); //TODO: un-hard-code the texture unit, perhaps
    stateSet->addUniform( texInfoArray );
    stateSet->getOrCreateUniform( "osgearth_region_count", osg::Uniform::INT )->set( (int)regions.size() );

    return stateSet;
}

void
TextureCompositorTex3D::updateMasterStateSet( osg::StateSet* stateSet, int numImageLayers ) const
{
    osg::Program* program = new osg::Program();
    program->addShader( new osg::Shader( osg::Shader::VERTEX, s_source_vertMain ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, s_createFragShader(numImageLayers) ) );
    stateSet->setAttributeAndModes( program, osg::StateAttribute::ON );
}
#endif
