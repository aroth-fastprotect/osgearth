/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/Capabilities>
#include <osg/FragmentProgram>
#include <osg/GraphicsContext>
#include <osg/GL>
#include <osg/GLExtensions>
#include <osg/GL2Extensions>
#include <osg/Texture>
#include <osgViewer/Version>
#include <osgEarth/Config>
#include <osgEarth/XmlUtils>
#include <osgEarth/Version>

#include <fstream>

#ifdef _WIN32
extern "C" unsigned long __stdcall GetTempPathA(unsigned long nBufferLength,char * lpBuffer );
#endif

using namespace osgEarth;

#define LC "[Capabilities] "

// ---------------------------------------------------------------------------
// A custom P-Buffer graphics context that we will use to query for OpenGL 
// extension and hardware support. (Adapted from osgconv in OpenSceneGraph)

struct MyGraphicsContext
{
    MyGraphicsContext(bool quadBufferStereo)
    {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x = 0;
        traits->y = 0;
        traits->width = 1;
        traits->height = 1;
        traits->windowDecoration = false;
        traits->doubleBuffer = false;
        traits->sharedContext = 0;
        traits->pbuffer = false;
		traits->quadBufferStereo = quadBufferStereo;

        // Intel graphics adapters dont' support pbuffers, and some of their drivers crash when
        // you try to create them. So by default we will only use the unmapped/pbuffer method
        // upon special request.
        if ( getenv( "OSGEARTH_USE_PBUFFER_TEST" ) )
        {
            traits->pbuffer = true;
            OE_INFO << LC << "Activating pbuffer test for graphics capabilities" << std::endl;
            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
            if ( !_gc.valid() )
                OE_WARN << LC << "Failed to create pbuffer" << std::endl;
        }

        if (!_gc.valid())
        {
            // fall back on a mapped window
            traits->pbuffer = false;
            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
        }

        if (_gc.valid()) 
        {
            _gc->realize();
            _gc->makeCurrent();

            if ( traits->pbuffer == false )
            {
                OE_DEBUG << LC << "Realized graphics window for OpenGL operations." << std::endl;
            }
            else
            {
                OE_DEBUG << LC << "Realized pbuffer for OpenGL operations." << std::endl;
            }
        }
        else
        {
            OE_WARN << LC << "Failed to create graphic window too." << std::endl;
        }
    }

    bool valid() const { return _gc.valid() && _gc->isRealized(); }
	osg::GraphicsContext * gc() { return _gc.get(); }
private:
    osg::ref_ptr<osg::GraphicsContext> _gc;
};

// ---------------------------------------------------------------------------

#define SAYBOOL(X) (X?"yes":"no")

Capabilities::Capabilities() :
_maxFFPTextureUnits     ( 1 ),
_maxGPUTextureUnits     ( 1 ),
_maxGPUTextureCoordSets ( 1 ),
_maxTextureSize         ( 256 ),
_maxFastTextureSize     ( 256 ),
_maxLights              ( 1 ),
_depthBits              ( 0 ),
_supportsGLSL           ( false ),
_GLSLversion            ( 1.0f ),
_supportsTextureArrays  ( false ),
_supportsMultiTexture   ( false ),
_supportsStencilWrap    ( true ),
_supportsTwoSidedStencil( false ),
_supportsTexture2DLod   ( false ),
_supportsMipmappedTextureUpdates( false ),
_supportsDepthPackedStencilBuffer( false ),
_supportsQuadBufferStereo( false ),
_supportsOcclusionQuery ( false ),
_supportsDrawInstanced  ( false ),
_supportsUniformBufferObjects( false ),
_maxUniformBlockSize    ( 0 )
{
    // little hack to force the osgViewer library to link so we can create a graphics context
    osgViewerGetVersion();

    // check the environment in order to disable ATI workarounds
    bool enableATIworkarounds = true;
    if ( ::getenv( "OSGEARTH_DISABLE_ATI_WORKAROUNDS" ) != 0L )
        enableATIworkarounds = false;

	bool disableQuadBufferStereoTest = false;
    if ( ::getenv( "OSGEARTH_QUADBUFFER_DISABLE" ) != 0L )
        disableQuadBufferStereoTest = true;

    if(!readCache())
    {
        // only detect the hardware when the cache failed

        // first create a opengl context with quad buffer stereo enabled
        MyGraphicsContext * mgc;
	    if(!disableQuadBufferStereoTest)
        {
		    mgc = new MyGraphicsContext(true);
		    _supportsQuadBufferStereo = mgc->valid();
	    }
	    else
	    {
		    mgc = NULL;
		    _supportsQuadBufferStereo = false;
	    }
	    if(!_supportsQuadBufferStereo)
	    {
		    // delete the old context
		    if(mgc)
			    delete mgc;
		    // second try to create a new graphics context without quad buffer stereo
		    mgc = new MyGraphicsContext(false);
	    }

        if ( mgc->valid() )
        {
            osg::GraphicsContext* gc = mgc->gc();
            unsigned int id = gc->getState()->getContextID();
            const osg::GL2Extensions* GL2 = osg::GL2Extensions::Get( id, true );
            _vendor = std::string( reinterpret_cast<const char*>(glGetString(GL_VENDOR)) );
            _renderer = std::string( reinterpret_cast<const char*>(glGetString(GL_RENDERER)) );
            _version = std::string( reinterpret_cast<const char*>(glGetString(GL_VERSION)) );
            glGetIntegerv( GL_MAX_TEXTURE_UNITS, &_maxFFPTextureUnits );
            glGetIntegerv( GL_MAX_TEXTURE_IMAGE_UNITS_ARB, &_maxGPUTextureUnits );
            glGetIntegerv( GL_MAX_TEXTURE_COORDS_ARB, &_maxGPUTextureCoordSets );
            glGetIntegerv( GL_DEPTH_BITS, &_depthBits );

            // Use the texture-proxy method to determine the maximum texture size 
            glGetIntegerv( GL_MAX_TEXTURE_SIZE, &_maxTextureSize );
            for( int s = _maxTextureSize; s > 2; s >>= 1 )
            {
                glTexImage2D( GL_PROXY_TEXTURE_2D, 0, GL_RGBA8, s, s, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0L );
                GLint width = 0;
                glGetTexLevelParameteriv( GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width );
                if ( width == s )
                {
                    _maxTextureSize = s;
                    break;
                }
            }
            glGetIntegerv( GL_MAX_LIGHTS, &_maxLights );
            _supportsGLSL = GL2->isGlslSupported();

            if ( _supportsGLSL )
            {
                _GLSLversion = GL2->getLanguageVersion();
            }
            _supportsTextureArrays = 
                _supportsGLSL &&
                osg::getGLVersionNumber() >= 2.0 && // hopefully this will detect Intel cards
                osg::isGLExtensionSupported( id, "GL_EXT_texture_array" );
            _supportsTexture3D = osg::isGLExtensionSupported( id, "GL_EXT_texture3D" );
            _supportsMultiTexture = 
                osg::getGLVersionNumber() >= 1.3 ||
                osg::isGLExtensionSupported( id, "GL_ARB_multitexture") ||
                osg::isGLExtensionSupported( id, "GL_EXT_multitexture" );
            _supportsStencilWrap = osg::isGLExtensionSupported( id, "GL_EXT_stencil_wrap" );
            _supportsTwoSidedStencil = osg::isGLExtensionSupported( id, "GL_EXT_stencil_two_side" );
        	_supportsDepthPackedStencilBuffer = osg::isGLExtensionSupported( id, "GL_EXT_packed_depth_stencil" ) || 
            	                                osg::isGLExtensionSupported( id, "GL_OES_packed_depth_stencil" );
            
            _supportsDrawInstanced = osg::isGLExtensionOrVersionSupported( id, "GL_EXT_draw_instanced", 3.1f );
            _supportsUniformBufferObjects = osg::isGLExtensionOrVersionSupported( id, "GL_ARB_uniform_buffer_object", 2.0f );
	        if ( _supportsUniformBufferObjects && _maxUniformBlockSize == 0 )
                _supportsUniformBufferObjects = false;
    
            _supportsDepthPackedStencilBuffer = osg::isGLExtensionSupported( id, "GL_EXT_packed_depth_stencil" );
        	_supportsOcclusionQuery = osg::isGLExtensionSupported( id, "GL_ARB_occlusion_query" );

            // ATI workarounds:
            bool isATI = _vendor.find("ATI ") == 0;

            _supportsMipmappedTextureUpdates = isATI && enableATIworkarounds ? false : true;

#if 0
            // Intel workarounds:
            bool isIntel = 
                _vendor.find("Intel ")   != std::string::npos ||
                _vendor.find("Intel(R)") != std::string::npos ||
                _vendor.compare("Intel") == 0;
#endif

            _maxFastTextureSize = _maxTextureSize;
            glGetIntegerv( GL_MAX_UNIFORM_BLOCK_SIZE, &_maxUniformBlockSize );

        }
        // delete the final object of the graphics context
        delete mgc;

        writeCache();
    }

    OE_INFO << LC << "Detected hardware capabilities:" << std::endl;

    OE_INFO << LC << "  Vendor = " << _vendor << std::endl;

    OE_INFO << LC << "  Renderer = " << _renderer << std::endl;

    OE_INFO << LC << "  Version = " << _version << std::endl;

    OE_INFO << LC << "  Max FFP texture units = " << _maxFFPTextureUnits << std::endl;

    OE_INFO << LC << "  Max GPU texture units = " << _maxGPUTextureUnits << std::endl;

    OE_INFO << LC << "  Max GPU texture coordinate sets = " << _maxGPUTextureCoordSets << std::endl;

    OE_INFO << LC << "  Depth buffer bits = " << _depthBits << std::endl;

    OE_INFO << LC << "  Max texture size = " << _maxTextureSize << std::endl;

    OE_INFO << LC << "  Max lights = " << _maxLights << std::endl;

    OE_INFO << LC << "  GLSL = " << SAYBOOL(_supportsGLSL) << std::endl;

    if ( _supportsGLSL )
    {
        OE_INFO << LC << "  GLSL Version = " << _GLSLversion << std::endl;
    }

    OE_INFO << LC << "  Texture arrays = " << SAYBOOL(_supportsTextureArrays) << std::endl;

    OE_INFO << LC << "  depth-packed stencil = " << SAYBOOL(_supportsDepthPackedStencilBuffer) << std::endl;

    OE_INFO << LC << "  3D textures = " << SAYBOOL(_supportsTexture3D) << std::endl;

    OE_INFO << LC << "  Multitexturing = " << SAYBOOL(_supportsMultiTexture) << std::endl;

    OE_INFO << LC << "  Stencil wrapping = " << SAYBOOL(_supportsStencilWrap) << std::endl;

    OE_INFO << LC << "  2-sided stencils = " << SAYBOOL(_supportsTwoSidedStencil) << std::endl;

    OE_INFO << LC << "  draw instanced = " << SAYBOOL(_supportsDrawInstanced) << std::endl;

    OE_INFO << LC << "  uniform buffer objects = " << SAYBOOL(_supportsUniformBufferObjects) << std::endl;

    OE_INFO << LC << "  max uniform block size = " << _maxUniformBlockSize << std::endl;

    OE_INFO << LC << "  depth-packed stencil = " << SAYBOOL(_supportsDepthPackedStencilBuffer) << std::endl;

    OE_INFO << LC << "  Supports quad buffer stereo = " << SAYBOOL(_supportsQuadBufferStereo) << std::endl;

    OE_INFO << LC << "  occulsion query = " << SAYBOOL(_supportsOcclusionQuery) << std::endl;

    //_supportsTexture2DLod = osg::isGLExtensionSupported( id, "GL_ARB_shader_texture_lod" );
    //OE_INFO << LC << "  texture2DLod = " << SAYBOOL(_supportsTexture2DLod) << std::endl;
    OE_INFO << LC << "  Mipmapped texture updates = " << SAYBOOL(_supportsMipmappedTextureUpdates) << std::endl;

    OE_INFO << LC << "  Max Fast Texture Size = " << _maxFastTextureSize << std::endl;
}

static const std::string & osgearth_capabilities_cache_file()
{
    static std::string s_capabilitiesCacheFile;
    if(s_capabilitiesCacheFile.empty())
    {
#ifdef _WIN32
        char tmp[512];
        if(GetTempPathA(sizeof(tmp)/sizeof(tmp[0]), tmp))
        {
            strcat(tmp, "\\_osgearth_capabilities_cache");
            s_capabilitiesCacheFile = tmp;
        }
#else
        s_capabilitiesCacheFile = "/tmp/_osgearth_capabilities_cache";
#endif
    }
    return s_capabilitiesCacheFile;
}

bool Capabilities::readCache()
{
    bool ret;
    const std::string & cache_file = osgearth_capabilities_cache_file();
    XmlDocument * doc = XmlDocument::load(cache_file);
    if(doc)
    {
        OE_INFO << LC << "Read cached capabilities from " << cache_file << std::endl;

        Config docConf = doc->getConfig();
        Config conf;
        if ( docConf.hasChild( "capabilities" ) )
            conf = docConf.child( "capabilities" );

        conf.getIfSet("maxffptextureunits", _maxFFPTextureUnits );
        conf.getIfSet("maxgputextureunits", _maxGPUTextureUnits );
        conf.getIfSet("maxgputexturecoordsets", _maxGPUTextureCoordSets );
        conf.getIfSet("maxtexturesize", _maxTextureSize );
        conf.getIfSet("maxfasttexturesize", _maxFastTextureSize );
        conf.getIfSet("maxlights", _maxLights );
        conf.getIfSet("depthbits", _depthBits );
        conf.getIfSet("supportsglsl", _supportsGLSL );
        conf.getIfSet("glslversion", _GLSLversion );
        conf.getIfSet("supportstexturearrays", _supportsTextureArrays );
        conf.getIfSet("supportstexture3d", _supportsTexture3D );
        conf.getIfSet("supportsmultitexture", _supportsMultiTexture );
        conf.getIfSet("supportsstencilwrap", _supportsStencilWrap );
        conf.getIfSet("supportstwosidedstencil", _supportsTwoSidedStencil );
        conf.getIfSet("supportstexture2dlod", _supportsTexture2DLod );
        conf.getIfSet("supportsmipmappedtextureupdates", _supportsMipmappedTextureUpdates );
        conf.getIfSet("supportsdepthpackedstencilbuffer", _supportsDepthPackedStencilBuffer );
        conf.getIfSet("supportsquadbufferstereo", _supportsQuadBufferStereo );
        conf.getIfSet("supportsocclusionquery", _supportsOcclusionQuery );
        conf.getIfSet("supportsdrawinstanced", _supportsDrawInstanced );
        conf.getIfSet("supportsuniformbufferobjects", _supportsUniformBufferObjects );
        conf.getIfSet("vendor", _vendor );
        conf.getIfSet("renderer", _renderer );
        conf.getIfSet("version", _version );

        ret = true;
    }
    else
    {
        OE_INFO << LC << "Capabilities cache " << cache_file << " is invalid" << std::endl;
        ret = false;
    }
    return ret;
}

bool Capabilities::writeCache()
{
    bool ret;
    Config conf("capabilities");
    conf.update("maxffptextureunits", _maxFFPTextureUnits );
    conf.update("maxgputextureunits", _maxGPUTextureUnits );
    conf.update("maxgputexturecoordsets", _maxGPUTextureCoordSets );
    conf.update("maxtexturesize", _maxTextureSize );
    conf.update("maxfasttexturesize", _maxFastTextureSize );
    conf.update("maxlights", _maxLights );
    conf.update("depthbits", _depthBits );
    conf.update("supportsglsl", _supportsGLSL );
    conf.update("glslversion", _GLSLversion );
    conf.update("supportstexturearrays", _supportsTextureArrays );
    conf.update("supportstexture3d", _supportsTexture3D );
    conf.update("supportsmultitexture", _supportsMultiTexture );
    conf.update("supportsstencilwrap", _supportsStencilWrap );
    conf.update("supportstwosidedstencil", _supportsTwoSidedStencil );
    conf.update("supportstexture2dlod", _supportsTexture2DLod );
    conf.update("supportsmipmappedtextureupdates", _supportsMipmappedTextureUpdates );
    conf.update("supportsdepthpackedstencilbuffer", _supportsDepthPackedStencilBuffer );
    conf.update("supportsquadbufferstereo", _supportsQuadBufferStereo );
    conf.update("supportsocclusionquery", _supportsOcclusionQuery );
    conf.update("supportsdrawinstanced", _supportsDrawInstanced );
    conf.update("supportsuniformbufferobjects", _supportsUniformBufferObjects );
    conf.update("vendor", _vendor );
    conf.update("renderer", _renderer );
    conf.update("version", _version );

    const std::string & cache_file = osgearth_capabilities_cache_file();
    std::ofstream out( cache_file.c_str());
    ret = out.is_open();
    if ( ret )
    {
        OE_INFO << LC << "Write detected capabilities to " << cache_file << std::endl;

        // dump that Config out as XML.
        osg::ref_ptr<XmlDocument> xml = new XmlDocument( conf );
        xml->store( out );
    }
    return ret;
}


