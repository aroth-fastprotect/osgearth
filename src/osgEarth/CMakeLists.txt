IF   (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY)
ELSE (DYNAMIC_OSGEARTH)
    ADD_DEFINITIONS(-DOSGEARTH_LIBRARY_STATIC)
ENDIF(DYNAMIC_OSGEARTH)

OPTION(CURL_IS_STATIC "on if curl is a static lib " ON)
MARK_AS_ADVANCED(CURL_IS_STATIC)

#IF(WIN32)
#   SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
#    IF(CURL_IS_STATIC)
#        ADD_DEFINITIONS(-DCURL_STATICLIB)
#        SET(TARGET_EXTERNAL_LIBRARIES ws2_32 winmm wldap32)
#    ENDIF(CURL_IS_STATIC)
#ENDIF(WIN32)

SET(TARGET_PUBLIC_H
	Bounds
    Cache
    CacheBin
    CachePolicy
	CacheSeed
	Capabilities
    ClampableNode
    ClampingTechnique
    ColorFilter
	Common
	CompositeTileSource
	Config
    Containers
	Cube
    CullingUtils
    DepthOffset
    DPLineSegmentIntersector
    Draggers
    DrapeableNode
    DrapingTechnique
    DrawInstanced
    ECEF
	ElevationLayer
    ElevationLOD
    ElevationQuery
	Export
    FadeEffect
	FileUtils
	GeoCommon
	GeoData
    Geoid
	GeoMath
	HeightFieldUtils
	HTTPClient
	ImageLayer
	ImageMosaic
    ImageToHeightFieldConverter
	ImageUtils
    IOTypes
	JsonUtils
	Layer
    LineFunctor
	Locators
    LocalTangentPlane
	Map
    MapCallback
    MapFrame
    MapInfo
    MapModelChange
	MapNode
    MapNodeObserver
	MapNodeOptions
	MapOptions
	MaskLayer
	MaskNode
    MaskSource
    MemCache
	ModelLayer
	ModelSource
    NodeUtils
	Notify
	optional
	OverlayDecorator
    OverlayNode
    Pickers
	Profile
	Progress
	Random
	Registry
	Revisioning
    ShaderFactory
    ShaderGenerator
    ShaderUtils
	SparseTexture2DArray
	SpatialReference
    StateSetCache
	StringUtils
	TaskService
    Terrain
    TerrainLayer
    TerrainOptions
	TerrainEngineNode
    TextureCompositor
    TextureCompositorMulti
    TextureCompositorTexArray
	TileKey
	TileSource
	ThreadingUtils
	Units
    URI
    Utils
	Version
	VerticalDatum
    Viewpoint
    VirtualProgram
	XmlUtils
)

SET(TARGET_H
    tinystr.h
    tinyxml.h 
	)
	
LIST(APPEND TARGET_SRC
    tinystr.cpp
    tinyxml.cpp
    tinyxmlerror.cpp
    tinyxmlparser.cpp
	)

LIST(APPEND TARGET_SRC
	Bounds.cpp
    Cache.cpp
    CachePolicy.cpp
    CacheSeed.cpp
	Capabilities.cpp
    ClampableNode.cpp
    ClampingTechnique.cpp
    ColorFilter.cpp
    CompositeTileSource.cpp
    Config.cpp
    Cube.cpp
    CullingUtils.cpp
    DepthOffset.cpp
	Draggers.cpp
    DPLineSegmentIntersector.cpp
    DrapeableNode.cpp
    DrapingTechnique.cpp
    DrawInstanced.cpp
    ECEF.cpp
    ElevationLayer.cpp
    ElevationLOD.cpp
    ElevationQuery.cpp
    FadeEffect.cpp
    FileUtils.cpp
    GeoData.cpp
    Geoid.cpp
    GeoMath.cpp
    HeightFieldUtils.cpp
    HTTPClient.cpp
    ImageLayer.cpp
    ImageMosaic.cpp
    ImageToHeightFieldConverter.cpp
    ImageUtils.cpp
    IOTypes.cpp
    JsonUtils.cpp
    Layer.cpp
    Locators.cpp
    LocalTangentPlane.cpp
    Map.cpp
    MapCallback.cpp
    MapFrame.cpp
    MapInfo.cpp
    MapNode.cpp
    MapNodeOptions.cpp
    MapOptions.cpp
    MaskLayer.cpp
    MaskNode.cpp
    MaskSource.cpp
    MemCache.cpp
	MimeTypes.cpp
	ModelLayer.cpp
	ModelSource.cpp
	NodeUtils.cpp
	Notify.cpp
	OverlayDecorator.cpp
    OverlayNode.cpp
	Pickers.cpp
    Profile.cpp
	Progress.cpp
	Random.cpp
    Registry.cpp
    Revisioning.cpp
    ShaderFactory.cpp
    ShaderGenerator.cpp
    ShaderUtils.cpp
    SparseTexture2DArray.cpp
    SpatialReference.cpp
    StateSetCache.cpp
    StringUtils.cpp
    TaskService.cpp
    Terrain.cpp
    TerrainLayer.cpp
    TerrainOptions.cpp
    TerrainEngineNode.cpp
    TextureCompositor.cpp
    TextureCompositorMulti.cpp
    TextureCompositorTexArray.cpp
    TileKey.cpp
    TileSource.cpp
    ThreadingUtils.cpp
    Units.cpp
    URI.cpp
    Utils.cpp
    Version.cpp
	VerticalDatum.cpp
    Viewpoint.cpp
    VirtualProgram.cpp
    XmlUtils.cpp
)

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR} ${CURL_INCLUDE_DIR} ${OSG_INCLUDE_DIR} )

IF (TINYXML_FOUND)
    INCLUDE_DIRECTORIES(${TINYXML_INCLUDE_DIR})
ENDIF (TINYXML_FOUND)

LIST(APPEND TARGET_LIBRARIES_VARS CURL_LIBRARY GDAL_LIBRARY)

LIST(APPEND TARGET_ADDED_LIBRARIES
	osg
	osgUtil
	osgSim
	osgTerrain
	osgDB
	osgFX
	osgViewer
	osgText
	osgGA
	osgShadow
	osgManipulator
	OpenThreads
    )

SETUP_OSG_LIB(osgEarth)
