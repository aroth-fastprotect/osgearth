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
    Cache
    CacheBin
    CachePolicy
	CacheSeed
	Capabilities
	Common
	CompositeTileSource
	Config
    Containers
	Cube
    CullingUtils
    DrapeableNode
    ECEF
	ElevationLayer
    ElevationQuery
	Export
	EGM
	FileUtils
	FindNode
	GeoData
	GeoMath
	HeightFieldUtils
	HTTPClient
	ImageToHeightFieldConverter
	ImageLayer
	ImageMosaic
	ImageUtils
    IOTypes
	JsonUtils
	Layer
    LineFunctor
	Locators
    LocalTangentPlane
	Map
	MapNode
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
	OverlayDecorator
	Profile
	Progress
	Random
	Registry
	Revisioning
    ShaderComposition
    ShaderUtils
	SparseTexture2DArray
	SpatialReference
	StringUtils
	TaskService
    Terrain
    TerrainLayer
    TerrainOptions
	TerrainEngineNode
    TextureCompositor
    TextureCompositorMulti
    TextureCompositorTexArray
    TileFactory
	TileKey
	TileSource
	ThreadingUtils
	Units
    URI
    Utils
	Version
	VerticalSpatialReference
    Viewpoint
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
    Cache.cpp
    CachePolicy.cpp
    CacheSeed.cpp
	Capabilities.cpp
    CompositeTileSource.cpp
    Config.cpp
    Cube.cpp
    CullingUtils.cpp
    DrapeableNode.cpp
    ECEF.cpp
    EGM.cpp
    ElevationLayer.cpp
    ElevationQuery.cpp
    FileUtils.cpp
    GeoData.cpp
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
    Profile.cpp
	Progress.cpp
	Random.cpp
    Registry.cpp
    ShaderComposition.cpp
    ShaderUtils.cpp
    SparseTexture2DArray.cpp
    SpatialReference.cpp
    StringUtils.cpp
    TaskService.cpp
    Terrain.cpp
    TerrainLayer.cpp
    TerrainOptions.cpp
    TerrainEngineNode.cpp
    TextureCompositor.cpp
    TextureCompositorMulti.cpp
    TextureCompositorTexArray.cpp
    TileFactory.cpp
    TileKey.cpp
    TileSource.cpp
    Units.cpp
    URI.cpp
    Utils.cpp
    Version.cpp
    VerticalSpatialReference.cpp
    Viewpoint.cpp
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
	OpenThreads
    )

SETUP_OSG_LIB(osgEarth)
