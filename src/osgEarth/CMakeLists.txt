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

SET(LIB_NAME osgEarth)

SET(TARGET_PUBLIC_H
    Caching
	CacheSeed
	Capabilities
	Common
	CompositeTileSource
	Config
	Cube
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
	JsonUtils
	Layer
	Locators
    LocalTangentPlane
	Map
	MapNode
	MapNodeOptions
	MapOptions
	MaskLayer
	MaskNode
    MaskSource
    MeshConsolidator
	ModelLayer
	ModelSource
    NodeUtils
	Notify
	OverlayDecorator
	Profile
	Progress
	Registry
	Revisioning
    ShaderComposition
    ShaderUtils
	SparseTexture2DArray
	SpatialReference
	StringUtils
	TaskService
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
	#tinystr.h # only to simplify merging
	#tinyxml.h # only to simplify merging 
	TMS
	Units
    URI
    Utils
	Version
	VerticalSpatialReference
	XmlUtils
)

SET(TARGET_H
    tinystr.h
    tinyxml.h 
	)

SET(TARGET_SRC
    Caching.cpp
    CacheSeed.cpp
	Capabilities.cpp
    CompositeTileSource.cpp
    Config.cpp
    Cube.cpp
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
    MeshConsolidator.cpp
	MimeTypes.cpp
	ModelLayer.cpp
	ModelSource.cpp
	NodeUtils.cpp
	Notify.cpp
	OverlayDecorator.cpp
    Profile.cpp
	Progress.cpp
    Registry.cpp
    ShaderComposition.cpp
    ShaderUtils.cpp
    SparseTexture2DArray.cpp
    SpatialReference.cpp
    StringUtils.cpp
    TaskService.cpp
    TerrainLayer.cpp
    TerrainOptions.cpp
    TerrainEngineNode.cpp
    TextureCompositor.cpp
    TextureCompositorMulti.cpp
    TextureCompositorTexArray.cpp
    TileFactory.cpp
    TileKey.cpp
    TileSource.cpp
    tinystr.cpp
    tinyxml.cpp
    tinyxmlerror.cpp
    tinyxmlparser.cpp
    TMS.cpp
    Units.cpp
    URI.cpp
    Utils.cpp
    Version.cpp
    VerticalSpatialReference.cpp
    XmlUtils.cpp
)

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR} ${CURL_INCLUDE_DIR} ${OSG_INCLUDE_DIR} )

SET(TARGET_LIBRARIES_VARS CURL_LIBRARY GDAL_LIBRARY)

SET(TARGET_ADDED_LIBRARIES
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
