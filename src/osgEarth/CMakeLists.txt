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

SET(HEADER_PATH ${OpenSceneGraph_SOURCE_DIR}/include/${LIB_NAME})
SET(TARGET_PUBLIC_H
    ${HEADER_PATH}/Caching
	${HEADER_PATH}/CacheSeed
	${HEADER_PATH}/Capabilities
	${HEADER_PATH}/Common
	${HEADER_PATH}/Config
	${HEADER_PATH}/Cube
	${HEADER_PATH}/ElevationLayer
	${HEADER_PATH}/Export
	${HEADER_PATH}/EGM
	${HEADER_PATH}/FileUtils
	${HEADER_PATH}/FindNode
	${HEADER_PATH}/GeoData
	${HEADER_PATH}/GeoMath
	${HEADER_PATH}/HeightFieldUtils
	${HEADER_PATH}/HTTPClient
	${HEADER_PATH}/ImageToHeightFieldConverter
	${HEADER_PATH}/ImageLayer
	${HEADER_PATH}/ImageMosaic
	${HEADER_PATH}/ImageUtils
	${HEADER_PATH}/JsonUtils
	${HEADER_PATH}/Layer
	${HEADER_PATH}/Locators
	${HEADER_PATH}/Map
	${HEADER_PATH}/MapNode
	${HEADER_PATH}/MapNodeOptions
	${HEADER_PATH}/MapOptions
	${HEADER_PATH}/MaskLayer
	${HEADER_PATH}/MaskNode
	${HEADER_PATH}/ModelLayer
	${HEADER_PATH}/ModelSource
	${HEADER_PATH}/Notify
	${HEADER_PATH}/Profile
	${HEADER_PATH}/Progress
	${HEADER_PATH}/Registry
	${HEADER_PATH}/Revisioning
    ${HEADER_PATH}/ShaderComposition
    ${HEADER_PATH}/ShaderUtils
	${HEADER_PATH}/SparseTexture2DArray
	${HEADER_PATH}/SpatialReference
	${HEADER_PATH}/StringUtils
	${HEADER_PATH}/TaskService
    ${HEADER_PATH}/TerrainLayer
    ${HEADER_PATH}/TerrainOptions
	${HEADER_PATH}/TerrainEngineNode
    ${HEADER_PATH}/TextureCompositor
    ${HEADER_PATH}/TextureCompositorMulti
    ${HEADER_PATH}/TextureCompositorTexArray
    ${HEADER_PATH}/TileFactory
	${HEADER_PATH}/TileKey
	${HEADER_PATH}/TileSource
	${HEADER_PATH}/ThreadingUtils
	${HEADER_PATH}/TMS
	${HEADER_PATH}/Units
	${HEADER_PATH}/Version
	${HEADER_PATH}/VerticalSpatialReference
	${HEADER_PATH}/XmlUtils
)

SET(TARGET_H
    tinystr.h
    tinyxml.h 
	)

SET(TARGET_SRC
    Caching.cpp
    CacheSeed.cpp
	Capabilities.cpp
    Config.cpp
    Cube.cpp
    EGM.cpp
    ElevationLayer.cpp
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
    Map.cpp
    MapNode.cpp
    MapNodeOptions.cpp
    MapOptions.cpp
    MaskLayer.cpp
    MaskNode.cpp
	MimeTypes.cpp
	ModelLayer.cpp
	ModelSource.cpp
	Notify.cpp
    Profile.cpp
	Progress.cpp
    Registry.cpp
    ShaderComposition.cpp
    ShaderUtils.cpp
    SparseTexture2DArray.cpp
    SpatialReference.cpp
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
	OpenThreads
    )

SETUP_OSG_LIB(osgEarth)
