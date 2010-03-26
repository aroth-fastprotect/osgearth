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
	${HEADER_PATH}/Common
	${HEADER_PATH}/Compositing
	${HEADER_PATH}/Config
	${HEADER_PATH}/Cube
	${HEADER_PATH}/DriverOptions
	${HEADER_PATH}/EarthFile
	${HEADER_PATH}/EarthTerrainTechnique
	${HEADER_PATH}/EarthTerrain
	${HEADER_PATH}/Export
	${HEADER_PATH}/FileUtils
	${HEADER_PATH}/FileLocationCallback
	${HEADER_PATH}/FindNode
	${HEADER_PATH}/GeoData
	${HEADER_PATH}/HeightFieldUtils
	${HEADER_PATH}/HTTPClient
	${HEADER_PATH}/ImageToHeightFieldConverter
	${HEADER_PATH}/ImageUtils
	${HEADER_PATH}/JsonUtils
	${HEADER_PATH}/Locators
	${HEADER_PATH}/Map
	${HEADER_PATH}/MapLayer
	${HEADER_PATH}/MapNode
	${HEADER_PATH}/MapEngine
	${HEADER_PATH}/MapEngineProperties
	${HEADER_PATH}/MaskNode
	${HEADER_PATH}/ModelLayer
	${HEADER_PATH}/ModelSource
	${HEADER_PATH}/MultiPassTerrainTechnique
	${HEADER_PATH}/Profile
	${HEADER_PATH}/Progress
	${HEADER_PATH}/Registry
	${HEADER_PATH}/SpatialReference
	${HEADER_PATH}/StringUtils
	${HEADER_PATH}/TaskService
	${HEADER_PATH}/TerrainTileEdgeNormalizerUpdateCallback
	${HEADER_PATH}/TileKey
	${HEADER_PATH}/TileSource
	${HEADER_PATH}/TileSourceFactory
	${HEADER_PATH}/TMS
	${HEADER_PATH}/VersionedTerrain
	${HEADER_PATH}/XmlUtils
)

SET(TARGET_SRC
    Caching.cpp
    CacheSeed.cpp
    Compositing.cpp
    Config.cpp
    Cube.cpp
    DriverOptions.cpp
    EarthFile.cpp
    EarthTerrainTechnique.cpp
    FileUtils.cpp
    GeoData.cpp
    HeightFieldUtils.cpp
    HTTPClient.cpp
    ImageToHeightFieldConverter.cpp
    ImageUtils.cpp
    JsonUtils.cpp
    Locators.cpp
    Map.cpp
    MapLayer.cpp
    MapNode.cpp
    MapEngine.cpp
    MapEngineProperties.cpp
    MaskNode.cpp
	MimeTypes.cpp
	ModelLayer.cpp
	ModelSource.cpp
	MultiPassTerrainTechnique.cpp
	Notify.cpp
    Profile.cpp
	Progress.cpp
    Registry.cpp
    SpatialReference.cpp
    TaskService.cpp
    TerrainTileEdgeNormalizerUpdateCallback.cpp
    TileKey.cpp
    TileSource.cpp
    TileSourceFactory.cpp
    TMS.cpp
    Version.cpp
    VersionedTerrain.cpp
    XmlUtils.cpp
)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} ${CURL_INCLUDE_DIR} ${EXPAT_INCLUDE_DIR} ${GDAL_INCLUDE_DIR})

SET(TARGET_LIBRARIES_VARS CURL_LIBRARY GDAL_LIBRARY EXPAT_LIBRARY)

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
