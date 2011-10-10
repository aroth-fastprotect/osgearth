#---------------------------------------------------
# OSG CMAKE SUPPORT 
# (C) by Michael Wagner, mtw@shared-reality.com 2005
# (C) Eric Wing, Luigi Calori and Robert Osfield 2006-2007
#---------------------------------------------------

PROJECT(OSGEARTH_PLUGINS_MASTER)

IF(MSVC80 OR MSVC90)
    IF(NOT OSG_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS)
        SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /MANIFEST:NO")
    ENDIF()
ENDIF()

#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES 
    osgEarth
)
#SET(TARGET_COMMON_LIBRARIES osg osgDB osgUtil osgEarth)

############################################################
#
#  NodeKit/Psudo loader plugins
#
ADD_SUBDIRECTORY(earth)

ADD_SUBDIRECTORY(wcs)
ADD_SUBDIRECTORY(wms)
ADD_SUBDIRECTORY(tilecache)
ADD_SUBDIRECTORY(tileservice)
ADD_SUBDIRECTORY(yahoo)
ADD_SUBDIRECTORY(arcgis_map_cache)
ADD_SUBDIRECTORY(arcgis)
ADD_SUBDIRECTORY(tms)
ADD_SUBDIRECTORY(vpb)
ADD_SUBDIRECTORY(osg)
ADD_SUBDIRECTORY(agglite)
ADD_SUBDIRECTORY(model_simple)
ADD_SUBDIRECTORY(model_lodovr)
ADD_SUBDIRECTORY(debug)

IF(LIBZIP_FOUND)
	ADD_SUBDIRECTORY(zipfs)
	ADD_SUBDIRECTORY(worldwind)
ENDIF(LIBZIP_FOUND)

IF(GDAL_FOUND)
	ADD_SUBDIRECTORY(gdal)
	ADD_SUBDIRECTORY(feature_ogr)
  	ADD_SUBDIRECTORY(feature_wfs)  
#  	ADD_SUBDIRECTORY(model_feature_overlay)
	ADD_SUBDIRECTORY(model_feature_stencil)
	ADD_SUBDIRECTORY(model_feature_geom)
	ADD_SUBDIRECTORY(model_feature_label)
  	ADD_SUBDIRECTORY(mask_feature)
  	ADD_SUBDIRECTORY(label_overlay)
ENDIF(GDAL_FOUND)

IF(SQLITE3_FOUND)
  ADD_SUBDIRECTORY(cache_sqlite3)
ENDIF(SQLITE3_FOUND)

ADD_SUBDIRECTORY(engine_osgterrain)

# not yet ready drivers: droam and seamless
#ADD_SUBDIRECTORY(engine_droam)
#ADD_SUBDIRECTORY(engine_seamless)

