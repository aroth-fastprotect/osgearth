#---------------------------------------------------
# OSG CMAKE SUPPORT 
# (C) by Michael Wagner, mtw@shared-reality.com 2005
# (C) Eric Wing, Luigi Calori and Robert Osfield 2006-2007
#---------------------------------------------------

PROJECT(OSGEARTH_PLUGINS_MASTER)

SET(TARGET_COMMON_LIBRARIES osg osgDB osgUtil osgEarth)
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

IF(LIBZIP_FOUND)
	ADD_SUBDIRECTORY(zipfs)
	ADD_SUBDIRECTORY(worldwind)
ENDIF(LIBZIP_FOUND)

IF(GDAL_FOUND)
	ADD_SUBDIRECTORY(gdal)
	ADD_SUBDIRECTORY(feature_ogr)
	ADD_SUBDIRECTORY(model_feature_overlay)
	ADD_SUBDIRECTORY(model_feature_stencil)
	ADD_SUBDIRECTORY(model_feature_geom)
	ADD_SUBDIRECTORY(model_feature_label)
ENDIF(GDAL_FOUND)

IF(SQLITE3_FOUND)
  ADD_SUBDIRECTORY(cache_sqlite3)
ENDIF(SQLITE3_FOUND)

ADD_SUBDIRECTORY(engine_osgterrain)
#ADD_SUBDIRECTORY(engine_droam)
    
