
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)

SET(INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/engine_osgterrain)
SET(TARGET_PUBLIC_H 
    
    ${INCDIR}/Common
    ${INCDIR}/CustomTerrain
    ${INCDIR}/CustomTerrainTechnique
    ${INCDIR}/FileLocationCallback
    ${INCDIR}/MultiPassTerrainTechnique
    ${INCDIR}/OSGTerrainEngineNode
    ${INCDIR}/OSGTerrainOptions
    ${INCDIR}/OSGTileFactory
    ${INCDIR}/TerrainTileEdgeNormalizerUpdateCallback
    ${INCDIR}/TransparentLayer
    ${INCDIR}/SinglePassTerrainTechnique
    )
INCLUDE_DIRECTORIES(${INCDIR})

MESSAGE(STATUS "common=${INCDIR}/Common")

SET(TARGET_SRC
    CustomTerrain.cpp
    CustomTile.cpp
    MultiPassTerrainTechnique.cpp
    OSGTerrainEngineNode.cpp    
    OSGTileFactory.cpp
    Plugin.cpp
    TerrainTileEdgeNormalizerUpdateCallback.cpp
    SinglePassTerrainTechnique.cpp
)

SETUP_PLUGIN(osgearth_engine_osgterrain)
