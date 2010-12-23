
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthUtil)

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

SET(INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/engine_osgterrain)
SET(TARGET_PUBLIC_H 
    ${INCDIR}/Common
    ${INCDIR}/CustomTerrain
	${INCDIR}/CustomTile
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

SETUP_PLUGIN(osgearth_engine_osgterrain)
