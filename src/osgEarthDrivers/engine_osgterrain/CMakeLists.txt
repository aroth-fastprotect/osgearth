
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

SET(TARGET_PUBLIC_H 
    Common
    CustomTerrain
	CustomTile
    CustomTerrainTechnique
    FileLocationCallback
    MultiPassTerrainTechnique
    OSGTerrainEngineNode
    OSGTerrainOptions
    OSGTileFactory
    TerrainTileEdgeNormalizerUpdateCallback
    TransparentLayer
    SinglePassTerrainTechnique
    )

SETUP_PLUGIN(osgearth_engine_osgterrain)
