
LIST(APPEND TARGET_ADDED_LIBRARIES osgEarthSymbology osgTerrain osgGA)

SET(TARGET_SRC
    KeyNodeFactory.cpp
    LODFactorCallback.cpp
    MultiPassTerrainTechnique.cpp
    OSGTerrainEngineNode.cpp    
    OSGTileFactory.cpp
    ParallelKeyNodeFactory.cpp
    Plugin.cpp
    SerialKeyNodeFactory.cpp
    SinglePassTerrainTechnique.cpp
    StreamingTerrainNode.cpp
    StreamingTile.cpp
    TerrainNode.cpp
    Tile.cpp
    TileBuilder.cpp
)

SET(TARGET_PUBLIC_H 
    Common
    CustomTerrainTechnique
    DynamicLODScaleCallback   
    FileLocationCallback
    KeyNodeFactory
    LODFactorCallback
    MultiPassTerrainTechnique
    ParallelKeyNodeFactory
    OSGTerrainEngineNode
    OSGTerrainOptions
    OSGTileFactory
    SerialKeyNodeFactory
    SinglePassTerrainTechnique
    StreamingTerrainNode
    StreamingTile
    TerrainNode
    Tile
    TileBuilder
    TransparentLayer
    )
SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/engine_osgterrain)
SETUP_PLUGIN(osgearth_engine_osgterrain)
