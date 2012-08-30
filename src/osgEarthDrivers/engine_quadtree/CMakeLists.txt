
LIST(APPEND TARGET_ADDED_LIBRARIES osgEarthSymbology osgTerrain osgGA)

SET(TARGET_SRC
    CustomPagedLOD.cpp
    KeyNodeFactory.cpp
    LODFactorCallback.cpp
    QuadTreeTerrainEngineNode.cpp
    QuadTreeTerrainEngineDriver.cpp
    SerialKeyNodeFactory.cpp
    TerrainNode.cpp
    TileModelCompiler.cpp
    TileNode.cpp
    TileNodeRegistry.cpp
    TileModelFactory.cpp
)

SET(TARGET_PUBLIC_H
    Common
    CustomPagedLOD
    DynamicLODScaleCallback
    FileLocationCallback
    KeyNodeFactory
    LODFactorCallback
    QuadTreeTerrainEngineNode
    QuadTreeTerrainEngineOptions
    QuickReleaseGLObjects
    SerialKeyNodeFactory
    TerrainNode
    TileModel
    TileModelCompiler
    TileNode
    TileNodeRegistry
    TileModelFactory
)

SET(TARGET_INSTALL_INCLUDE_DIR osgEarthDrivers/engine_quadtree)
SETUP_PLUGIN(osgearth_engine_quadtree)
