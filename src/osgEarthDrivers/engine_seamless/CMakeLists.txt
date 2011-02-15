SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES})

SET(TARGET_SRC
        QSC.cpp
        Euler.cpp
        Patch.cpp
        PatchGroup.cpp
        PatchSet.cpp
        Projected.cpp
        Geographic.cpp
        GeoPatch.cpp
        SeamlessEngineNode.cpp
        SeamlessPlugin.cpp
)

SET(INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../include/osgEarthDrivers/engine_seamless)
SET(TARGET_PUBLIC_H 
        ${INCDIR}/QSC
        ${INCDIR}/Euler
        ${INCDIR}/Patch
        ${INCDIR}/PatchInfo
        ${INCDIR}/PatchGroup
        ${INCDIR}/PatchSet
        ${INCDIR}/Projected
        ${INCDIR}/Geographic
        ${INCDIR}/GeoPatch
        ${INCDIR}/SeamlessEngineNode
        ${INCDIR}/SeamlessOptions
)

INCLUDE_DIRECTORIES(${INCDIR} ${GDAL_INCLUDE_DIR})

# to install public driver includes:
SETUP_PLUGIN(osgearth_engine_seamless)
