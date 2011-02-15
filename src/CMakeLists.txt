#the old construct SUBDIRS( was substituded by ADD_SUBDIRECTORY that is to be preferred according on CMake docs.
FOREACH( myfolder
        osgEarth
        osgEarthFeatures
        osgEarthUtil
        osgEarthDrivers
        osgEarthSymbology
        applications
    )

    ADD_SUBDIRECTORY(${myfolder})

ENDFOREACH( myfolder)
