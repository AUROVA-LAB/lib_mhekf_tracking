#edit the following line to add the librarie's header files
FIND_PATH(mhekf_tracking_INCLUDE_DIRS 
          mhekf_tracking.h
          observation.h 
          track.h
          tracking_target.h
          /usr/include/iridrivers 
          /usr/local/include/iridrivers)

FIND_LIBRARY(mhekf_tracking_LIBRARY
    NAMES mhekf_tracking
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iridrivers) 

IF (mhekf_tracking_INCLUDE_DIRS AND mhekf_tracking_LIBRARY)
   SET(mhekf_tracking_FOUND TRUE)
ENDIF (mhekf_tracking_INCLUDE_DIRS AND mhekf_tracking_LIBRARY)

IF (mhekf_tracking_FOUND)
   IF (NOT mhekf_tracking_FIND_QUIETLY)
      MESSAGE(STATUS "Found mhekf_tracking: ${mhekf_tracking_LIBRARY}")
   ENDIF (NOT mhekf_tracking_FIND_QUIETLY)
ELSE (mhekf_tracking_FOUND)
   IF (mhekf_tracking_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find mhekf_tracking")
   ENDIF (mhekf_tracking_FIND_REQUIRED)
ENDIF (mhekf_tracking_FOUND)

