# driver source files
SET(sources mhekf_tracking.cpp observation.cpp track.cpp tracking_target.cpp)
# application header files
SET(headers ../include/mhekf_tracking.h ../include/observation.h ../include/track.h ../include/tracking_target.h ../include/struct_definitions.h)
# locate the necessary dependencies
FIND_PACKAGE(PCL REQUIRED COMPONENTS common)

# add the necessary include directories
INCLUDE_DIRECTORIES(../include)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

# create the shared library
ADD_LIBRARY(mhekf_tracking SHARED ${sources})
# link necessary libraries
TARGET_LINK_LIBRARIES(mhekf_tracking ${PCL_COMMON_LIBRARIES})

INSTALL(TARGETS mhekf_tracking
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/iridrivers
        ARCHIVE DESTINATION lib/iridrivers)
INSTALL(FILES ${headers} DESTINATION include/iridrivers)
INSTALL(FILES ../Findmhekf_tracking.cmake DESTINATION ${CMAKE_ROOT}/Modules/)
ADD_SUBDIRECTORY(examples)
