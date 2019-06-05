# create an example application
ADD_EXECUTABLE(mhekf_tracking_test mhekf_tracking_test.cpp)
# link necessary libraries
TARGET_LINK_LIBRARIES(mhekf_tracking_test mhekf_tracking)
