add_library(ole_rawdata rawdata.cc calibration.cc)
target_link_libraries(ole_rawdata 
                      ${catkin_LIBRARIES}
                      ${YAML_CPP_LIBRARIES})
install(TARGETS ole_rawdata
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
