add_executable(cloud_node cloud_node.cc convert.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cloud_node ole_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS cloud_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(cloud_nodelet cloud_nodelet.cc convert.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(cloud_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cloud_nodelet ole_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS cloud_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(transform_node transform_node.cc transform.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_node ole_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS transform_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(transform_nodelet transform_nodelet.cc transform.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(transform_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_nodelet ole_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS transform_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
