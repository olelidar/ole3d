# build the driver node
add_executable(ole_node ole_node.cc driver.cc)
add_dependencies(ole_node ole_driver_gencfg)
target_link_libraries(ole_node
  ole_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(driver_nodelet nodelet.cc driver.cc)
add_dependencies(driver_nodelet ole_driver_gencfg)
target_link_libraries(driver_nodelet
  ole_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# install runtime files
install(TARGETS ole_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
