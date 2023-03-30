add_library(ole_input input.cc)
target_link_libraries(ole_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(ole_input ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS ole_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
