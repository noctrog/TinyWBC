set(EXAMPLE_RESOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hyq_urdf)
set(EXAMPLE_RESOURCE_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/hyq_urdf)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/configure.h.cmake
  ${CMAKE_CURRENT_SOURCE_DIR}/include/configure.h)

include_directories(include)

file(GLOB srcs CONFIGURE_DEPENDS *.cc)
add_executable(hyq_demo ${srcs})
target_link_libraries(hyq_demo PUBLIC ${dart_required_libraries} ${PROJECT_NAME})

install(FILES
hyq_urdf/meshes/trunk/trunk.dae
hyq_urdf/meshes/leg/upperleg.dae
hyq_urdf/meshes/leg/hipassembly.dae
hyq_urdf/meshes/leg/lowerleg.dae
hyq_urdf/urdf/hyq.urdf
DESTINATION ${EXAMPLE_RESOURCE_INSTALL_DIR}/hyq_urdf)

