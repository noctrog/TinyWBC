include_directories(include)

file(GLOB srcs CONFIGURE_DEPENDS *.cc)
add_executable(hyq_demo ${srcs})
target_link_libraries(hyq_demo PUBLIC ${dart_required_libraries})
