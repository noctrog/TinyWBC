set(dart_required_components utils-urdf gui-osg)
set(dart_required_libraries dart dart-utils-urdf dart-gui-osg)

include_directories(../src/include)

find_package(DART 6.10.0 REQUIRED COMPONENTS ${dart_required_components} CONFIG)

add_subdirectory(hyq)
