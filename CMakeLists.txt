cmake_minimum_required(VERSION 3.12)

project("tiny_wbc")

set(dart_required_components utils-urdf gui-osg)
set(dart_required_libraries dart dart-utils-urdf dart-gui-osg)

find_package(DART 6.10.0 REQUIRED COMPONENTS ${dart_required_components} CONFIG)

add_subdirectory(examples)
