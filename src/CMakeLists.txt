find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${OSQPEIGEN_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

file(GLOB srcs CONFIGURE_DEPENDS *.cc)
add_library(${PROJECT_NAME} ${srcs})
target_link_libraries(${PROJECT_NAME} PUBLIC OsqpEigen::OsqpEigen ${pinocchio_LIBRARIES})
