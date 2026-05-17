cmake_minimum_required(VERSION 3.5)

project(db-CBS)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(YAML REQUIRED yaml-cpp)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)
add_subdirectory(dynoplan)


# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

## db_cbs
add_executable(db_cbs
  src/db_cbs.cpp
  src/dbcbs_utils.cpp
)

target_include_directories(db_cbs PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include
)

target_link_libraries(db_cbs PRIVATE 
  idbastar::optimization
  idbastar::tdbastar
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${FCL_LIBRARIES}
)

# db-ecbs
add_executable(db_ecbs
  src/db_ecbs.cpp
  src/dbcbs_utils.cpp
)

target_include_directories(db_ecbs PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include
)

target_link_libraries(db_ecbs PRIVATE 
  idbastar::optimization
  idbastar::tdbastar_epsilon
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${FCL_LIBRARIES}
)
 