cmake_minimum_required(VERSION 3.5)

project(db-CBS)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(YAML REQUIRED yaml-cpp)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)
set(OMPL_DIR "/usr/local/share/ompl/cmake")
find_package(ompl REQUIRED)

message(STATUS "OMPL_FOUND: ${OMPL_FOUND}")
message(STATUS "OMPL_INCLUDE_DIRS: ${OMPL_INCLUDE_DIRS}")
message(STATUS "OMPL_LIBRARIES: ${OMPL_LIBRARIES}")
message(STATUS "OMPL_VERSION: ${OMPL_VERSION}")

find_library(
  LZ4_LIBRARIES
  NAMES liblz4.so
  HINTS "lz4/lib")
if(LZ4_LIBRARIES)
  message(STATUS "Found: ${LZ4_LIBRARIES}")
else()
  message(STATUS "Not found: ${LZ4_LIBRARIES}")
  message(FATAL_ERROR "Cannot find required LZ4 libraries")
endif()

add_subdirectory(dynoplan)

# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# add_subdirectory(k_cbs/Kinodynamic-Conflict-Based-Search EXCLUDE_FROM_ALL)
# include_directories(
#   ${EIGEN3_INCLUDE_DIR}
#   ${CMAKE_SOURCE_DIR}/deps/libMultiRobotPlanning/include
#   ${FCL_INCLUDE_DIRS}
#   ${CMAKE_SOURCE_DIR}/k_cbs/Kinodynamic-Conflict-Based-Search/src
#   ${CMAKE_BINARY_DIR}/k_cbs/Kinodynamic-Conflict-Based-Search/src
#   ${OCTOMAP_INCLUDE_DIRS}
# )

# db_cbs
add_executable(db_cbs
  src/db_cbs.cpp
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

# sst*
add_executable(main_ompl
  src/main_ompl.cpp
  src/robots.cpp
)
target_link_libraries(main_ompl
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${FCL_LIBRARIES}
)
target_include_directories(main_ompl PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# kcbs
# add_executable(main_kcbs
#   src/main_kcbs.cpp
#   src/robots.cpp
# )

# target_link_libraries(main_kcbs PUBLIC
#   ${Boost_LIBRARIES}
#   ${YAML_LIBRARIES}
#   ${FCL_LIBRARIES}
#   ompl
# )

# target_include_directories(main_kcbs PUBLIC 
#   ${YAML_INCLUDE_DIRS}
#   ${FCL_INCLUDE_DIRS}
# )

# octomap2openscad
# add_executable(octomap2openscad
#   src/octomap2openscad
# )

# target_link_libraries(octomap2openscad
#   ${Boost_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
# )

# target_compile_options(octomap2openscad PUBLIC -std=c++11 -Wall)
# # octomap_stats
# add_executable(octomap_stats
#   src/octomap_stats
# )

# target_link_libraries(octomap_stats
#   ${Boost_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
# )

# target_compile_options(octomap_stats PUBLIC -std=c++11 -Wall)