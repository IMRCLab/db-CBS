cmake_minimum_required(VERSION 3.5)

project(db-CBS)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(YAML REQUIRED yaml-cpp)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)



add_subdirectory(dynoplan)

# set(ompl_ROOT ${}/k_cbs/ompl)
# find_package(ompl)
# pkg_check_modules(OMPL REQUIRED ompl)

# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_subdirectory(k_cbs/Kinodynamic-Conflict-Based-Search EXCLUDE_FROM_ALL)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/deps/libMultiRobotPlanning/include
  ${FCL_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/k_cbs/Kinodynamic-Conflict-Based-Search/src
  ${CMAKE_BINARY_DIR}/k_cbs/Kinodynamic-Conflict-Based-Search/src
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(motion_planning_common
  src/robots.cpp
)
target_include_directories(motion_planning_common
  PRIVATE ${CMAKE_BINARY_DIR}/deps/fcl/include
  PRIVATE ${CMAKE_SOURCE_DIR}/deps/fcl/include
)
set_property(TARGET motion_planning_common PROPERTY POSITION_INDEPENDENT_CODE ON)

## ompl
add_executable(main_ompl
  src/main_ompl.cpp
  # src/robots.cpp
)
target_link_libraries(main_ompl
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(main_ompl PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)


## kcbs

add_executable(main_kcbs
  src/main_kcbs.cpp
  # src/robots.cpp
)
target_link_libraries(main_kcbs
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(main_kcbs PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

# ## dbastar

# add_executable(dbastar
#   src/main_dbastar.cpp
  
# )
# target_link_libraries(dbastar
#   motion_planning_common
#   ${Boost_LIBRARIES}
#   ${YAML_LIBRARIES}
#   ompl
#   ${FCL_LIBRARIES}
# )
# target_include_directories(dbastar PUBLIC 
#   ${YAML_INCLUDE_DIRS}
#   ${FCL_INCLUDE_DIRS}
# )

## main_ompl_geometric

add_executable(main_ompl_geometric
  src/main_ompl_geometric.cpp
 
)
target_link_libraries(main_ompl_geometric
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(main_ompl_geometric PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)
## db_cbs

add_executable(db_cbs
  src/db_cbs.cpp
  # src/robots.cpp
)

target_include_directories(db_cbs PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include
)

target_link_libraries(db_cbs PRIVATE 
  idbastar::optimization
  idbastar::tdbastar
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(main_ompl PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

# db-Ecbs
add_executable(db_ecbs
  src/db_ecbs.cpp
)

target_include_directories(db_ecbs PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include
)

target_link_libraries(db_ecbs PRIVATE 
  idbastar::optimization
  idbastar::tdbastar_epsilon
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(main_ompl PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)
# octomap2openscad
add_executable(octomap2openscad
  src/octomap2openscad
)

target_link_libraries(octomap2openscad
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

target_compile_options(octomap2openscad PUBLIC -std=c++11 -Wall)
# octomap_stats
add_executable(octomap_stats
  src/octomap_stats
)

target_link_libraries(octomap_stats
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

target_compile_options(octomap_stats PUBLIC -std=c++11 -Wall)

# discrete_search_test - to debug the low-level focal heuristic
add_executable(discrete_search_test
  src/discrete_search_test.cpp
)

target_include_directories(discrete_search_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include
)

target_link_libraries(discrete_search_test PRIVATE 
  idbastar::optimization
  idbastar::tdbastar_epsilon
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(main_ompl PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

## standalone tdbA*

# add_executable(tdbastar_standalone
#   src/tdbastar_standalone.cpp
# )

# target_include_directories(tdbastar_standalone PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include
# )

# target_link_libraries(tdbastar_standalone PRIVATE 
#   idbastar::optimization
#   idbastar::tdbastar
#   motion_planning_common
#   ${Boost_LIBRARIES}
#   ${YAML_LIBRARIES}
#   ompl
#   ${FCL_LIBRARIES}
# )
# target_include_directories(main_ompl PUBLIC 
#   ${YAML_INCLUDE_DIRS}
#   ${FCL_INCLUDE_DIRS}
# )
