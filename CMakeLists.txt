cmake_minimum_required(VERSION 3.5)

project(db-CBS)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(YAML REQUIRED yaml-cpp)
pkg_check_modules(FCL REQUIRED fcl ccd)
pkg_check_modules(OMPL REQUIRED ompl)

# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

## ompl
add_executable(main_ompl
  src/main_ompl.cpp
  src/robots.cpp
)
target_link_libraries(main_ompl
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ${OMPL_LINK_LIBRARIES}
  ${FCL_LIBRARIES}
)
target_include_directories(main_ompl PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

