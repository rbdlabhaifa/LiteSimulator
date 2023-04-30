cmake_minimum_required(VERSION 3.2)
project(LiteSimulator)

set(CMAKE_CXX_STANDARD 17)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Pangolin REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Threads REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS} include/)
add_library(LiteSimulator src/simulator.cpp src/auxiliary.cpp)
target_link_libraries(LiteSimulator
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${Boost_LIBRARIES}
        -lpthread ctello spdlog
        )
add_executable(runSimulatorExample Examples/main.cpp)
target_link_libraries(runSimulatorExample LiteSimulator)
