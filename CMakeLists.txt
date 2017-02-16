cmake_minimum_required(VERSION 2.6)

project(Toporecognizer C CXX)

set(CMAKE_BUILD_DIR "build")

set(MY_SOURCES main.cpp TriplelinePointsetClass.cpp LocalPointset.cpp LinearCurve.cpp Plane.cpp Point3D.cpp PointCloudGenerator.cpp QuadraticCurve.cpp Spline.cpp)

set(CMAKE_CXX_FLAGS "-g -O0 -Wall")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}") 
ADD_DEFINITIONS("-std=c++0x")

add_executable(TopoRecognizer ${MY_SOURCES})
include_directories("${CMAKE_SOURCE_DIR}")
ADD_CUSTOM_TARGET (clearAll
	COMMAND ${CMAKE_COMMAND} -E remove *.png *.gnu *.ori Level* *.txt *.gif
	COMMAND ${CMAKE_COMMAND} -E echo "rm *.png *.gnu *.ori *.gif"
)

ADD_CUSTOM_TARGET (rm_cmake
	COMMAND ${CMAKE_COMMAND} -E remove CMakeFiles/
)

