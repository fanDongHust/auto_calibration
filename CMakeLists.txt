cmake_minimum_required(VERSION 2.8)

project(CamPoseAutoCalibrate)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_MODULE_PATH "/path")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV 3.1 REQUIRED )
#find_package( g2o REQUIRED )
find_package( Sophus REQUIRED )
set(Sophus_LIBRARIES libSophus.so)

include_directories( 
    ${OpenCV_INCLUDE_DIRS}
#    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3/"
	${Sophus_INCLUDE_DIRS}
)



#file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
#add_library(sw_calib SHARED ${SOURCES})

add_library(line_extractor line_extractor.cpp)
add_library(line_extractorShared SHARED line_extractor.cpp)
target_link_libraries(line_extractor 
	${OpenCV_LIBS}
	${Sophus_LIBRARIES}
)

#add_library(calibration calibration.cpp)
#add_library(calibrationShared SHARED calibration.cpp)
#target_link_libraries(calibration 
#	${OpenCV_LIBS}
#	${Sophus_LIBRARIES})

	

add_executable(main main.cpp)
target_link_libraries(main
	${OpenCV_LIBS}
#	${G2O_LIBS}
#	g2o_core
	line_extractor
#	calibration
#	${Sophus_LIBRARIES}
)
	






