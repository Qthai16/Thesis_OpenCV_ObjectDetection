cmake_minimum_required(VERSION 2.8)
project( main )
find_package( OpenCV REQUIRED )

option(BUILD_ON_PI 	"Option description" ON)
option(BUILD_ON_HOST 	"Option description" OFF)

include(svm_sign_detect/dlib/cmake)

set(CMAKE_BUILD_TYPE Release)
set(DLIB_NO_GUI_SUPPORT OFF)

MACRO(add_source name)
    ADD_EXECUTABLE(${name} svm_sign_detect/src/${name}.cpp)
    TARGET_LINK_LIBRARIES(${name} dlib )
    TARGET_LINK_LIBRARIES(${name} ${OpenCV_LIBS} )
ENDMACRO()

if(BUILD_ON_PI)
	SET(GCC_COVERAGE_COMPILE_FLAGS "-g -pthread -lwiringPi")
	SET(GCC_COVERAGE_LINK_FLAGS    "-g -pthread -lwiringPi")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
	
	SET(SOURCE_FILES src/main.cpp src/pi_lane_detect.cpp src/control.cpp src/utility.cpp src/raspi_uart.cpp)

	add_library(SignDetect svm_sign_detect/src/sign_detect.cpp)
	target_link_libraries(SignDetect dlib ${OpenCV_LIBS})

	message(STATUS "BUILD ON PI")
	add_executable( main ${SOURCE_FILES})
	target_link_libraries( main ${OpenCV_LIBS} SignDetect)
endif(BUILD_ON_PI)

if(BUILD_ON_HOST)
	SET(GCC_COVERAGE_COMPILE_FLAGS "-g -pthread")
	SET(GCC_COVERAGE_LINK_FLAGS    "-g -pthread")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	
	SET(SOURCE_FILES src/main.cpp src/pi_lane_detect.cpp src/control.cpp src/utility.cpp src/jetson_uart.cpp)

	add_library(SignDetect svm_sign_detect/src/sign_detect.cpp)
	target_link_libraries(SignDetect dlib ${OpenCV_LIBS})

	message(STATUS "BUILD ON HOST")
	add_executable( main ${SOURCE_FILES})
	target_link_libraries( main ${OpenCV_LIBS} SignDetect)
endif(BUILD_ON_HOST)





