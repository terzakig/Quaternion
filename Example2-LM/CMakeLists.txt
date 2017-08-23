project(Example2)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)


# OpenCV stuff... (libs and includes)
set( OpenCV_INCLUDE_DIRS
	"/usr/local/include/opencv"
	"/usr/local/include/opencv2"
   )
# There are more than we need, but better be safe...
set( OpenCV_LIBS
	"/usr/local/lib/libopencv_calib3d.so"
	"/usr/local/lib/libopencv_core.so"
	"/usr/local/lib/libopencv_features2d.so"
	"/usr/local/lib/libopencv_flann.so"
	"/usr/local/lib/libopencv_highgui.so"
	#"/usr/local/lib/libopencv_imgcodecs.so"
	"/usr/local/lib/libopencv_imgproc.so"
	"/usr/local/lib/libopencv_ml.so"
	"/usr/local/lib/libopencv_objdetect.so"
	"/usr/local/lib/libopencv_photo.so"
	#"/usr/local/lib/libopencv_shape.so"
	"/usr/local/lib/libopencv_stitching.so"
	"/usr/local/lib/libopencv_superres.so"
	"/usr/local/lib/libopencv_video.so"
	#"/usr/local/lib/libopencv_videoio.so"
	"/usr/local/lib/libopencv_videostab.so"
	
   )

## OpenCV libs and header definitions end here


set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

include_directories(${OpenCV_INCLUDE_DIRS})



set(sources example2.cpp)


add_executable(example2 ${sources})

target_link_libraries(example2 ${OpenCV_LIBS})