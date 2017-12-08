##############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

find_package(OpenCV   REQUIRED)

link_directories(/usr/local/lib)
add_executable(camera_pusher ${SOURCES})
add_dependencies(camera_pusher kobuki_msgs_gencpp)
target_link_libraries(camera_pusher 
			${catkin_LIBRARIES}
			${OpenCV_LIBS}
			OpenNI2
			avformat
			avdevice
			avcodec
			avutil
			pthread
			swscale
			avfilter
			swresample
			z
			m
			lzma
			bz2
			rt
			theoraenc
			theoradec
			x264
			)

install(TARGETS camera_pusher
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
