cmake_minimum_required(VERSION 3.18)
project(motion)
set(CMAKE_CXX_STANDARD 11)
SET(COMPILE_DEFINITIONS -Werror)

include_directories(/opt/vc/include)
include_directories(/opt/vc/include/interface/vcos/pthreads)
include_directories(/opt/vc/include/interface/vmcs_host)
include_directories(/opt/vc/include/interface/vmcs_host/linux)

include_directories(/home/pi/src/raspberrypi/userland)
include_directories(/opt/vc/src/hello_pi/libs/vgfont)

link_directories(/opt/vc/lib)
link_directories(/opt/vc/src/hello_pi/libs/vgfont)

find_package( OpenCV REQUIRED )

find_library(AVCODEC_LIBRARY avcodec)
find_library(AVFORMAT_LIBRARY avformat)
find_library(AVUTIL_LIBRARY avutil)


add_executable(motion main.cpp)
target_link_libraries(motion mmal_core mmal_util mmal_vc_client vcos bcm_host cairo pthread rt m freetype vgfont EGL openmaxil ${OpenCV_LIBS} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY})

