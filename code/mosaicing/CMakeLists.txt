cmake_minimum_required(VERSION 3.0)

project(mosaicing)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)

file(GLOB SRCS "*.h" "*.cpp")

#add_executable(image_seg ${srcs})
add_library(mosaicing STATIC
        ${SRCS})

target_link_libraries(mosaicing
	${OpenCV_LIBS}
)

set_target_properties(mosaicing PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")