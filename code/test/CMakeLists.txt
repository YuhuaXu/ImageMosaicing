cmake_minimum_required(VERSION 3.0)

project(test)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)

file(GLOB srcs "*.h" "*.cpp")

add_executable(test ${srcs})

target_link_libraries(test mosaicing
	${OpenCV_LIBS}
)

include_directories(../mosaicing)

set_target_properties(mosaicing PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")