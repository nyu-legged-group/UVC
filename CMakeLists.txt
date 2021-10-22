cmake_minimum_required(VERSION 3.16.3)
project(Biped)

add_compile_options(-std=c++11)
SET(CMAKE_VERBOSE_MAKEFILE on )
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${EXECUTABLE_OUTPUT_PATH})

find_package(OpenGL)
find_package(Threads)

include_directories(
  include
  /usr/local/include
)

add_library(biped STATIC
            src/core.cpp
            )

add_executable(test_humanoid_realtime src/biped.cpp)
target_link_libraries(test_humanoid_realtime biped -lode -ldrawstuff -lX11 ${OPENGL_INCLUDE_DIR} ${OPENGL_LIBRARIES})
