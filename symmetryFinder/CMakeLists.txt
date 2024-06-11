cmake_minimum_required(VERSION 3.23)

project(
    symmetryFinder
    VERSION 0.1
)

#Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/app")

add_executable( symmetryFinder )
 target_sources(symmetryFinder PUBLIC
  src/main.cpp
  )
target_include_directories(symmetryFinder PUBLIC "${PROJECT_SOURCE_DIR}/include")



