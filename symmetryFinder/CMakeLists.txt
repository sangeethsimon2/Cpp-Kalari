cmake_minimum_required(VERSION 3.23)

project(
    symmetryFinder
    VERSION 0.1
)

#Find necessary packages
find_package(GTest REQUIRED)
include_directories(${GTest_INCLUDE_DIRS})


#Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/app")

#Executable for symmetryFinder app
add_executable( symmetryFinder )
 target_sources(symmetryFinder PUBLIC
  src/main.cpp
  )
#Project include files
target_include_directories(symmetryFinder PUBLIC "${PROJECT_SOURCE_DIR}/include")

#Executable for unitTesting
add_executable( unitTesting )
 target_sources(unitTesting PUBLIC
  src/unitTesting.cpp
  )
#include gtest headers, project headers
target_include_directories(unitTesting PUBLIC
"/home/cfd/simon/.local/include"
"${PROJECT_SOURCE_DIR}/include"
)
#Link against gtest static libraries
target_link_libraries(unitTesting PUBLIC ${GTEST_LIBRARIES})

