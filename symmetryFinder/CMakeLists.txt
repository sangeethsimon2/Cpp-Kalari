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

add_executable( symmetryFinder )
 target_sources(symmetryFinder PUBLIC
  src/main.cpp
  )
target_include_directories(symmetryFinder PUBLIC "${PROJECT_SOURCE_DIR}/include")

add_executable( unitTesting )
 target_sources(unitTesting PUBLIC
  src/unitTesting.cpp
  )
target_include_directories(unitTesting PUBLIC "/home/cfd/simon/.local/include")
target_link_libraries(unitTesting PUBLIC ${GTEST_LIBRARIES})

