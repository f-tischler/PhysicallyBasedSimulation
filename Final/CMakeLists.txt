cmake_minimum_required(VERSION 3.5.1)
project(FinalProject LANGUAGES CXX)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})

if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++1z /wd4141")
	set(CMAKE_CXX_FLAGS_RELEASE "/Ox /Ot /DNDEBUG")
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z -Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

file(GLOB SOURCE_FILES
    "*.h"
    "*.cpp"
)

add_executable(FinalProject ${SOURCE_FILES}  ${HEADERS})

target_include_directories(FinalProject PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

set(SFML_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/SFML-2.4.2)

find_package(SFML 2 REQUIRED network audio graphics window system)
include_directories(${SFML_INCLUDE_DIR})
target_link_libraries(FinalProject ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(FinalProject ${EIGEN3_LIBRARIES} ${EIGEN3_DEPENDENCIES})
