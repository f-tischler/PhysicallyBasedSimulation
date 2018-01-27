cmake_minimum_required(VERSION 3.5.1)
project(FinalProject LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Debug)

if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++14 /wd4141")
	set(CMAKE_CXX_FLAGS_RELEASE "/Ox /Ot /DNDEBUG")
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

set(SOURCE_FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/Main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Polygon.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Console.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/ContactInfo.hpp
)

add_executable(FinalProject ${SOURCE_FILES}  ${HEADERS})

target_include_directories(FinalProject PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})

set(SFML_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/SFML-2.4.2)

find_package(SFML 2 REQUIRED network audio graphics window system)


if(SFML_FOUND)
	include_directories(${SFML_INCLUDE_DIR})
	target_link_libraries(FinalProject ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
endif()