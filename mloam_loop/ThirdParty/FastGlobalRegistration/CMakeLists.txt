cmake_minimum_required(VERSION 3.0.0)

project(FastGlobalRegistration)

set(FastGlobalRegistration_VERSION_MAJOR "1")
set(FastGlobalRegistration_VERSION_MINOR "0")

# Build either a STATIC or SHARED library
if (NOT FastGlobalRegistration_LINK_MODE)
  set(FastGlobalRegistration_LINK_MODE "STATIC")
endif(NOT FastGlobalRegistration_LINK_MODE)
message(STATUS "Libary linkage will be " ${FastGlobalRegistration_LINK_MODE})

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# default built type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
		FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Set OS-specific things here
if(WIN32)
	add_definitions(-DWINDOWS)
	message(STATUS "Compiling on Windows")
	if(MSVC)
		message(STATUS "Compiling with MSVC")
		add_definitions(-DNOMINMAX)
		add_definitions(-D_USE_MATH_DEFINES)
		add_definitions(-D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE -D_SCL_SECURE_NO_WARNINGS)		# suppress C4996 warning
		add_definitions(/wd4267 /wd4244 /wd4305 /wd4522)
	endif(MSVC)
elseif(CYGWIN)
	message(STATUS "Compiling on Cygwin")
	# enable c++11
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	add_definitions(-DCYGWIN)
elseif(APPLE)
	add_definitions(-DUNIX)
	# enable c++11
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	# compile speed optimization for clang
	add_definitions(-Os)
	# disable OpenMP since it is not currently supported on OSX
	message(STATUS "Compiling on OSX")
elseif(UNIX)
	add_definitions(-DUNIX)
	add_compile_options(-Wno-deprecated-declarations)
	add_compile_options(-Wno-unused-result)
	# enable c++11
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(WIN32)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(FastGlobalRegistrationLib ${FastGlobalRegistration_LINK_MODE} app.cpp app.h)
add_executable(FastGlobalRegistration main.cpp)
add_executable(Evaluation evaluation.cpp)
TARGET_LINK_LIBRARIES(FastGlobalRegistration FastGlobalRegistrationLib ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(Evaluation FastGlobalRegistrationLib ${PCL_LIBRARIES})

add_executable(create_input create_input.cpp)
TARGET_LINK_LIBRARIES(create_input FastGlobalRegistrationLib ${PCL_LIBRARIES})

