# Set CMake minimum requirement:
cmake_minimum_required(VERSION 3.15.0)

# Set project version information:
project(EVA VERSION 1.0 LANGUAGES C CXX)

option(CXX "enable C++ compilation" ON)

# Specify the C++ standard:
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CXX)
  enable_language(CXX)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}.")

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT ")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()

if ( CMAKE_COMPILER_IS_GNUCC )
	message("Apply compiler flags:")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fPIC")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

	set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -fPIC" )
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
endif()

# Set Optmimization Flags:
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set Path to Boost:
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
#set(Boost_NO_SYSTEM_PATHS ON)
#set(BOOST_ROOT $ENV{BOOST_ROOT})

# Load the relevant components for the project:
find_package(Boost 1.84 COMPONENTS graph filesystem program_options REQUIRED)

# Link OpenMP for multi-threading
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

find_package(HiGHS REQUIRED)

# Check if the code is run in "Release" or "Debug":
IF(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_definitions(RELEASE_BUILD)
ELSE()
	add_compile_definitions(DEBUG_BUILD)
ENDIF(CMAKE_BUILD_TYPE MATCHES "Release")

add_subdirectory(libs)
add_subdirectory(app)




