# Graphically Recursive Simultaneous Task Allocation, Planning,
# Scheduling, and Execution
#
# Modeling and Optimizing the Provisioning of Exhaustible Capabilities
# for Simultaneous Task Allocation and Scheduling
#
# Author: Andrew Messing
# Author: Glen Neville
# Author: Jinwoo Park
#
# Copyright (C) 2020–2023 Andrew Messing
# Copyright (C) 2020–2023 Glen Neville
# Copyright (C) 2026 Jinwoo Park
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


cmake_minimum_required(VERSION 3.16)
project(traits
        VERSION 1.0.0
        DESCRIPTION "Trait Throughput and Allocation Informed Team Scheduling (TRAITS)"
        LANGUAGES CXX
        )
message("Builiding ${PROJECT_NAME}...")

## Versioning guide
## version major increases with each publication
## version minor increases with a new feature
## version patch increases with smaller changes that need to be distinguished

## Store the language
get_property(LANGUAGES GLOBAL PROPERTY ENABLED_LANGUAGES)

set(DEFAULT_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

include(GNUInstallDirs)
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

## Add external cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Determine if this project is built as a subproject (using add_subdirectory)
## or if it is the master project
set(MASTER_PROJECT OFF)
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(MASTER_PROJECT ON)
    message(STATUS "CMake version: ${CMAKE_VERSION}")
endif (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)

## Options
include(CMakeDependentOption)
# --- Documentation Build Option ---
option(BUILD_DOCUMENTATION 
    "Whether to build the documentation (Note: gets built in the docs folder in the source)" 
    ${MASTER_PROJECT})

#cmake_policy(SET CMP0127 NEW)
string(TOUPPER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_UPPER)  # Uppercase Starts
# --- Unit Test Build Option ---
cmake_dependent_option(BUILD_UNITTESTS 
    "Whether to build the unit tests (Note: this only works for DEBUG builds)"
    ON
    "${MASTER_PROJECT};'${CMAKE_BUILD_TYPE_UPPER}' STREQUAL 'DEBUG'"
    OFF)
#message(${BUILD_UNITTESTS})

# --- Code Coverage Build Option ---
cmake_dependent_option(BUILD_COVERAGE
    "Whether to bulid the coverage output (Note: this only works for DEBUG builds)" 
    OFF 
    "'${CMAKE_BUILD_TYPE_UPPER}' STREQUAL 'DEBUG'"
    OFF)
# --- Optimization Profiling Option ---
cmake_dependent_option(USE_PROFILING
    "Whether to set the optimization level for profiling (Note: this only works for DEBUG builds)" 
    OFF 
    "${MASTER_PROJECT};'${CMAKE_BUILD_TYPE_UPPER}' STREQUAL 'DEBUG'"
    OFF)
unset(CMAKE_BUILD_TYPE_UPPER)  # Uppercase Ends

option(BUILD_EXECUTABLES
    "Whether to build the executables in the executable folder"
    OFF)

## Basically string options
set(DATA_DIR 
    "${CMAKE_CURRENT_SOURCE_DIR}/data" 
    CACHE PATH 
    "Directory to the data folder")

## Add external modules
add_subdirectory(extern)
find_package(OpenMP)

## Add pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Place to fix an issue with fmt and pybind11
# See: https://en.wikipedia.org/wiki/Position-independent_code
# See: https://github.com/fmtlib/fmt/issues/2456
# See: https://github.com/fmtlib/fmt/issues/548
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)

## Collect header and source files
add_subdirectory(include)
add_subdirectory(src)

## Create Library (_traits)
add_library(_${PROJECT_NAME} STATIC)
target_sources(_${PROJECT_NAME}
        PUBLIC
        ${HEADERS}
        PRIVATE
        ${SOURCES})
    
target_precompile_headers(_${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/config.hpp>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}/config.hpp>)

target_compile_features(_${PROJECT_NAME}
        PRIVATE
        cxx_std_20)
target_link_libraries(_${PROJECT_NAME}
        PUBLIC
        cli11
        concurrencpp::concurrencpp
        duck_invoke
        fmt::fmt
        gurobi
        magic_enum
        nlohmann_json::nlohmann_json
        ompl
        OpenMP::OpenMP_CXX
        robin_hood
        spdlog::spdlog
        yaml-cpp
        )
target_include_directories(_${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_options(_${PROJECT_NAME}
        PUBLIC
        "-fcoroutines"
        "-fvisibility=hidden"
        "-Werror=return-type"
        "$<$<AND:$<CONFIG:DEBUG>,$<BOOL:${USE_PROFILING}>>:-O0>"
        "$<$<CONFIG:RELEASE>:-Ofast>")
target_compile_definitions(_${PROJECT_NAME}
        PUBLIC
        "$<$<CONFIG:DEBUG>:DEBUG>")


## Build documentation
if (BUILD_DOCUMENTATION)
    add_subdirectory(docs)
endif (BUILD_DOCUMENTATION)

## Build unit tests
if (BUILD_UNITTESTS)
    add_subdirectory(tests)
endif (BUILD_UNITTESTS)

## Build executables
if (BUILD_EXECUTABLES)
    add_subdirectory(executables)
endif (BUILD_EXECUTABLES)
