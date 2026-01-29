# Graphically Recursive Simultaneous Task Allocation, Planning,
# Scheduling, and Execution
#
# Copyright (C) 2020–2023
#
# Author: Andrew Messing
# Author: Glen Neville
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
message("Building tests")

# TODO(Andrew): fix the CI to use gurobi
option(IGNORE_GUROBI "Ignores the gurobi unit tests" OFF)

# Collect headers/sources/data files
add_subdirectory(include)
add_subdirectory(src)

enable_testing()
find_package(GTest 1.11 REQUIRED)
include(GoogleTest)

# Create unit test executable
add_executable(unittests)
target_sources(unittests
        PRIVATE
        ${TEST_HEADERS} ${TEST_SOURCES})
target_compile_features(unittests
        PUBLIC
        cxx_std_20)
target_link_libraries(unittests
        PRIVATE
        ${GTEST_BOTH_LIBRARIES} gmock Threads::Threads
        _${PROJECT_NAME})
target_compile_options(unittests
        PRIVATE
        ${_opts})
target_include_directories(unittests
        PRIVATE
        include
        ${GTEST_INCLUDE_DIRS})

if (IGNORE_GUROBI)
    message("Ignoring Gurobi")
    target_compile_definitions(unittests PUBLIC NO_MILP)
endif (IGNORE_GUROBI)

if (BUILD_COVERAGE)
    target_code_coverage(unittests)
endif (BUILD_COVERAGE)

gtest_discover_tests(unittests)