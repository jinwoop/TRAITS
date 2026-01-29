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

## Third Party Libraries

# Use: concurrencpp
if (NOT TARGET concurrencpp)
    find_package(concurrencpp REQUIRED CONFIG)
    set_target_properties(concurrencpp::concurrencpp PROPERTIES IMPORTED_GLOBAL ON)
endif (NOT TARGET concurrencpp)

# Use: range-v3
if (NOT TARGET range-v3)
    find_package(range-v3 REQUIRED CONFIG)
    set_target_properties(range-v3 PROPERTIES IMPORTED_GLOBAL ON)
endif (NOT TARGET range-v3)

# Use: Formatting of strings (Will be the implementation for string formatting in C++20)
if (NOT TARGET fmt)
    find_package(fmt REQUIRED CONFIG)
    set_target_properties(fmt::fmt PROPERTIES IMPORTED_GLOBAL ON)
endif (NOT TARGET fmt)

# Use: Logger
if (NOT TARGET spdlog)
    find_package(spdlog REQUIRED CONFIG)
    set_target_properties(spdlog::spdlog 
        PROPERTIES 
        IMPORTED_GLOBAL ON
        POSITION_INDEPENDENT_CODE ON)
endif (NOT TARGET spdlog)

# Use: json and a "any" alternative
if (NOT TARGET nlohmann_json)
    find_package(nlohmann_json 
                    3.10.2 
                    REQUIRED CONFIG)
    set_target_properties(nlohmann_json::nlohmann_json
        PROPERTIES 
        IMPORTED_GLOBAL ON)
endif (NOT TARGET nlohmann_json)

# Use yaml parsing
if (NOT TARGET yaml-cpp)
    find_package(yaml-cpp REQUIRED CONFIG)
    set_target_properties(yaml-cpp PROPERTIES IMPORTED_GLOBAL ON)
endif (NOT TARGET yaml-cpp)

# Use: Mixed-Integer Linear Programming Solver
if (NOT TARGET gurobi)
    find_package(gurobi REQUIRED)
endif (NOT TARGET gurobi)

# Use: Matrix algebra library
if (NOT TARGET Eigen3)
    # Install via apt
    find_package(Eigen3 3.3 REQUIRED CONFIG)
endif (NOT TARGET Eigen3)

# Use: Motion Planning library
if (NOT TARGET ompl)
    # Build and install
    find_package(ompl 1.5.2 REQUIRED CONFIG)
endif (NOT TARGET ompl)

# Use: fast maps and sets
if (NOT TARGET robin_hood)
    add_subdirectory(robin_hood)
endif (NOT TARGET robin_hood)

# Use: enum reflection
if (NOT TARGET magic_enum)
    add_subdirectory(magic_enum)
endif (NOT TARGET magic_enum)

# Use: command line arguments
if (NOT TARGET cli11)
    add_subdirectory(cli11)
endif (NOT TARGET cli11)

# Use: tag invoke
if (NOT TARGET duck_invoke)
    add_subdirectory(duck_invoke)
endif (NOT TARGET duck_invoke)

list(APPEND LCOV_REMOVE_PATTERNS "'${CMAKE_CURRENT_SOURCE_DIR}/*' ")