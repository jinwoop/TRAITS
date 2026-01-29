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

file(GLOB_RECURSE HEADERS **.hpp)
set(HEADERS ${HEADERS} PARENT_SCOPE)

# Configure config file
configure_file(
        "${PROJECT_SOURCE_DIR}/include/traits/config.hpp.in"
        "${PROJECT_BINARY_DIR}/include/traits/config.hpp"
        @ONLY
)

# Create core file (Is this bad for any reason? Compile Time?)
set(CORE_INCLUDES "")
foreach (INCLUDE IN ITEMS ${HEADERS})
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/" "" INCLUDE "${INCLUDE}")
    string(APPEND CORE_INCLUDES "#include \"${INCLUDE}\"\n")
endforeach ()

configure_file(
        "${PROJECT_SOURCE_DIR}/include/traits/core.hpp.in"
        "${PROJECT_BINARY_DIR}/include/traits/core.hpp"
        @ONLY
)
