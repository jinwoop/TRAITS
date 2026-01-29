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

# Check if Doxygen is installed
find_package(Doxygen)

if (DOXYGEN_FOUND)
    message("Building documentation...")

    # Set input and output files
    set(doxygen_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(doxygen_out ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # Request to configure the files
    configure_file(${doxygen_in} ${doxygen_out})

    add_custom_target(documentation ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxygen_out}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
else (DOXYGEN_FOUND)
    message("Doxygen needs to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)