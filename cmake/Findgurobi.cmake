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

find_path(GUROBI_INCLUDE_DIRECTORY
        NAMES
        gurobi_c.h
        gurobi_c++.h
        HINTS
        ${GUROBI_DIR}
        $ENV{GUROBI_HOME}
        /opt/gurobi/linux64
        /opt/gurobi/armlinux64
        /opt/gurobi911/linux64
        /opt/gurobi912/linux64
        /opt/gurobi950/linux64
        /opt/gurobi110/linux64
        /opt/gurobi110/armlinux64
        PATH_SUFFIXES
        include)

find_library(GUROBI_LIBRARY
        NAMES
        gurobi
        gurobi91
        gurobi95
        gurobi110
        HINTS
        ${GUROBI_DIR}
        $ENV{GUROBI_HOME}
        /opt/gurobi/linux64
        /opt/gurobi/armlinux64
        /opt/gurobi911/linux64
        /opt/gurobi912/linux64
        /opt/gurobi950/linux64
        /opt/gurobi110/linux64
        /opt/gurobi110/armlinux64
        PATH_SUFFIXES
        lib)

find_library(GUROBI_CXX_LIBRARY
        NAMES
        gurobi_c++
        HINTS
        ${GUROBI_DIR}
        $ENV{GUROBI_HOME}
        /opt/gurobi/linux64
        /opt/gurobi/armlinux64
        /opt/gurobi911/linux64
        /opt/gurobi912/linux64
        /opt/gurobi950/linux64
        /opt/gurobi110/linux64
        /opt/gurobi110/armlinux64
        PATH_SUFFIXES
        lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(gurobi REQUIRE_VARS GUROBI_CXX_LIBRARY GUROBI_LIBRARY GUROBI_INCLUDE_DIRECTORY)

if (GUROBI_CXX_LIBRARY AND GUROBI_LIBRARY AND GUROBI_INCLUDE_DIRECTORY)
    set(gurobi_FOUND TRUE)
endif (GUROBI_CXX_LIBRARY AND GUROBI_LIBRARY AND GUROBI_INCLUDE_DIRECTORY)

if (gurobi_FOUND)
    add_library(gurobi_c UNKNOWN IMPORTED GLOBAL)
    set_target_properties(gurobi_c PROPERTIES
            IMPORTED_CONFIGURATIONS RELEASE
            IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
            IMPORTED_LOCATION_RELEASE ${GUROBI_LIBRARY}
            INTERFACE_INCLUDE_DIRECTORIES ${GUROBI_INCLUDE_DIRECTORY}
            IMPORTED_GLOBAL ON)

    add_library(gurobi UNKNOWN IMPORTED GLOBAL)
    set_target_properties(gurobi PROPERTIES
            IMPORTED_CONFIGURATIONS RELEASE
            IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
            IMPORTED_LOCATION_RELEASE ${GUROBI_CXX_LIBRARY}
            INTERFACE_LINK_LIBRARIES gurobi_c
            INTERFACE_INCLUDE_DIRECTORIES ${GUROBI_INCLUDE_DIRECTORY}
            IMPORTED_GLOBAL ON)
endif (gurobi_FOUND)