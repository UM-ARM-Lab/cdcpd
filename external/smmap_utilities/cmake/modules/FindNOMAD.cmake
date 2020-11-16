## Copied from FindGUROBI.cmake


# - Try to find NOMAD
# Once done this will define
#  NOMAD_FOUND - System has NOMAD
#  NOMAD_INCLUDE_DIRS - The NOMAD include directories
#  NOMAD_LIBRARIES - The libraries needed to use NOMAD

find_path(NOMAD_INCLUDE_DIR NAMES nomad.hpp PATHS "$ENV{NOMAD_HOME}/src")
find_path(SGTELIB_INCLUDE_DIR NAMES sgtelib.hpp PATHS "$ENV{NOMAD_HOME}/ext/sgtelib/src")

find_library(NOMAD_LIBRARY NAMES nomad PATHS "$ENV{NOMAD_HOME}/lib")
find_library(SGTELIB_LIBRARY NAMES sgtelib PATHS "$ENV{NOMAD_HOME}/ext/sgtelib/lib")

set(NOMAD_INCLUDE_DIRS "${NOMAD_INCLUDE_DIR};${SGTELIB_INCLUDE_DIR}")
set(NOMAD_LIBRARIES "${NOMAD_LIBRARY};${SGTELIB_LIBRARY}")


#message(STATUS "-------------${NOMAD_INCLUDE_DIRS}")
#message(STATUS "-------------${NOMAD_LIBRARIES}")

# use c++ headers as default
# set(NOMAD_COMPILER_FLAGS "-DIL_STD" CACHE STRING "NOMAD Compiler Flags")

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBCPLEX_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(NOMAD DEFAULT_MSG NOMAD_LIBRARY NOMAD_INCLUDE_DIR SGTELIB_LIBRARY SGTELIB_INCLUDE_DIR)

mark_as_advanced(NOMAD_INCLUDE_DIR NOMAD_LIBRARY SGTELIB_LIBRARY SGTELIB_INCLUDE_DIR)
