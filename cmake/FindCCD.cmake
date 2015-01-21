# Find CCD
#
# This sets the following variables:
# CCD_FOUND
# CCD_INCLUDE_DIRS
# CCD_LIBRARIES
# CCD_VERSION

find_package(PkgConfig QUIET)
pkg_check_modules(PC_CCD ccd)

find_path(CCD_INCLUDE_DIR ccd/ccd.h
          HINTS ${PC_CCD_INCLUDEDIR}
          PATHS "${CMAKE_INSTALL_PREFIX}/include")

find_library(CCD_LIBRARY NAMES ccd
             HINTS ${PC_CCD_LIBDIR})

set(CCD_LIBRARIES    ${CCD_LIBRARY})
set(CCD_INCLUDE_DIRS ${CCD_INCLUDE_DIR})
set(CCD_VERSION      ${PC_CCD_VERSION})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CCD DEFAULT_MSG
                                  CCD_LIBRARY CCD_INCLUDE_DIR)

mark_as_advanced(CCD_INCLUDE_DIR CCD_LIBRARY)

