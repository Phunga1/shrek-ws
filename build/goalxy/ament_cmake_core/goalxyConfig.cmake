# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_goalxy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED goalxy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(goalxy_FOUND FALSE)
  elseif(NOT goalxy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(goalxy_FOUND FALSE)
  endif()
  return()
endif()
set(_goalxy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT goalxy_FIND_QUIETLY)
  message(STATUS "Found goalxy: 0.0.0 (${goalxy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'goalxy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${goalxy_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(goalxy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${goalxy_DIR}/${_extra}")
endforeach()
