# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_merge_pointclouds_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED merge_pointclouds_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(merge_pointclouds_FOUND FALSE)
  elseif(NOT merge_pointclouds_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(merge_pointclouds_FOUND FALSE)
  endif()
  return()
endif()
set(_merge_pointclouds_CONFIG_INCLUDED TRUE)

# output package information
if(NOT merge_pointclouds_FIND_QUIETLY)
  message(STATUS "Found merge_pointclouds: 0.0.0 (${merge_pointclouds_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'merge_pointclouds' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${merge_pointclouds_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(merge_pointclouds_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${merge_pointclouds_DIR}/${_extra}")
endforeach()
