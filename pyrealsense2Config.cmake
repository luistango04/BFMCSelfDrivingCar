
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was pyrealsense2Config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set(pyrealsense2_VERSION_MAJOR "2")
set(pyrealsense2_VERSION_MINOR "53")
set(pyrealsense2_VERSION_PATCH "1")

set(pyrealsense2_VERSION ${realsense2_VERSION_MAJOR}.${realsense2_VERSION_MINOR}.${realsense2_VERSION_PATCH})

include("${CMAKE_CURRENT_LIST_DIR}/pyrealsense2Targets.cmake")
set(realsense2_LIBRARY pyrealsense2::pyrealsense2)
