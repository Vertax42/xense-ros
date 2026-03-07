# xensesdkConfig.cmake
# Workaround: the installed SDK only ships xensesdkTargets.cmake without a Config file.
# This file wraps it so that find_package(xensesdk REQUIRED) works correctly.

set(_xensesdk_targets_file "/usr/local/lib/cmake/xensesdk/xensesdkTargets.cmake")

if(NOT EXISTS "${_xensesdk_targets_file}")
  set(xensesdk_FOUND FALSE)
  set(xensesdk_NOT_FOUND_MESSAGE
    "Could not find xensesdk targets at ${_xensesdk_targets_file}. "
    "Make sure libxensesdk is installed (cmake --install ~/libxensesdk/build).")
  return()
endif()

if(NOT TARGET xense::xensesdk)
  include("${_xensesdk_targets_file}")
endif()

set(xensesdk_FOUND TRUE)
set(xensesdk_LIBRARIES xense::xensesdk)
set(xensesdk_INCLUDE_DIRS "/usr/local/include")
set(xensesdk_VERSION "0.0.2")
