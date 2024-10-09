if("${LEMON_INCLUDE_DIR}" STREQUAL "" OR "${LEMON_INCLUDE_DIR}" STREQUAL "LEMON_INCLUDE_DIR-NOTFOUND")

  # Set possible paths to include
  find_path(LEMON_INCLUDE_DIR
    lemon/list_graph.h
    PATHS $ENV{BAPCOD_ROOT}/BaPCod/Tools/lemon-1.3.1/build/include/
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../Tools/lemon-1.3.1/build/include/"
    PATHS "/opt/cluster/plafrim-dev/lemon/1.2.3/include/"
    PATHS "$ENV{LEMON_ROOT}/include/"
    PATHS "${LEMON_ROOT}/include/"
    PATHS "C:/Program Files/LEMON/include")
  
  if (WIN32)
     find_library(LEMON_LIBRARY
        lemon.lib
        PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../Tools/lemon-1.3.1/build/lib/"
        PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../Tools/lemon-1.3.1/build/lemon/${CMAKE_BUILD_TYPE}"
        PATHS "C:/Program Files/LEMON/lib"
        PATHS "$ENV{LEMON_ROOT}/lib/"
        PATHS "${LEMON_ROOT}/lib/")
  else()
      find_library(LEMON_LIBRARY
        libemon.a
        PATHS $ENV{BAPCOD_ROOT}/BaPCod/Tools/lemon-1.3.1/build/lib/
        PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../Tools/lemon-1.3.1/build/lib/"
        PATHS "/opt/cluster/plafrim-dev/lemon/1.2.3/lib/")
  endif()

  mark_as_advanced(LEMON_LIBRARY LEMON_INCLUDE_DIR)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(LEMON DEFAULT_MSG LEMON_INCLUDE_DIR LEMON_LIBRARY)

endif()
