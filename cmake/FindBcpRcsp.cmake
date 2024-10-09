# try to find installed version

find_path(BCP_RCSP_INCLUDE_DIR
	  NAMES rcsp_interface.hpp
	  PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../Tools/rcsp/include"
	  PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../Tools/rcsp/build/include"
	  PATHS "${CMAKE_SOURCE_DIR}/Tools/rcsp/include"
	  PATHS "${CMAKE_SOURCE_DIR}/Tools/rcsp/build/include"
	  )


if(WIN32)
    find_library(BCP_RCSP_LIBRARY
            NAMES rcsp rcspwindows
            PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../Tools/rcsp/lib"
            PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../Tools/rcsp/build/lib"
            PATHS "${CMAKE_SOURCE_DIR}/Tools/rcsp/lib"
            PATHS "${CMAKE_SOURCE_DIR}/Tools/rcsp/build/lib"
            )
elseif(APPLE)
    find_library(BCP_RCSP_LIBRARY
            NAMES rcsp rcspmacos
            PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../Tools/rcsp/lib"
            PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../Tools/rcsp/build/lib"
            PATHS "${CMAKE_SOURCE_DIR}/Tools/rcsp/lib"
            PATHS "${CMAKE_SOURCE_DIR}/Tools/rcsp/build/lib"
            )
elseif(UNIX)
    find_library(BCP_RCSP_LIBRARY
            NAMES rcsp rcsplinux
            PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../Tools/rcsp/lib"
            PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../Tools/rcsp/build/lib"
            PATHS "${CMAKE_SOURCE_DIR}/Tools/rcsp/lib"
            PATHS "${CMAKE_SOURCE_DIR}/Tools/rcsp/build/lib"
            )
endif()

if ((NOT "${BCP_RCSP_INCLUDE_DIR}" STREQUAL "BCP_RCSP_INCLUDE_DIR-NOTFOUND")
    AND (NOT "${BCP_RCSP_LIBRARY}" STREQUAL "BCP_RCSP_LIBRARY-NOTFOUND"))
    set(RCSP_VERSION "" CACHE INTERNAL "")
    file(STRINGS "${BCP_RCSP_INCLUDE_DIR}/rcsp_version.hpp" BCP_RCSP_VERSION REGEX "[0-9]+.[0-9]+.[0-9]+")
    string(REGEX REPLACE "[^0-9.]+" "" BCP_RCSP_VERSION ${BCP_RCSP_VERSION})
    message(STATUS "The value of bcp is: ${BCP_RCSP_VERSION}")
    message("-- Found optional BCP_RCSP library version ${BCP_RCSP_VERSION}")
    add_definitions(-DBCP_RCSP_IS_FOUND)
    get_filename_component(BCP_RCSP_LIBRARY_DIRS "${BCP_RCSP_LIBRARY}" PATH)
    
else()
    # use sources ?

    find_path(BCP_RCSP_INCLUDE_DIR
              NAMES rcsp_interface.hpp
              PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../Tools/rcsp/include_dev"
	      PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../Tools/rcsp/build/include_dev"
	      )

    if (NOT "${BCP_RCSP_INCLUDE_DIR}" STREQUAL "BCP_RCSP_INCLUDE_DIR-NOTFOUND")
       add_definitions(-DBCP_RCSP_IS_FOUND)
       set(BCP_RCSP_LIBRARY_DIRS "")
       set(BCP_RCSP_LIBRARY     "rcsp")
       message("-- Found sources of optional BCP_RCSP library")    
     else()
	# not found
    	set(BCP_RCSP_LIBRARY_DIRS "")
	set(BCP_RCSP_LIBRARY     "")
    	set(BCP_RCSP_INCLUDE_DIR "")
    endif()
endif()

#include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(BcpRcsp "Could not find optional BCP_RCSP lib" BCP_RCSP_INCLUDE_DIR BCP_RCSP_LIBRARY)

mark_as_advanced(BCP_RCSP_INCLUDE_DIR BCP_RCSP_LIBRARY_DIRS BCP_RCSP_LIBRARY) 

#message("BcpRcsp search results : " ${BCP_RCSP_INCLUDE_DIR} " _ " ${BCP_RCSP_LIBRARY} " _ " ${BCP_RCSP_LIBRARY_DIRS})
