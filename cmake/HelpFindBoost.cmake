# This file helps users forcing the use of boost provided within BaPCod.
# It should be included in the CMakeList.txt of the application

#set(Boost_NO_BOOST_CMAKE $ENV{Boost_NO_BOOST_CMAKE})
#set(Boost_NO_SYSTEM_PATHS $ENV{Boost_NO_SYSTEM_PATHS})
set(Boost_NO_BOOST_CMAKE  ON)
set(Boost_NO_SYSTEM_PATHS ON)

if ("$ENV{BAPCOD_ROOT}" STREQUAL "")
  #NO set(BOOST_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Tools/boost_1_76_0/build)
  else()
  set(BOOST_ROOT $ENV{BAPCOD_ROOT}/BaPCod/Tools/boost_1_76_0/build)
endif()

#message("FORCING USE OF BOOST IN TOOLS")
 
