cmake_minimum_required(VERSION 2.8.4)
project(cplex_example)

set(CMAKE_MODULE_PATH               "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS                 "${CMAKE_CXX_FLAGS} -std=c++14 -DIL_STD -m64 -Wall -lpthread")
set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS_DEBUG} -Og -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -O3 -flto")
set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX   "-isystem ")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "${CMAKE_CURRENT_SOURCE_DIR}/build")
set(LINKER_OPTIONS                  -flto -Wl,--no-as-needed)
set(BapCod_PATH "" CACHE PATH "Path to the Bapcod")

if(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set(CMAKE_CXX_FLAGS_DEBUG       "${CMAKE_CXX_FLAGS_DEBUG} -save-temps=obj")
endif()

option(BapCod "Enable the external project" OFF)

if(BapCod)
  #set(Boost_DEBUG ON)
  #set(BOOST_ROOT "BOOST_ROOT")
  find_package(LEMON)
  find_package(Boost 1.74 REQUIRED COMPONENTS program_options regex system timer chrono thread REQUIRED)
endif()


find_package(CPLEX)

include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})
include_directories(SYSTEM ${CPLEX_CONCERT_INCLUDE_DIR})
include_directories(SYSTEM ${CPLEX_CP_INCLUDE_DIR})


set(SOURCE_FILES
    src/main.cpp)

add_executable(Bender ${SOURCE_FILES})

find_package(CPLEX REQUIRED)

# Always link CPLEX, regardless of BapCod setting
target_link_libraries(Bender PRIVATE ${CPLEX_CP_LIBRARY} PRIVATE ${CPLEX_ILOCPLEX_LIBRARY} PRIVATE ${CPLEX_CONCERT_LIBRARY} PRIVATE ${CPLEX_LIBRARY})

target_link_libraries(Bender PRIVATE dl)
if(BapCod)
    add_definitions(-DUSE_BAP=true)

    find_package(BcpRcsp)
    set(USER_LIBRARIES ${BCP_RCSP_LIBRARY})
    set(USER_INCLUDE_DIR ${BCP_RCSP_INCLUDE_DIR})
    message("-- Boost_INCLUDE_DIR: ${Boost_INCLUDE_DIR}")
    message("-- Boost_LIBRARIES: ${Boost_LIBRARIES}")
    message("-- BapCodPath: ${BapCod_PATH}")
    message("-- BapCodInc: ${BapCod_PATH}/Bapcod/include")
    target_include_directories(Bender PRIVATE ${Boost_INCLUDE_DIR} PRIVATE ${PROJECT_SOURCE_DIR}/include PRIVATE ${BapCod_PATH}/Bapcod/include PRIVATE ${BapCod_PATH}/Bapcod/include_dev PRIVATE ${BapCod_PATH}/Tools/rcsp/include PRIVATE ${LEMON_INCLUDE_DIR})

    target_link_libraries(Bender
    PRIVATE dl
    PRIVATE ${PROJECT_SOURCE_DIR}/libs/libVRP_solver.a
    PRIVATE ${PROJECT_SOURCE_DIR}/libs/libbapcod_release.a
    PRIVATE ${Boost_LIBRARIES}
    PRIVATE ${LEMON_LIBRARY}
    PRIVATE ${PROJECT_SOURCE_DIR}/libs/librcsplinux.a

)
endif()
