cmake_minimum_required(VERSION 2.8.4)
project(cplex_example)

set(CMAKE_MODULE_PATH               "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS                 "${CMAKE_CXX_FLAGS} -std=c++14 -DIL_STD -m64 -Wall -Werror")
set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS_DEBUG} -Og -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -O3 -flto")
set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX   "-isystem ")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "${CMAKE_CURRENT_SOURCE_DIR}/build")
set(LINKER_OPTIONS                  -flto -Wl,--no-as-needed)

if(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set(CMAKE_CXX_FLAGS_DEBUG       "${CMAKE_CXX_FLAGS_DEBUG} -save-temps=obj")
endif()

find_package(CPLEX)

include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})
include_directories(SYSTEM ${CPLEX_CONCERT_INCLUDE_DIR})

set(SOURCE_FILES
    src/main.cpp)

add_executable(         Bender ${SOURCE_FILES})
target_link_libraries(  Bender PRIVATE ${CPLEX_LIBRARIES})
target_link_libraries(  Bender PRIVATE dl)