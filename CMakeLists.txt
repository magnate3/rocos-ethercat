cmake_minimum_required(VERSION 3.10)
project(rocos_ecm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS date_time filesystem)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  include/${PROJECT_NAME}
  include/sdk
)

link_directories(
  lib
)

file(GLOB_RECURSE SRCS src/*.cpp)
file(GLOB_RECURSE HDRS include/*.h include/*.hpp)
file(GLOB_RECURSE ENI_FILE config/*.xml)
file(GLOB_RECURSE CONFIG_FILE config/*.yaml)


add_executable(${PROJECT_NAME} ${SRCS})

set(DEP_LIBS
        AtemRasSrv
        EcMaster
        emllI8254x
        dl
        rt
        Threads::Threads
        Boost::boost
        Boost::date_time
        Boost::filesystem
        yaml-cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        PRIVATE
        ${DEP_LIBS}
)

add_subdirectory(lib/LinkOsLayer/atemsys)

file(COPY lib/libemllI8254x.so DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${ENI_FILE} ${CONFIG_FILE} DESTINATION ${CMAKE_BINARY_DIR}/bin)

configure_file(scripts/initECM.sh.in  ${CMAKE_BINARY_DIR}/bin/initECM.sh)
configure_file(scripts/runECM.sh.in   ${CMAKE_BINARY_DIR}/bin/runECM.sh)

#############
## Install ##
#############



#############
## Testing ##
#############

enable_testing()
add_executable(unit_test test/unit_test.cpp)
target_link_libraries(unit_test
        PRIVATE
        ${DEP_LIBS}
        )
add_test(NAME unit_test COMMAND unit_test)
