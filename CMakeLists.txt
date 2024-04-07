cmake_minimum_required(VERSION 3.8)
# SET(CMAKE_C_COMPILER "/nix/store/f3fm03pjva88ymwzy9m6x9k4xjpk6k5m-gcc-7.5.0/bin/gcc")
# SET(CMAKE_CXX_COMPILER "/nix/store/f3fm03pjva88ymwzy9m6x9k4xjpk6k5m-gcc-7.5.0/bin/g++")
project(CodeCraftSDK)


# The additional / is important to remove the last character from the path.
# Note that it does not matter if the OS uses / or \, because we are only
# saving the path size.
# string(LENGTH "${CMAKE_SOURCE_DIR}/" SOURCE_PATH_SIZE)
# add_definitions("-DSOURCE_PATH_SIZE=${SOURCE_PATH_SIZE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
#set(CMAKE_BUILD_TYPE Debug)

if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)

AUX_SOURCE_DIRECTORY(. src)
ADD_EXECUTABLE(main ${src}
        tree_map.hpp)