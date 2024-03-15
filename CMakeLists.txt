cmake_minimum_required (VERSION 3.8)
project(CodeCraftSDK)

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
ADD_EXECUTABLE(main ${src})