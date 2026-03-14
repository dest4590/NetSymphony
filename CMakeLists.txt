cmake_minimum_required(VERSION 3.20)
project(NetSymphony LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(NetSymphony
        src/main.cpp
)

if (WIN32)
    target_link_libraries(NetSymphony PRIVATE ws2_32)
endif ()