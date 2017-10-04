project(extended-kalman-filters)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources ./main.cpp ./tools.cpp ./FusionEKF.cpp ./kalman_filter.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.14.1/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(extended-kalman-filters ${sources})

target_link_libraries(extended-kalman-filters z ssl uv uWS)