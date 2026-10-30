cmake_minimum_required(VERSION 3.15)
project(PicoTester CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(PicoTester WIN32 main.cpp)
target_compile_definitions(PicoTester PRIVATE UNICODE _UNICODE)
target_link_libraries(PicoTester PRIVATE setupapi comctl32)
if(MSVC)
  target_compile_options(PicoTester PRIVATE /W4 /utf-8)
endif()
