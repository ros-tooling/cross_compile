cmake_minimum_required(VERSION 3.5)
project(dummy_pkg)
add_executable(dummy_binary main.cpp)
install(TARGETS dummy_binary DESTINATION bin)
