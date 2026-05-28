cmake_minimum_required(VERSION 3.16)
project(tmc5130_cpp LANGUAGES CXX)

add_library(tmc5130_cpp STATIC src/tmc5130.cpp)
target_include_directories(tmc5130_cpp PUBLIC include)
target_compile_features(tmc5130_cpp PUBLIC cxx_std_17)
