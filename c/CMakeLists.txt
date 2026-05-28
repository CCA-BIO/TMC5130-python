cmake_minimum_required(VERSION 3.16)
project(tmc5130_c LANGUAGES C)

add_library(tmc5130 STATIC src/tmc5130.c)
target_include_directories(tmc5130 PUBLIC include)
target_compile_features(tmc5130 PUBLIC c_std_99)
