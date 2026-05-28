cmake_minimum_required(VERSION 3.16)
project(tmc5130_cpp LANGUAGES CXX)

add_library(tmc5130_cpp STATIC src/tmc5130.cpp)
target_include_directories(tmc5130_cpp PUBLIC include)
target_compile_features(tmc5130_cpp PUBLIC cxx_std_17)

enable_testing()

add_executable(tmc5130_cpp_tests tests/test_tmc5130.cpp)
target_link_libraries(tmc5130_cpp_tests PRIVATE tmc5130_cpp)
add_test(NAME tmc5130_cpp_tests COMMAND tmc5130_cpp_tests)
