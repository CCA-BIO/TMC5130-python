cmake_minimum_required(VERSION 3.16)
project(tmc5130_c LANGUAGES C)

add_library(tmc5130 STATIC src/tmc5130.c)
target_include_directories(tmc5130 PUBLIC include)
target_compile_features(tmc5130 PUBLIC c_std_99)
if(NOT MSVC)
  target_link_libraries(tmc5130 PUBLIC m)
endif()

enable_testing()

add_executable(tmc5130_c_tests tests/test_tmc5130.c)
target_link_libraries(tmc5130_c_tests PRIVATE tmc5130)
add_test(NAME tmc5130_c_tests COMMAND tmc5130_c_tests)
