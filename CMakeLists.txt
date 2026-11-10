cmake_minimum_required(VERSION 3.16)
project(dmar_simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(dmar_core
    src/environment.cpp
    src/clustering.cpp
    src/policies.cpp
    src/rollout.cpp
    src/scenarios.cpp
    src/simulator.cpp
    src/state.cpp
)
target_include_directories(dmar_core PUBLIC include)
target_compile_options(dmar_core PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(dmar_core_tests tests/test_core.cpp)
target_link_libraries(dmar_core_tests PRIVATE dmar_core)
add_test(NAME dmar_core_tests COMMAND dmar_core_tests)

add_executable(dmar_experiments src/experiments.cpp)
target_link_libraries(dmar_experiments PRIVATE dmar_core)

add_executable(dmar_benchmark src/benchmark.cpp)
target_link_libraries(dmar_benchmark PRIVATE dmar_core)