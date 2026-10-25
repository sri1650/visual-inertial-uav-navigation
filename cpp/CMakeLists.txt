cmake_minimum_required(VERSION 3.20)
project(vionav_core LANGUAGES CXX)

find_package(Eigen3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(vionav_core
  src/timestamp_sync.cpp
  src/calibration.cpp
  src/imu.cpp
  src/quality.cpp
  src/keyframes.cpp
  src/estimator_state.cpp
  src/estimator.cpp
)
target_include_directories(vionav_core PUBLIC include)
target_link_libraries(vionav_core PUBLIC Eigen3::Eigen)
target_compile_options(vionav_core PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()

add_executable(timestamp_sync_test tests/timestamp_sync_test.cpp)
target_link_libraries(timestamp_sync_test PRIVATE vionav_core)
add_test(NAME timestamp_sync_test COMMAND timestamp_sync_test)

add_executable(calibration_test tests/calibration_test.cpp)
target_link_libraries(calibration_test PRIVATE vionav_core)
add_test(NAME calibration_test COMMAND calibration_test)

add_executable(imu_test tests/imu_test.cpp)
target_link_libraries(imu_test PRIVATE vionav_core)
add_test(NAME imu_test COMMAND imu_test)

add_executable(estimator_state_test tests/estimator_state_test.cpp)
target_link_libraries(estimator_state_test PRIVATE vionav_core)
add_test(NAME estimator_state_test COMMAND estimator_state_test)

add_executable(quality_test tests/quality_test.cpp)
target_link_libraries(quality_test PRIVATE vionav_core)
add_test(NAME quality_test COMMAND quality_test)

add_executable(keyframes_test tests/keyframes_test.cpp)
target_link_libraries(keyframes_test PRIVATE vionav_core)
add_test(NAME keyframes_test COMMAND keyframes_test)

add_executable(estimator_test tests/estimator_test.cpp)
target_link_libraries(estimator_test PRIVATE vionav_core)
add_test(NAME estimator_test COMMAND estimator_test)
