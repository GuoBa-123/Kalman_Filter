cmake_minimum_required(VERSION 3.15)

project(KalmanProject LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 包含头文件目录
include_directories(include)

# 添加可执行文件
add_executable(kalman_test
KalmanFilter.cc
)

# 编译优化（可选）
target_compile_options(kalman_test PRIVATE
    -Wall
    -Wextra
    -O3
)
