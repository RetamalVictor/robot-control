cmake_minimum_required(VERSION 3.4)
project(pid_controller)

set(CMAKE_CXX_STANDARD 11)

set(pybind11_DIR "C:\\Users\\victo\\Desktop\\VU master\\thesis\\.thesis\\lib\\site-packages\\pybind11\\share\\cmake\\pybind11")

find_package(pybind11 REQUIRED)
pybind11_add_module(pid_controller pid.cpp)
