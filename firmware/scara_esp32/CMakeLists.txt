# Proyecto ESP-IDF del SCARA Estación 7 (firmware con micro-ROS).
# Compilar en una terminal SIN "source" de ROS 2 (lo exige micro_ros_espidf_component).
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(scara_esp32)
