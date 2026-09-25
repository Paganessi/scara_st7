# scara_fw_sim — compila la lógica PURA del firmware (firmware/scara_esp32/main/ctrl_logic.c)
# como librería compartida para que el simulador fake_esp32 ejecute EXACTAMENTE el mismo
# código que corre en el ESP32 (PID, enclavamiento, watchdog, bloqueo, debounce).
# No hay copia: se compila el archivo del firmware directamente desde el repo.
cmake_minimum_required(VERSION 3.8)
project(scara_fw_sim C)
find_package(ament_cmake REQUIRED)

get_filename_component(FW_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/../../../firmware/scara_esp32/main" ABSOLUTE)
if(NOT EXISTS "${FW_MAIN}/ctrl_logic.c")
  message(FATAL_ERROR "No encuentro ${FW_MAIN}/ctrl_logic.c (¿se movió el firmware?)")
endif()

add_library(scara_ctrl_logic SHARED "${FW_MAIN}/ctrl_logic.c")
target_include_directories(scara_ctrl_logic PRIVATE "${FW_MAIN}")
set_target_properties(scara_ctrl_logic PROPERTIES C_STANDARD 11)
target_compile_options(scara_ctrl_logic PRIVATE -Wall -Wextra -Werror)

install(TARGETS scara_ctrl_logic LIBRARY DESTINATION lib)
ament_package()
