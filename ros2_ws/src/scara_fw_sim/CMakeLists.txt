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

# Pruebas unitarias del firmware (las mismas de firmware/scara_esp32/host_test): así
# `colcon test` también verifica la lógica del ESP32.
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  get_filename_component(FW_TEST "${FW_MAIN}/../host_test/test_ctrl_logic.c" ABSOLUTE)
  add_executable(test_ctrl_logic "${FW_TEST}" "${FW_MAIN}/ctrl_logic.c")
  target_include_directories(test_ctrl_logic PRIVATE "${FW_MAIN}")
  set_target_properties(test_ctrl_logic PROPERTIES C_STANDARD 11)
  target_compile_options(test_ctrl_logic PRIVATE -Wall -Wextra -Werror)
  ament_add_test(firmware_ctrl_logic
    COMMAND "$<TARGET_FILE:test_ctrl_logic>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()
ament_package()
