# Pruebas de la lógica del firmware en el PC (sin ESP32, sin ESP-IDF).
#   make test     compila con gcc (-Wall -Wextra -Werror) y corre las pruebas
#   make lib      librería compartida que usa el simulador fake_esp32
CC ?= gcc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror -Wconversion -Wno-sign-conversion
SRC_DIR := ../main

.PHONY: test lib clean
test: build/test_ctrl_logic
	./build/test_ctrl_logic

build/test_ctrl_logic: test_ctrl_logic.c $(SRC_DIR)/ctrl_logic.c $(SRC_DIR)/ctrl_logic.h $(SRC_DIR)/ctrl_config.h
	@mkdir -p build
	$(CC) $(CFLAGS) -I$(SRC_DIR) test_ctrl_logic.c $(SRC_DIR)/ctrl_logic.c -o $@

lib: build/libscara_ctrl_logic.so
build/libscara_ctrl_logic.so: $(SRC_DIR)/ctrl_logic.c $(SRC_DIR)/ctrl_logic.h $(SRC_DIR)/ctrl_config.h
	@mkdir -p build
	$(CC) $(CFLAGS) -fPIC -shared -I$(SRC_DIR) $(SRC_DIR)/ctrl_logic.c -o $@

clean:
	rm -rf build
