# Se compilan los MISMOS drivers del firmware real para probarlos tal cual.
idf_component_register(
    SRCS "test_main.c"
         "../../main/motor.c"
         "../../main/encoder.c"
         "../../main/limit_switches.c"
    INCLUDE_DIRS "." "../../main")
