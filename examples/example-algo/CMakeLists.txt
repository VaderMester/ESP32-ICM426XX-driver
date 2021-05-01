# Minimum CMakeList File For ESP32 Component

idf_component_register(SRCS 
                            "example-algo.c"
                            "main.c"
                        INCLUDE_DIRS 
                            "include"
                        REQUIRES
                        ) 