# Minimum CMakeList File For ESP32 Component

idf_component_register(SRCS 
                            "I2Cdev.c" 
                        INCLUDE_DIRS 
                            "include"
                        REQUIRES
                        ) 