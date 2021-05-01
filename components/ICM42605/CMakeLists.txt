  
# Minimum CMakeList File For ESP32 Component

idf_component_register(SRCS 
                            "ICM42605.c"
                        INCLUDE_DIRS 
                            "include"
                        REQUIRES
                        "I2Cdev" "log" "cubeOS"
                        ) 