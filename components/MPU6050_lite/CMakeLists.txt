  
# Minimum CMakeList File For ESP32 Component

idf_component_register(SRCS 
                            "MPU6050_lite.c"
                        INCLUDE_DIRS 
                            "include"
                        REQUIRES
                        "I2Cdev" "log"
                        ) 