# Minimum CMakeList File For ESP32 Component

idf_component_register(SRCS 
                           
                            "main.c"
                        INCLUDE_DIRS 
                            ""
                        REQUIRES
                        "ICM426XX"
                        )