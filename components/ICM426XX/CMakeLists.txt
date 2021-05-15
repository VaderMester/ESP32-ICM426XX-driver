
idf_component_register(SRCS
                        "ICM426XX.c"
                        "Invn/EmbUtils/DataConverter.c"
                        "Invn/EmbUtils/ErrorHelper.c"
                        "Invn/EmbUtils/InvBasicMath.c"
                        "Invn/EmbUtils/Message.c"
                        "Invn/Drivers/Icm426xx/Icm426xxTransport.c"
                        "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.c"
                        "Invn/Drivers/Icm426xx/Icm426xxSelfTest.c"
                        "Invn/Drivers/Icm426xx/Icm426xxDriver_HL_apex.c"
                        "Invn/Drivers/Ak0991x/Ak0991x.c"
                        "Invn/Helpers/Icm426xx/helperClockCalib.c"
                        INCLUDE_DIRS
                        "include"
                        "Invn/"
                        "Invn/Drivers/Icm426xx"
                        "Invn/Drivers/Ak0991x"
                        "Invn/EmbUtils"
                        "Invn/LibAGM"
                        "Invn/Helpers"
                        "Invn/Helpers/Icm426xx"
                        REQUIRES
                        "I2Cdev" "log" "nvs_flash"
                        )
# Import the library, specifying a target name and the library path.
# The private dependencies of the library is also specified.
# find_library(INVNALGOAGM InvnAlgoAGM "lib/")
# add_prebuilt_library(prebuilt ${INVNALGOAGM})

# target_link_libraries(${COMPONENT_LIB} PRIVATE prebuilt)