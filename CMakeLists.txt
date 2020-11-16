cmake_minimum_required(VERSION 3.5)

set(PROJECT_VER "1")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-CubeOS-V01)

spiffs_create_partition_image(storage filesystem FLASH_IN_PROJECT)
