# ESPAsyncButton - Event-based asynchronous button library for the ESP32 arduino/IDF
# https://github.com/vortigont/ESPAsyncButton

cmake_minimum_required(VERSION 3.5)
idf_build_get_property(target IDF_TARGET)

FILE(GLOB_RECURSE app_sources "src/*.cpp")

# Build ESPAsyncButton as an ESP-IDF component
if(ESP_PLATFORM)
    idf_component_register(
        SRCS ${app_sources}
        INCLUDE_DIRS "./src"
        #REQUIRES ${depends}
        #PRIV_REQUIRES
    )
endif()

project(ESPAsyncButton VERSION 1.0.0)
