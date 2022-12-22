# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(COMPONENTS esp32 esp_psram esp_rom freertos log main esp_timer lvgl LovyanGFX)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# LVGL custom config file setup
add_compile_definitions( LGFX_USE_V1 )  # use V1 instead of V0
add_compile_definitions( LGFX_WT32_SC01 )  # WT32-SC01 autodetect

add_compile_definitions( LV_CONF_INCLUDE_SIMPLE=1 )  # allow custom lv_conf.h path
idf_build_set_property(COMPILE_OPTIONS "-Imain" APPEND)  # directory with lv_conf.h

project(lvgl-fps)
