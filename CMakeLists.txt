cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
list(APPEND "EXTRA_COMPONENT_DIRS" "./components/lvgl_esp32_drivers" "./components/lvgl_esp32_drivers/lvgl_touch" "./components/lvgl_esp32_drivers/lvgl_tft" "./components/wtctrl" "./components/screen" "./components/bus" "./components/touch_panel")

project(LCD86BOX)
