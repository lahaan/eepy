# ESP32-IDF component: only top-level src/*.c (excludes src/pico/ Arduino code,
# which is built by the pico2w env via build_src_filter).
FILE(GLOB app_sources ${CMAKE_SOURCE_DIR}/src/*.c ${CMAKE_SOURCE_DIR}/src/*.cpp)

idf_component_register(SRCS ${app_sources}
    PRIV_REQUIRES nvs_flash esp_wifi esp_event esp_netif driver)
