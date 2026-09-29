idf_component_register(
    SRCS "frameq.c" "main.c" "capture.c" "link.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_spi esp_driver_gpio esp_driver_i2c esp_driver_jpeg esp_driver_ppa
             esp_mm esp_timer esp_rom cam_link
)
