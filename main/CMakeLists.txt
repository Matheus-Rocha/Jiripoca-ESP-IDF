cmake_minimum_required(VERSION 4.0)

list(APPEND SOURCES main.c global.c acquire.c system.c save.c send.c
                    sensors/adc.c sensors/bmp.c sensors/gps.c sensors/hx.c sensors/icm.c
                    camera/capture.c camera/frameq.c camera/recorder.c)

list(APPEND REQUIRES esp_adc esp_ringbuf esp_timer esp_driver_i2c esp_driver_uart
                        nvs_flash sdmmc fatfs esp_bmp390 esp_littlefs-master
                        icm20948-master esp-idf-sx126x minmea-master vqf-main eskf
                        esp_driver_spi esp_driver_gpio esp_driver_sdmmc
                        esp_driver_jpeg esp_driver_ppa esp_mm espressif__esp_video)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "." "camera"
    REQUIRES ${REQUIRES}
)
