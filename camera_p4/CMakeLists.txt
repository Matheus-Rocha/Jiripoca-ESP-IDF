cmake_minimum_required(VERSION 3.16)

# Shares the SPI link protocol with the Jiripoca S3 firmware
set(EXTRA_COMPONENT_DIRS ../components/cam_link)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(jiripoca_cam_p4)
