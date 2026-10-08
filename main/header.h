#pragma once

#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <esp_check.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>

#include "freertos/ringbuf.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/sdmmc_host.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <driver/uart.h>

#include <esp_littlefs.h>
#include <esp_vfs_fat.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <sdmmc_cmd.h>
#include <sys/stat.h>

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>

#include "bmp390.h"
#include "eskf.h"
#include "icm20948.h"
#include "icm20948_i2c.h"
#include "minmea.h"
#include "ra01s.h"
#include "vqf_c.h"

#define HIGH 1
#define LOW  0

/*
 * PINOUT (ESP32-P4)
 * PROVISIONAL: mapped onto the ESP32-P4-EYE devkit so the firmware runs on the bench without touching
 * the camera, SD card, USB or console pins. Pins marked J22 are reachable on the devkit header; the
 * others reuse devkit features we don't use (LCD, knob) and are NOT physically accessible.
 * Replace with the final board pinout.
 */

/* Flight I/O */
#define DROGUE_GPIO GPIO_NUM_18 // provisional (devkit LCD CS)
#define MAIN_GPIO   GPIO_NUM_19 // provisional (devkit LCD DC)
#define RBF_GPIO    GPIO_NUM_47 // provisional (devkit knob B)
#define BUTTON_GPIO GPIO_NUM_35 // devkit BOOT button, used for format mode
#define LED_GPIO    GPIO_NUM_15 // provisional (devkit LCD RST)
#define BUZZER_GPIO GPIO_NUM_16 // provisional (devkit LCD MOSI)

#define VIN_ADC_UNIT    ADC_UNIT_1
#define VIN_ADC_CHANNEL ADC_CHANNEL_1 // ADC1 CH1 = GPIO17, provisional (devkit LCD CLK)

/* I2C: BMP390 and ICM20948 share the camera bus (devkit I2C, also on J22 pins 10/12) */
#define I2C_SDA_IO GPIO_NUM_14
#define I2C_SCL_IO GPIO_NUM_13

/* GPS */
#define GPS_UART_NUM UART_NUM_1
#define GPS_RX       GPIO_NUM_6  // J22 pin 9
#define GPS_TX       GPIO_NUM_10 // J22 pin 5

/* LoRa SX126x on SPI2 */
#define SPI_HOST   SPI2_HOST
#define DMA_CHAN   SPI_DMA_CH_AUTO
#define SPI2_CLK   GPIO_NUM_52 // J22 pin 16
#define SPI2_MOSI  GPIO_NUM_53 // J22 pin 15
#define SPI2_MISO  GPIO_NUM_51 // J22 pin 17
#define LORA_CS    GPIO_NUM_50 // J22 pin 18
#define LORA_RESET GPIO_NUM_54 // J22 pin 13
#define LORA_BUSY  GPIO_NUM_7  // J22 pin 8
#define LORA_DIO1  GPIO_NUM_8  // J22 pin 7

/* SD card: SDMMC slot 0 uses fixed IO MUX pins (GPIO39-44), IO powered by on-chip LDO 4 */
#define SD_SLOT     SDMMC_HOST_SLOT_0
#define SD_LDO_CHAN 4
#define SD_PWR_EN_N GPIO_NUM_46 // P-MOS switch, low = card powered

#define I2C_SPEED 400000 // 400kHz, fast mode

#define GPS_BAUDRATE 9600

#define GPS_RX_CHUNK  128
#define GPS_BUFF_SIZE 4096

#define BMP390_I2C_ADDRESS   (0x77)
#define ICM20948_I2C_ADDRESS (0x68)

#define ICM_SAMPLE_RATE_S  0.01f // 100Hz
#define BMP_SAMPLE_RATE_MS 50    // 20Hz
#define GPS_SAMPLE_RATE_MS 200   // 5Hz, M8N GPS & GLONASS

// AUMENTAR OS BITS DO status e mudar para estados do fogute melhor e tals
/*#define TASK_INIT       BIT(0)
#define SETUP_OK        BIT(1)
#define FATAL_ERROR     BIT(2)
#define ARMED           BIT(3)
#define BOOST           BIT(4)
#define COAST           BIT(5)
#define DROGUE_DEPLOYED BIT(6)
#define MAIN_DEPLOYED   BIT(7)
#define LANDING         BIT(8)
#define LANDED          BIT(9)
*/

/* STATUS FLAGS */
#define INITIALIZED     BIT(0)
#define ARMED           BIT(1)
#define BOOST           BIT(2)
#define COAST           BIT(3)
#define DROGUE_DEPLOYED BIT(4)
#define MAIN_DEPLOYED   BIT(5)
#define LANDING         BIT(6)
#define LANDED          BIT(7)
#define NVS_EDIT        BIT(10)

/* INIT FLAGS */
// separar estados do foguete e estados de inicialização dos componentes e afins
#define BMP_INIT  BIT(0)
#define MS_INIT   BIT(1)
#define LSM_INIT  BIT(2)
#define BMM_INIT  BIT(3)
#define HX_INIT   BIT(4)
#define SD_INIT   BIT(5)
#define LFS_INIT  BIT(6)
#define LORA_INIT BIT(7)
#define GPS_INIT  BIT(8)

#define SETUP_INIT (BMP_INIT | MS_INIT | LSM_INIT | BMM_INIT | HX_INIT | SD_INIT | LFS_INIT | LORA_INIT | GPS_INIT)

/* SAVE FLAGS */
#define EVT_SD_DONE  BIT(0)
#define EVT_LFS_DONE BIT(1)
#define EVT_NVS_DONE BIT(2)

/* CAMERA FLAGS */
#define CAM_EVT_SD_READY  BIT(0)
#define CAM_EVT_SD_FAILED BIT(1)
#define CAM_EVT_DONE      BIT(2)
#define CAM_EVT_BUS_READY BIT(3)

#define THRESHOLD_MS 150 // Time threshold for state transitions in ms (e.g. boost to coast, deploy drogue, etc.)
#define PREPARE_FOR_LANDING_S                                                                                          \
    5.0f // Time to landing to set LANDING flag in seconds (e.g. if time to landing is less than 5s)
#define BOOST_THRESHOLD_A  2 * G // Vertical acceleration threshold to detect boost phase in m/s²
#define COAST_THRESHOLD_A  2 * G // Vertical acceleration threshold to enter coast phase in m/s²
#define DROGUE_THRESHOLD_V 0.5f  // Vertical velocity threshold to deploy drogue in m/s
#define MAIN_ALTITUDE      450   // Altitude above initial to deploy main in meters

#define KNOWN_ALTITUDE    715 // m, launch zone altitude
#define KNOWN_TEMPERATURE 20  // °C, temperature at launch zone

#define V_DIV_RATIO 0.015 // Voltage divider ratio {[(10k + 20k) / 20k] * 0.1}

// Scaling factors for sensors
#define GYRO_SCALE  1.0f / 65.5f   // ±500dps
#define ACC_SCALE   1.0f / 2048.0f // ±16g
#define MAG_SCALE   0.15f
#define TEMP_SCALE  1.0f / 333.87f
#define TEMP_OFFSET 21.0f // °C

#define G 9.80665f // m/s²

enum SENSOR_BIT {
    ICM_BIT,
    BMP_BIT,
    GPS_BIT,
    ADC_BIT,
};

// ICM20948 sample buffer
typedef struct {
    int16_t accel_x, accel_y, accel_z; // @SAVE (LSB)
    int16_t gyro_x, gyro_y, gyro_z;    // @SAVE (LSB)
    int16_t mag_x, mag_y, mag_z;       // @SAVE (LSB)
    float   q1, q2, q3, q4;            // @SEND (quaternions)
    float   az;                        // @INTERNAL (Vertical acceleration in m/s²)
    uint8_t accel;                     // @SEND (|g| * 10 with gravity removed, e.g. 159 for 15.9g)
} icm20948_sample_t;

// GPS sample buffer
typedef struct {
    float    latitude;     // @SAVE + SEND (°)
    float    longitude;    // @SAVE + SEND (°)
    float    altitude;     // @SAVE + SEND (m above known altitude)
    float    vel_vertical; // @SAVE + SEND (m/s)
    uint32_t utc_time;     // @INTERNAL (HHMMSS)
    uint8_t  sAcc;         // @SAVE (Speed accuracy Estimate * 10 in m/s)
    uint8_t
        fix; // @SAVE + SEND (GGA fix quality: 0=No Fix, 1=Standard GPS (2D/3D), 2=Differential GPS, 6=Estimated (DR))
} gps_sample_t;

// BMP390 sample buffer
typedef struct {
    float altitude; // @SEND (m above known altitude)
    float pressure; // @SAVE (Pa)
} bmp390_sample_t;

/*
 * @brief Data structure to store sensor data.
 * @note Fields are grouped and ordered by size for memory efficiency.
 */
typedef struct {
    uint32_t time; // @SAVE + SEND (Milliseconds since boot, or GPS time in HHMMSS)

    icm20948_sample_t icm;
    gps_sample_t      gps;
    bmp390_sample_t   bmp;
    eskf_t            kf;

    uint8_t status;  // @SAVE + SEND (Bitfields)
    uint8_t voltage; // @SAVE + SEND (V * 10, e.g. 33 for 3.3V)
} data_t;

/*
 * @brief Data structure for saving to SD card and LittleFS.
 * @note Packed to avoid padding bytes.
 */
typedef struct __attribute__((packed)) {
    uint32_t time;
    float    pressure;
    float    latitude, longitude;
    float    gps_altitude, gps_vel_vertical;
    int16_t  accel_x, accel_y, accel_z;
    int16_t  gyro_x, gyro_y, gyro_z;
    int16_t  mag_x, mag_y, mag_z;
    uint8_t  status;
    uint8_t  voltage; // V * 10
    uint8_t  sAcc;    // SAcc * 10
    uint8_t  fix;
} save_t;

/*
 * @brief Data structure for sending to ground station.
 * @note Packed to avoid padding bytes.
 */
typedef struct __attribute__((packed)) {
    uint32_t time;
    float    latitude, longitude;
    uint16_t kf_altitude, kf_apogee; // m * 10 (limited to 6553.5 m)
    int16_t  kf_vel_vertical;        // m/s * 10
    int16_t  q1, q2, q3, q4;         // quaternions * 10 000
    uint8_t  accel;                  // |g| * 10
    uint8_t  status;
    uint8_t  voltage; // V * 10
    uint8_t  fix;
} send_t;

/* SYSTEM STRUCTURES */
typedef struct __attribute__((packed)) {
    uint32_t sd_files;
    uint32_t lfs_files;
    bool     format;
} file_counter_t;

typedef struct __attribute__((packed)) {
    uint32_t name_check; // 4 Bytes
    uint32_t timestamp;  // 4 Bytes
} file_header_t;         // 8 bytes

/* EVENT STRUCTURES */
typedef enum __attribute__((packed)) {
    EVT_INIT_READY,   // task_status finished peripheral setup
    EVT_SETUP_OK,     // system tasks initialized correctly
    EVT_SETUP_FAILED, // system initialization failed
    EVT_ARM,          // system armed
} status_event_t;