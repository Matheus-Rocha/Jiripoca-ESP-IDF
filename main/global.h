#pragma once

#include "header.h"

/* DATA MANAGEMENT*/
extern data_t            data_g;
extern icm20948_sample_t icm_sample_g;
extern gps_sample_t      gps_sample_g;
extern bmp390_sample_t   bmp_sample_g;
extern uint8_t           battery_voltage_g;
extern file_counter_t    file_counter_g;
extern atomic_bool       lfs_full; // Flag to indicate if LittleFS is full

/* QUEUE HANDLE*/
extern QueueHandle_t xEventQueue;
extern QueueHandle_t xSDQueue;
extern QueueHandle_t xLittleFSQueue;
extern QueueHandle_t xLoraQueue;
extern QueueHandle_t xB4LaunchQueue; // Queue to send data before launch phase

/* RING BUFFER */
extern RingbufHandle_t xSDRingBuf;
extern RingbufHandle_t xLFSRingBuf;

/* SEMAPHORE */
extern SemaphoreHandle_t xI2CSem;

/* MUTEX */
extern portMUX_TYPE xDATAMutex;
extern portMUX_TYPE xBMPMutex;
extern portMUX_TYPE xGPSMutex;
extern portMUX_TYPE xICMMutex;
extern portMUX_TYPE xADCMutex;

/* TASK HANDLE */
extern TaskHandle_t xTaskLora;
extern TaskHandle_t xTaskAcquire;

/* EVENT HANDLE */
extern EventGroupHandle_t xInitEventGroup;
extern EventGroupHandle_t xNVSCounterEventGroup; // NVS counter synchronization
extern EventGroupHandle_t xFormatEventGroup;     // LittleFS and SD format synchronization
extern EventGroupHandle_t xCamEventGroup;        // SD availability and camera shutdown synchronization

/* BUS HANDLE*/
extern i2c_master_bus_handle_t bus_handle;

/* TASKS */
void task_setup(void *pvParameters);
void task_fusion(void *pvParameters);
void task_bmp(void *pvParameters);
void task_gps(void *pvParameters);
void task_acquire(void *pvParameters);
void task_adc(void *pvParameters);
void task_log(void *pvParameters);
void task_sd(void *pvParameters);
void task_lfs(void *pvParameters);
void task_nvs(void *pvParameters);
void task_lora(void *pvParameters);
void task_buzzer_led(void *pvParameters);
void task_camera(void *pvParameters);
