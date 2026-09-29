#include "global.h"

/* DATA MANAGEMENT */
data_t            data_g            = {0};
bmp390_sample_t   bmp_sample_g      = {0};
gps_sample_t      gps_sample_g      = {0};
icm20948_sample_t icm_sample_g      = {0};
uint8_t           battery_voltage_g = 0;
file_counter_t    file_counter_g    = {0};
atomic_bool       lfs_full          = ATOMIC_VAR_INIT(false);

/* QUEUE HANDLE*/
QueueHandle_t xEventQueue    = NULL;
QueueHandle_t xSDQueue       = NULL;
QueueHandle_t xLittleFSQueue = NULL;
QueueHandle_t xLoraQueue     = NULL;
QueueHandle_t xB4LaunchQueue = NULL;

/* RING BUFFER */
RingbufHandle_t xSDRingBuf  = NULL;
RingbufHandle_t xLFSRingBuf = NULL;

/* SEMAPHORE */
SemaphoreHandle_t xI2CSem = NULL;

/* MUTEX */
portMUX_TYPE xDATAMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xBMPMutex  = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xGPSMutex  = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xICMMutex  = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE xADCMutex  = portMUX_INITIALIZER_UNLOCKED;

/* EVENT HANDLE */
EventGroupHandle_t xInitEventGroup       = NULL;
EventGroupHandle_t xNVSCounterEventGroup = NULL;
EventGroupHandle_t xFormatEventGroup     = NULL;
EventGroupHandle_t xCamEventGroup        = NULL;

/* TASK HANDLE */
TaskHandle_t xTaskLora    = NULL;
TaskHandle_t xTaskAcquire = NULL;

/* BUS HANDLE */
i2c_master_bus_handle_t bus_handle = NULL;