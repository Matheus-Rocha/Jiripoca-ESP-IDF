#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <driver/i2c_master.h>
#include <esp_err.h>

/* Output video: centre crop of the 1920x1080 sensor frame scaled by exactly 0.5 (the PPA
 * scaler has 1/16 precision, so crop and output must keep an exact ratio). */
#define CAM_OUT_WIDTH   640
#define CAM_OUT_HEIGHT  480
#define CAM_CROP_WIDTH  1280
#define CAM_CROP_HEIGHT 960

#define CAM_FRAME_MAX         (48 * 1024) // Largest accepted JPEG
#define CAM_JPEG_TARGET_BYTES (20 * 1024) // ~500 KB/s at 25 fps on the shared SD card
#define CAM_JPEG_QUALITY_INIT 60
#define CAM_JPEG_QUALITY_MIN  20
#define CAM_JPEG_QUALITY_MAX  85

#define CAM_PRETRIGGER_FRAMES 50  // ~2 s kept before BOOST, so liftoff is in the video
#define CAM_QUEUE_FRAMES      125 // ~5 s of backlog when the SD card stalls (PSRAM)

/* SD file: cam_file_hdr_t, then (cam_rec_hdr_t + JPEG bytes) per frame. Read by tools/cam_decode.py */
#define CAM_FILE_MAGIC   0x5649434Au // "JCIV"
#define CAM_REC_MAGIC    0x4D52464Au // "JFRM"
#define CAM_FILE_VERSION 1

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t open_ms;
} cam_file_hdr_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint32_t capture_ms; // same clock as the flight log "time" field (ms since boot)
    uint32_t write_ms;
    uint32_t len;
    uint8_t  status; // flight status bits when the frame was written
    uint8_t  reserved[3];
} cam_rec_hdr_t;

_Static_assert(sizeof(cam_file_hdr_t) == 12, "cam_file_hdr_t layout changed");
_Static_assert(sizeof(cam_rec_hdr_t) == 24, "cam_rec_hdr_t layout changed");

typedef struct {
    uint8_t *data;
    uint32_t size; // allocated bytes
    uint32_t len;
    uint32_t seq;
    uint32_t capture_ms;
} cam_frame_t;

/* Frame queue: capture task produces, recorder task consumes */
esp_err_t    frameq_init(void);
void         frameq_set_recording(bool recording);
cam_frame_t *frameq_acquire_write(void);
void         frameq_commit(uint32_t len, uint32_t seq, uint32_t capture_ms);
cam_frame_t *frameq_peek(void);
void         frameq_pop(void);
void         frameq_release(void);
bool         frameq_wait(uint32_t timeout_ms);
uint32_t     frameq_dropped(void);

void      board_power_init(void); // Must run before the I2C bus is used: powers the camera and the I2C pull-ups
esp_err_t capture_start(i2c_master_bus_handle_t i2c_bus);
