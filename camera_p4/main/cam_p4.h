#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "cam_protocol.h"

/* Output video: centre crop of the 1920x1080 sensor frame scaled by exactly 0.5 (the PPA
 * scaler has 1/16 precision, so crop and output must keep an exact ratio). */
#define CAM_OUT_WIDTH   640
#define CAM_OUT_HEIGHT  480
#define CAM_CROP_WIDTH  1280
#define CAM_CROP_HEIGHT 960

#define CAM_JPEG_TARGET_BYTES (20 * 1024) // ~500 KB/s at 25 fps, leaves margin on the link and the S3 SD
#define CAM_JPEG_QUALITY_INIT 60
#define CAM_JPEG_QUALITY_MIN  20
#define CAM_JPEG_QUALITY_MAX  85

#define CAM_PRETRIGGER_FRAMES 50  // ~2 s kept while in standby, so liftoff is in the video
#define CAM_QUEUE_FRAMES      125 // ~5 s of backlog while the S3 catches up (PSRAM)

typedef struct {
    uint8_t *data;
    uint32_t size; // allocated bytes
    uint32_t len;
    uint32_t seq;
    uint32_t capture_ms; // P4 clock
} cam_frame_t;

/* Frame queue: capture task produces, link task consumes */
esp_err_t    frameq_init(void);
void         frameq_set_recording(bool recording);
cam_frame_t *frameq_acquire_write(void);
void         frameq_commit(uint32_t len, uint32_t seq, uint32_t capture_ms);
cam_frame_t *frameq_peek(void);
void         frameq_pop(void);
void         frameq_release(void);
bool         frameq_wait(uint32_t timeout_ms);
uint32_t     frameq_dropped(void);

esp_err_t capture_start(void);
esp_err_t link_start(void);
