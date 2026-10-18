#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <driver/i2c_master.h>
#include <esp_err.h>

/* SC035HGS native mode (MIPI 1-lane, RAW10): the ISP outputs it directly as YUV420 for the H.264 encoder.
 * Must match CONFIG_CAMERA_SC035HGS_MIPI_DEFAULT_FMT_* in sdkconfig and the XCLK in capture.c. */
#define CAM_WIDTH  640
#define CAM_HEIGHT 480
#define CAM_FPS    48

#define CAM_H264_BITRATE (2 * 1000 * 1000) // ~250 KB/s on the SD card shared with the flight log
#define CAM_H264_GOP     CAM_FPS           // One IDR per second: bounds recovery after a lost frame
#define CAM_H264_QP_MIN  20
#define CAM_H264_QP_MAX  45

#define CAM_FRAME_MAX (64 * 1024) // Slot size: an IDR frame must always fit, or the stream cannot recover

#define CAM_PRETRIGGER_FRAMES (2 * CAM_FPS) // Kept before BOOST, so liftoff is in the video
#define CAM_QUEUE_FRAMES      (3 * CAM_FPS) // Backlog while the SD card stalls (~9 MB of PSRAM)

/* SD file: cam_file_hdr_t, then (cam_rec_hdr_t + H.264 Annex B access unit) per frame. Read by tools/cam_decode.py */
#define CAM_FILE_MAGIC   0x5649434Au // "JCIV"
#define CAM_REC_MAGIC    0x4D52464Au // "JFRM"
#define CAM_FILE_VERSION 2           // 2 = H.264 (1 was MJPEG)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t fps;
    uint32_t open_ms;
} cam_file_hdr_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint32_t capture_ms; // same clock as the flight log "time" field (ms since boot)
    uint32_t write_ms;
    uint32_t len;
    uint8_t  status;   // flight status bits when the frame was written
    uint8_t  keyframe; // 1 = IDR/I frame, decoding can start here
    uint8_t  reserved[2];
} cam_rec_hdr_t;

_Static_assert(sizeof(cam_file_hdr_t) == 12, "cam_file_hdr_t layout changed");
_Static_assert(sizeof(cam_rec_hdr_t) == 24, "cam_rec_hdr_t layout changed");

typedef struct {
    uint8_t *data;
    uint32_t size; // allocated bytes
    uint32_t len;
    uint32_t seq;
    uint32_t capture_ms;
    bool     keyframe;
} cam_frame_t;

/* Frame queue: capture task produces, recorder task consumes.
 * H.264 frames depend on the previous one, so frames are only ever dropped before encoding
 * or from the head of the standby ring (which is then advanced to the next keyframe). */
esp_err_t    frameq_init(void);
void         frameq_set_recording(bool recording);
cam_frame_t *frameq_acquire_write(void);
void         frameq_commit(uint32_t len, uint32_t seq, uint32_t capture_ms, bool keyframe);
cam_frame_t *frameq_peek(void);
void         frameq_pop(void);
void         frameq_release(void);
bool         frameq_wait(uint32_t timeout_ms);
uint32_t     frameq_dropped(void);

void      board_power_init(void); // Must run before the I2C bus is used: powers the camera and the I2C pull-ups
esp_err_t capture_start(i2c_master_bus_handle_t i2c_bus);
