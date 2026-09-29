#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * Shared by the Jiripoca ESP32-S3 (SPI master) and the ESP32-P4-EYE (SPI slave).
 * Every transaction is exactly CAM_LINK_CHUNK_SIZE bytes, full duplex:
 *   MOSI (S3 -> P4): cam_cmd_t at offset 0, rest zero.
 *   MISO (P4 -> S3): cam_chunk_hdr_t followed by up to CAM_LINK_PAYLOAD_MAX bytes of JPEG.
 * The P4 raises the handshake line once a transaction is loaded and lowers it when it ends,
 * so the S3 never clocks a slave that is not ready.
 */

#define CAM_LINK_CHUNK_SIZE 4096
#define CAM_LINK_FRAME_MAX  (32 * 1024) // S3 has no PSRAM: one frame must fit in internal RAM

#define CAM_LINK_MAGIC_CMD  0x444D434Au // "JCMD"
#define CAM_LINK_MAGIC_DATA 0x4D41434Au // "JCAM"

typedef enum {
    CAM_CMD_STANDBY = 0, // P4 keeps a pre-trigger ring buffer, sends nothing
    CAM_CMD_RECORD  = 1, // P4 sends the ring buffer, then live frames
} cam_cmd_code_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t s3_time_ms; // S3 esp_timer ms, lets the P4 stamp frames in the flight log clock
    uint8_t  cmd;        // cam_cmd_code_t
    uint8_t  reserved[3];
    uint32_t crc32; // esp_rom_crc32_le over the bytes before this field
} cam_cmd_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t frame_seq;
    uint32_t frame_len; // 0 = idle chunk, no frame data
    uint32_t offset;    // position of this payload inside the frame
    uint32_t capture_ms; // capture time in the S3 clock (0 = not synchronized yet)
    uint32_t dropped;    // frames dropped by the P4 since boot
    uint16_t payload_len;
    uint16_t reserved;
    uint32_t crc32; // esp_rom_crc32_le over the header bytes before this field, then the payload
} cam_chunk_hdr_t;

#define CAM_LINK_PAYLOAD_MAX (CAM_LINK_CHUNK_SIZE - sizeof(cam_chunk_hdr_t))

_Static_assert(sizeof(cam_cmd_t) == 16, "cam_cmd_t layout changed");
_Static_assert(sizeof(cam_chunk_hdr_t) == 32, "cam_chunk_hdr_t layout changed");

/* SD file written by the S3: cam_file_hdr_t, then (cam_rec_hdr_t + JPEG bytes) per frame */
#define CAM_FILE_MAGIC   0x5649434Au // "JCIV"
#define CAM_REC_MAGIC    0x4D52464Au // "JFRM"
#define CAM_FILE_VERSION 1

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t s3_open_ms;
} cam_file_hdr_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint32_t capture_ms; // same clock as the flight log "time" field (ms since S3 boot)
    uint32_t write_ms;
    uint32_t len;
    uint8_t  status; // flight status bits when the frame was written
    uint8_t  reserved[3];
} cam_rec_hdr_t;

_Static_assert(sizeof(cam_file_hdr_t) == 12, "cam_file_hdr_t layout changed");
_Static_assert(sizeof(cam_rec_hdr_t) == 24, "cam_rec_hdr_t layout changed");
