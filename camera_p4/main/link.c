#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"

#include "cam_p4.h"

static const char *TAG = "link";

/* J22 expansion header, wired to the Jiripoca S3 (see main/header.h on the S3 side) */
#define LINK_HOST      SPI2_HOST
#define LINK_SCLK      GPIO_NUM_52 // J22 pin 16
#define LINK_MOSI      GPIO_NUM_53 // J22 pin 15
#define LINK_MISO      GPIO_NUM_51 // J22 pin 17
#define LINK_CS        GPIO_NUM_50 // J22 pin 18
#define LINK_HANDSHAKE GPIO_NUM_54 // J22 pin 13

#define LINK_PRIORITY      6
#define LINK_STACK         4096
#define LINK_FRAME_WAIT_MS 20 // In RECORD with nothing queued, wait a bit before sending an idle chunk

static uint8_t *tx_buf;
static uint8_t *rx_buf;

static void IRAM_ATTR link_post_setup(spi_slave_transaction_t *t) {
    gpio_set_level(LINK_HANDSHAKE, 1);
}

static void IRAM_ATTR link_post_trans(spi_slave_transaction_t *t) {
    gpio_set_level(LINK_HANDSHAKE, 0);
}

static bool link_parse_cmd(const cam_cmd_t *c) {
    return c->magic == CAM_LINK_MAGIC_CMD &&
           c->crc32 == esp_rom_crc32_le(0, (const uint8_t *)c, offsetof(cam_cmd_t, crc32));
}

static uint16_t link_fill_chunk(const cam_frame_t *frame, uint32_t offset, bool synced, int32_t s3_offset_ms) {
    cam_chunk_hdr_t *h = (cam_chunk_hdr_t *)tx_buf;
    memset(h, 0, sizeof(*h));
    h->magic   = CAM_LINK_MAGIC_DATA;
    h->dropped = frameq_dropped();

    uint16_t payload = 0;
    if (frame) {
        payload = frame->len - offset > CAM_LINK_PAYLOAD_MAX ? CAM_LINK_PAYLOAD_MAX : frame->len - offset;
        memcpy(tx_buf + sizeof(*h), frame->data + offset, payload);
        h->frame_seq  = frame->seq;
        h->frame_len  = frame->len;
        h->offset     = offset;
        h->capture_ms = synced ? (uint32_t)((int32_t)frame->capture_ms + s3_offset_ms) : 0;
    }
    h->payload_len = payload;

    uint32_t crc = esp_rom_crc32_le(0, tx_buf, offsetof(cam_chunk_hdr_t, crc32));
    h->crc32     = esp_rom_crc32_le(crc, tx_buf + sizeof(*h), payload);
    return payload;
}

static void link_task(void *arg) {
    cam_cmd_code_t mode      = CAM_CMD_STANDBY;
    cam_frame_t   *frame     = NULL;
    uint32_t       offset    = 0;
    bool           synced    = false;
    int32_t        s3_offset = 0;

    while (true) {
        if (mode == CAM_CMD_RECORD && !frame) {
            frame = frameq_peek();
            if (!frame && frameq_wait(LINK_FRAME_WAIT_MS))
                frame = frameq_peek();
            offset = 0;
        }

        uint16_t payload = link_fill_chunk(frame, offset, synced, s3_offset);

        spi_slave_transaction_t t = {
            .length    = CAM_LINK_CHUNK_SIZE * 8,
            .tx_buffer = tx_buf,
            .rx_buffer = rx_buf,
        };
        if (spi_slave_transmit(LINK_HOST, &t, portMAX_DELAY) != ESP_OK)
            continue;

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        // Chunks are not acknowledged: a corrupted one only costs that frame on the S3
        if (frame) {
            offset += payload;
            if (offset >= frame->len) {
                frameq_pop();
                frame = NULL;
            }
        }

        const cam_cmd_t *c = (const cam_cmd_t *)rx_buf;
        if (!link_parse_cmd(c))
            continue;

        s3_offset = (int32_t)(c->s3_time_ms - now_ms);
        synced    = true;

        cam_cmd_code_t new_mode = c->cmd == CAM_CMD_RECORD ? CAM_CMD_RECORD : CAM_CMD_STANDBY;
        if (new_mode != mode) {
            if (new_mode == CAM_CMD_STANDBY && frame) {
                frameq_release();
                frame = NULL;
            }
            frameq_set_recording(new_mode == CAM_CMD_RECORD);
            mode = new_mode;
            ESP_LOGW(TAG, "S3 command: %s", mode == CAM_CMD_RECORD ? "RECORD" : "STANDBY");
        }
    }
}

esp_err_t link_start(void) {
    gpio_config_t hs_cfg = {
        .pin_bit_mask = BIT64(LINK_HANDSHAKE),
        .mode         = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&hs_cfg), TAG, "handshake gpio");
    gpio_set_level(LINK_HANDSHAKE, 0);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = LINK_MOSI,
        .miso_io_num     = LINK_MISO,
        .sclk_io_num     = LINK_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = CAM_LINK_CHUNK_SIZE,
    };
    spi_slave_interface_config_t slave_cfg = {
        .spics_io_num  = LINK_CS,
        .queue_size    = 1,
        .mode          = 0,
        .post_setup_cb = link_post_setup,
        .post_trans_cb = link_post_trans,
    };
    ESP_RETURN_ON_ERROR(spi_slave_initialize(LINK_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO), TAG, "spi slave");

    // CS idle high when the S3 is not connected yet
    gpio_set_pull_mode(LINK_CS, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(LINK_SCLK, GPIO_PULLDOWN_ONLY);

    tx_buf = spi_bus_dma_memory_alloc(LINK_HOST, CAM_LINK_CHUNK_SIZE, 0);
    rx_buf = spi_bus_dma_memory_alloc(LINK_HOST, CAM_LINK_CHUNK_SIZE, 0);
    if (!tx_buf || !rx_buf)
        return ESP_ERR_NO_MEM;

    BaseType_t ok = xTaskCreatePinnedToCore(link_task, "link", LINK_STACK, NULL, LINK_PRIORITY, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
