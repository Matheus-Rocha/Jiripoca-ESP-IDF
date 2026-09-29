#include "global.h"

#include <esp_heap_caps.h>
#include <esp_rom_crc.h>

#include "cam_protocol.h"

static const char *TAG_CAM = "CAMERA";

#define CAM_MOUNT              "/sdcard"
#define CAM_FILENAME_LENGTH    32
#define CAM_WAIT_REC_MS        100
#define CAM_WAIT_STANDBY_MS    500
#define CAM_STANDBY_POLL_MS    100
#define CAM_SYNC_INTERVAL_MS   2000
#define CAM_STATS_INTERVAL_MS  5000
#define CAM_LINK_WARN_INTERVAL 5000

typedef struct {
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t crc_errors;
    uint32_t link_timeouts;
    uint32_t write_errors;
    uint32_t bytes;
    uint32_t p4_dropped;
} cam_stats_t;

static SemaphoreHandle_t   xCamReadySem = NULL;
static spi_device_handle_t cam_spi      = NULL;
static uint8_t            *cam_tx       = NULL;
static uint8_t            *cam_rx       = NULL;
static uint8_t             cam_frame[CAM_LINK_FRAME_MAX];

static void IRAM_ATTR cam_handshake_isr(void *arg) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(xCamReadySem, &woken);
    if (woken)
        portYIELD_FROM_ISR();
}

static esp_err_t cam_link_init(void) {
    xCamReadySem = xSemaphoreCreateBinary();
    cam_tx       = heap_caps_calloc(1, CAM_LINK_CHUNK_SIZE, MALLOC_CAP_DMA);
    cam_rx       = heap_caps_calloc(1, CAM_LINK_CHUNK_SIZE, MALLOC_CAP_DMA);
    if (!xCamReadySem || !cam_tx || !cam_rx)
        return ESP_ERR_NO_MEM;

    gpio_config_t hs_cfg = {
        .pin_bit_mask = BIT64(CAM_HANDSHAKE_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // Stays low (not ready) if the P4 is absent
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&hs_cfg), TAG_CAM, "handshake gpio");

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        return err;
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CAM_HANDSHAKE_GPIO, cam_handshake_isr, NULL), TAG_CAM, "handshake isr");

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = CAM_SPI_MOSI,
        .miso_io_num     = CAM_SPI_MISO,
        .sclk_io_num     = CAM_SPI_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = CAM_LINK_CHUNK_SIZE,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(CAM_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO), TAG_CAM, "spi bus");

    spi_device_interface_config_t dev_cfg = {
        .mode             = 0,
        .clock_speed_hz   = CAM_SPI_FREQ_HZ,
        .spics_io_num     = CAM_SPI_CS,
        .cs_ena_posttrans = 3,
        .queue_size       = 1,
    };
    return spi_bus_add_device(CAM_SPI_HOST, &dev_cfg, &cam_spi);
}

static bool cam_wait_ready(uint32_t timeout_ms) {
    if (xSemaphoreTake(xCamReadySem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
        return true;

    // The rising edge may have happened before the ISR was attached
    if (gpio_get_level(CAM_HANDSHAKE_GPIO)) {
        xSemaphoreTake(xCamReadySem, 0);
        return true;
    }
    return false;
}

static esp_err_t cam_transfer(cam_cmd_code_t cmd) {
    cam_cmd_t *c  = (cam_cmd_t *)cam_tx;
    c->magic      = CAM_LINK_MAGIC_CMD;
    c->s3_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
    c->cmd        = cmd;
    c->crc32      = esp_rom_crc32_le(0, cam_tx, offsetof(cam_cmd_t, crc32));

    spi_transaction_t t = {
        .length    = CAM_LINK_CHUNK_SIZE * 8,
        .tx_buffer = cam_tx,
        .rx_buffer = cam_rx,
    };
    return spi_device_transmit(cam_spi, &t);
}

static bool cam_chunk_valid(const cam_chunk_hdr_t *h) {
    if (h->magic != CAM_LINK_MAGIC_DATA || h->payload_len > CAM_LINK_PAYLOAD_MAX)
        return false;

    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)h, offsetof(cam_chunk_hdr_t, crc32));
    crc          = esp_rom_crc32_le(crc, (const uint8_t *)h + sizeof(cam_chunk_hdr_t), h->payload_len);
    return crc == h->crc32;
}

static uint8_t cam_flight_status(void) {
    portENTER_CRITICAL(&xDATAMutex);
    uint8_t status = data_g.status;
    portEXIT_CRITICAL(&xDATAMutex);
    return status;
}

static FILE *cam_open_file(void) {
    char name[CAM_FILENAME_LENGTH];
    snprintf(name, sizeof(name), "%s/cam%lu.mjp", CAM_MOUNT, file_counter_g.sd_files);

    FILE *f = fopen(name, "wb");
    if (!f) {
        ESP_LOGE(TAG_CAM, "Failed to create %s", name);
        return NULL;
    }

    cam_file_hdr_t hdr = {
        .magic      = CAM_FILE_MAGIC,
        .version    = CAM_FILE_VERSION,
        .s3_open_ms = (uint32_t)(esp_timer_get_time() / 1000),
    };
    fwrite(&hdr, 1, sizeof(hdr), f);
    ESP_LOGW(TAG_CAM, "Recording to %s", name);
    return f;
}

static bool cam_write_frame(FILE *f, uint32_t seq, uint32_t capture_ms, uint32_t len, uint8_t status) {
    cam_rec_hdr_t rec = {
        .magic      = CAM_REC_MAGIC,
        .seq        = seq,
        .capture_ms = capture_ms,
        .write_ms   = (uint32_t)(esp_timer_get_time() / 1000),
        .len        = len,
        .status     = status,
    };
    return fwrite(&rec, 1, sizeof(rec), f) == sizeof(rec) && fwrite(cam_frame, 1, len, f) == len;
}

static void cam_close_file(FILE *f) {
    fflush(f);
    fsync(fileno(f));
    fclose(f);
}

void task_camera(void *pvParameters) {
    EventBits_t sd_bits = xEventGroupWaitBits(xCamEventGroup, CAM_EVT_SD_READY | CAM_EVT_SD_FAILED, pdFALSE,
                                              pdFALSE, portMAX_DELAY);
    if (sd_bits & CAM_EVT_SD_FAILED) {
        ESP_LOGE(TAG_CAM, "SD card unavailable, camera disabled");
        goto done;
    }

    esp_err_t err = cam_link_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_CAM, "Camera link init failed: %s", esp_err_to_name(err));
        goto done;
    }
    ESP_LOGI(TAG_CAM, "Camera link ready (SPI %d Hz), free internal heap: %u", CAM_SPI_FREQ_HZ,
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    FILE       *f          = NULL;
    cam_stats_t st         = {0};
    cam_stats_t st_prev    = {0};
    bool        in_frame   = false;
    uint32_t    frame_seq  = 0;
    uint32_t    frame_len  = 0;
    uint32_t    frame_got  = 0;
    uint32_t    frame_time = 0;
    TickType_t  last_sync  = xTaskGetTickCount();
    TickType_t  last_stats = xTaskGetTickCount();
    TickType_t  last_warn  = 0;

    while (true) {
        uint8_t status = cam_flight_status();
        bool    landed = status & LANDED;
        bool    record = (status & BOOST) && !landed;

        if (landed)
            break;

        if (!cam_wait_ready(record ? CAM_WAIT_REC_MS : CAM_WAIT_STANDBY_MS)) {
            st.link_timeouts++;
            if (xTaskGetTickCount() - last_warn >= pdMS_TO_TICKS(CAM_LINK_WARN_INTERVAL)) {
                ESP_LOGW(TAG_CAM, "P4 not responding (handshake low)");
                last_warn = xTaskGetTickCount();
            }
            continue;
        }

        if (cam_transfer(record ? CAM_CMD_RECORD : CAM_CMD_STANDBY) != ESP_OK) {
            st.link_timeouts++;
            continue;
        }

        const cam_chunk_hdr_t *h = (const cam_chunk_hdr_t *)cam_rx;
        if (!cam_chunk_valid(h)) {
            st.crc_errors++;
            in_frame = false;
            continue;
        }
        st.p4_dropped = h->dropped;

        if (h->frame_len == 0) {
            if (!record)
                vTaskDelay(pdMS_TO_TICKS(CAM_STANDBY_POLL_MS));
            continue;
        }
        if (!record)
            continue;

        if (h->offset == 0) {
            in_frame   = h->frame_len <= CAM_LINK_FRAME_MAX;
            frame_seq  = h->frame_seq;
            frame_len  = h->frame_len;
            frame_time = h->capture_ms;
            frame_got  = 0;
            if (!in_frame)
                st.frames_bad++;
        }

        if (!in_frame)
            continue;

        if (h->frame_seq != frame_seq || h->offset != frame_got || frame_got + h->payload_len > frame_len) {
            st.frames_bad++;
            in_frame = false;
            continue;
        }

        memcpy(&cam_frame[frame_got], (const uint8_t *)h + sizeof(cam_chunk_hdr_t), h->payload_len);
        frame_got += h->payload_len;

        if (frame_got < frame_len)
            continue;

        in_frame = false;
        if (!f)
            f = cam_open_file();
        if (!f || !cam_write_frame(f, frame_seq, frame_time, frame_len, status)) {
            st.write_errors++;
            continue;
        }
        st.frames_ok++;
        st.bytes += frame_len;

        // Bounds what an impact or power cut can lose
        if (xTaskGetTickCount() - last_sync >= pdMS_TO_TICKS(CAM_SYNC_INTERVAL_MS)) {
            fflush(f);
            fsync(fileno(f));
            last_sync = xTaskGetTickCount();
        }

        TickType_t now = xTaskGetTickCount();
        if (now - last_stats >= pdMS_TO_TICKS(CAM_STATS_INTERVAL_MS)) {
            float secs = (now - last_stats) * portTICK_PERIOD_MS / 1000.0f;
            ESP_LOGI(TAG_CAM, "%.1f fps, %.0f KB/s | bad %lu, crc %lu, timeouts %lu, write err %lu, P4 dropped %lu",
                     (st.frames_ok - st_prev.frames_ok) / secs, (st.bytes - st_prev.bytes) / 1024.0f / secs,
                     st.frames_bad, st.crc_errors, st.link_timeouts, st.write_errors, st.p4_dropped);
            st_prev    = st;
            last_stats = now;
        }
    }

    if (f) {
        cam_close_file(f);
        ESP_LOGW(TAG_CAM, "Landed, video closed: %lu frames, %lu KB", st.frames_ok, st.bytes / 1024);
    }

    // Best effort: tell the P4 to stop streaming
    if (cam_wait_ready(CAM_WAIT_STANDBY_MS))
        cam_transfer(CAM_CMD_STANDBY);

done:
    xEventGroupSetBits(xCamEventGroup, CAM_EVT_DONE);
    vTaskDelete(NULL);
}
