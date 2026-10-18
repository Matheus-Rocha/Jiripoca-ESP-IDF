#include "global.h"

#include "camera.h"

static const char *TAG_CAM = "CAMERA";

#define CAM_MOUNT             "/sdcard"
#define CAM_FILENAME_LENGTH   32
#define CAM_STANDBY_POLL_MS   100
#define CAM_FRAME_WAIT_MS     100
#define CAM_SYNC_INTERVAL_MS  2000
#define CAM_STATS_INTERVAL_MS 5000

static uint8_t cam_flight_status(void) {
    portENTER_CRITICAL(&xDATAMutex);
    uint8_t status = data_g.status;
    portEXIT_CRITICAL(&xDATAMutex);
    return status;
}

static FILE *cam_open_file(void) {
    char name[CAM_FILENAME_LENGTH];
    snprintf(name, sizeof(name), "%s/cam%lu.vid", CAM_MOUNT, file_counter_g.sd_files);

    FILE *f = fopen(name, "wb");
    if (!f) {
        ESP_LOGE(TAG_CAM, "Failed to create %s", name);
        return NULL;
    }

    cam_file_hdr_t hdr = {
        .magic   = CAM_FILE_MAGIC,
        .version = CAM_FILE_VERSION,
        .fps     = CAM_FPS,
        .open_ms = (uint32_t)(esp_timer_get_time() / 1000),
    };
    fwrite(&hdr, 1, sizeof(hdr), f);
    ESP_LOGW(TAG_CAM, "Recording to %s", name);
    return f;
}

static bool cam_write_frame(FILE *f, const cam_frame_t *frame, uint8_t status) {
    cam_rec_hdr_t rec = {
        .magic      = CAM_REC_MAGIC,
        .seq        = frame->seq,
        .capture_ms = frame->capture_ms,
        .write_ms   = (uint32_t)(esp_timer_get_time() / 1000),
        .len        = frame->len,
        .status     = status,
        .keyframe   = frame->keyframe,
    };
    return fwrite(&rec, 1, sizeof(rec), f) == sizeof(rec) && fwrite(frame->data, 1, frame->len, f) == frame->len;
}

void task_camera(void *pvParameters) {
    // The camera shares the I2C bus created by task_setup and writes to the card mounted by task_sd
    EventBits_t bits = xEventGroupWaitBits(xCamEventGroup, CAM_EVT_BUS_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    esp_err_t   err  = capture_start(bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_CAM, "Camera init failed: %s", esp_err_to_name(err));
        goto done;
    }

    bits = xEventGroupWaitBits(xCamEventGroup, CAM_EVT_SD_READY | CAM_EVT_SD_FAILED, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & CAM_EVT_SD_FAILED) {
        ESP_LOGE(TAG_CAM, "SD card unavailable, video will not be recorded");
        goto done;
    }

    FILE      *f           = NULL;
    bool       recording   = false;
    uint32_t   frames      = 0;
    uint32_t   frames_prev = 0;
    uint64_t   bytes       = 0;
    uint64_t   bytes_prev  = 0;
    uint32_t   write_errs  = 0;
    TickType_t last_sync   = xTaskGetTickCount();
    TickType_t last_stats  = xTaskGetTickCount();

    while (true) {
        uint8_t status = cam_flight_status();
        if (status & LANDED)
            break;

        bool record = status & BOOST;
        if (record != recording) {
            frameq_set_recording(record);
            recording = record;
        }
        if (!record) {
            vTaskDelay(pdMS_TO_TICKS(CAM_STANDBY_POLL_MS));
            continue;
        }

        cam_frame_t *frame = frameq_peek();
        if (!frame) {
            frameq_wait(CAM_FRAME_WAIT_MS);
            continue;
        }

        if (!f)
            f = cam_open_file();
        if (f && cam_write_frame(f, frame, status)) {
            frames++;
            bytes += frame->len;
        } else
            write_errs++;
        frameq_pop();

        TickType_t now = xTaskGetTickCount();

        // Bounds what an impact or power cut can lose
        if (f && now - last_sync >= pdMS_TO_TICKS(CAM_SYNC_INTERVAL_MS)) {
            fflush(f);
            fsync(fileno(f));
            last_sync = now;
        }

        if (now - last_stats >= pdMS_TO_TICKS(CAM_STATS_INTERVAL_MS)) {
            float secs = (now - last_stats) * portTICK_PERIOD_MS / 1000.0f;
            ESP_LOGI(TAG_CAM, "%.1f fps, %.0f KB/s written | write errors %lu, dropped %lu",
                     (frames - frames_prev) / secs, (bytes - bytes_prev) / 1024.0f / secs, write_errs,
                     frameq_dropped());
            frames_prev = frames;
            bytes_prev  = bytes;
            last_stats  = now;
        }
    }

    frameq_set_recording(false);
    if (f) {
        fflush(f);
        fsync(fileno(f));
        fclose(f);
        ESP_LOGW(TAG_CAM, "Landed, video closed: %lu frames, %lu KB", frames, (uint32_t)(bytes / 1024));
    }

done:
    xEventGroupSetBits(xCamEventGroup, CAM_EVT_DONE);
    vTaskDelete(NULL);
}
