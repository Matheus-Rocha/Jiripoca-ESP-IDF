#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/jpeg_encode.h"
#include "esp_log.h"

#include "cam_p4.h"

static const char *TAG = "frameq";

static cam_frame_t       slots[CAM_QUEUE_FRAMES];
static uint32_t          head, count;
static bool              recording;
static bool              head_in_tx; // link task is sending slots[head]
static uint32_t          dropped;
static portMUX_TYPE      lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t ready_sem;

esp_err_t frameq_init(void) {
    ready_sem = xSemaphoreCreateBinary();
    if (!ready_sem)
        return ESP_ERR_NO_MEM;

    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    for (int i = 0; i < CAM_QUEUE_FRAMES; i++) {
        size_t size   = 0;
        slots[i].data = jpeg_alloc_encoder_mem(CAM_LINK_FRAME_MAX, &mem_cfg, &size);
        if (!slots[i].data) {
            ESP_LOGE(TAG, "No memory for frame slot %d", i);
            return ESP_ERR_NO_MEM;
        }
        slots[i].size = size;
    }
    return ESP_OK;
}

void frameq_set_recording(bool rec) {
    portENTER_CRITICAL(&lock);
    recording = rec;
    portEXIT_CRITICAL(&lock);
}

// Returns the slot to encode into, or NULL if the frame must be dropped
cam_frame_t *frameq_acquire_write(void) {
    cam_frame_t *slot = NULL;

    portENTER_CRITICAL(&lock);
    uint32_t limit = recording ? CAM_QUEUE_FRAMES : CAM_PRETRIGGER_FRAMES;
    while (count >= limit && !head_in_tx) {
        head = (head + 1) % CAM_QUEUE_FRAMES;
        count--;
        if (recording)
            dropped++;
    }
    if (count < limit)
        slot = &slots[(head + count) % CAM_QUEUE_FRAMES];
    else
        dropped++;
    portEXIT_CRITICAL(&lock);

    return slot;
}

void frameq_commit(uint32_t len, uint32_t seq, uint32_t capture_ms) {
    portENTER_CRITICAL(&lock);
    cam_frame_t *slot = &slots[(head + count) % CAM_QUEUE_FRAMES];
    slot->len         = len;
    slot->seq         = seq;
    slot->capture_ms  = capture_ms;
    count++;
    portEXIT_CRITICAL(&lock);

    xSemaphoreGive(ready_sem);
}

cam_frame_t *frameq_peek(void) {
    cam_frame_t *slot = NULL;

    portENTER_CRITICAL(&lock);
    if (count > 0) {
        head_in_tx = true;
        slot       = &slots[head];
    }
    portEXIT_CRITICAL(&lock);

    return slot;
}

void frameq_pop(void) {
    portENTER_CRITICAL(&lock);
    if (count > 0) {
        head = (head + 1) % CAM_QUEUE_FRAMES;
        count--;
    }
    head_in_tx = false;
    portEXIT_CRITICAL(&lock);
}

void frameq_release(void) {
    portENTER_CRITICAL(&lock);
    head_in_tx = false;
    portEXIT_CRITICAL(&lock);
}

bool frameq_wait(uint32_t timeout_ms) {
    return xSemaphoreTake(ready_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

uint32_t frameq_dropped(void) {
    portENTER_CRITICAL(&lock);
    uint32_t d = dropped;
    portEXIT_CRITICAL(&lock);
    return d;
}
