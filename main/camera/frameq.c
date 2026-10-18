#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_private/esp_cache_private.h"

#include "camera.h"

static const char *TAG = "frameq";

static cam_frame_t       slots[CAM_QUEUE_FRAMES];
static uint32_t          head, count;
static bool              recording;
static bool              head_in_tx; // recorder is writing slots[head]
static uint32_t          dropped;
static portMUX_TYPE      lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t ready_sem;

esp_err_t frameq_init(void) {
    ready_sem = xSemaphoreCreateBinary();
    if (!ready_sem)
        return ESP_ERR_NO_MEM;

    // The encoder writes the slots by DMA: they must be cache-line aligned
    size_t align = 0;
    esp_err_t err = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &align);
    if (err != ESP_OK)
        return err;
    size_t size = (CAM_FRAME_MAX + align - 1) & ~(align - 1);

    for (int i = 0; i < CAM_QUEUE_FRAMES; i++) {
        slots[i].data = heap_caps_aligned_calloc(align, 1, size, MALLOC_CAP_SPIRAM);
        if (!slots[i].data) {
            ESP_LOGE(TAG, "No PSRAM for frame slot %d", i);
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

static void drop_head(void) {
    head = (head + 1) % CAM_QUEUE_FRAMES;
    count--;
}

// Returns the slot to encode into, or NULL if the frame must be skipped (it is then never encoded)
cam_frame_t *frameq_acquire_write(void) {
    cam_frame_t *slot = NULL;

    portENTER_CRITICAL(&lock);
    if (!recording && !head_in_tx) {
        // Standby ring: forget the oldest frames, but the ring must always start on a keyframe
        while (count >= CAM_PRETRIGGER_FRAMES)
            drop_head();
        while (count > 0 && !slots[head].keyframe)
            drop_head();
    }

    uint32_t limit = recording ? CAM_QUEUE_FRAMES : CAM_PRETRIGGER_FRAMES;
    if (count < limit)
        slot = &slots[(head + count) % CAM_QUEUE_FRAMES];
    else if (recording)
        dropped++;
    portEXIT_CRITICAL(&lock);

    return slot;
}

void frameq_commit(uint32_t len, uint32_t seq, uint32_t capture_ms, bool keyframe) {
    portENTER_CRITICAL(&lock);
    cam_frame_t *slot = &slots[(head + count) % CAM_QUEUE_FRAMES];
    slot->len         = len;
    slot->seq         = seq;
    slot->capture_ms  = capture_ms;
    slot->keyframe    = keyframe;
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
    if (count > 0)
        drop_head();
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
