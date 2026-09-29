#include "esp_log.h"

#include "cam_p4.h"

static const char *TAG = "main";

void app_main(void) {
    ESP_LOGI(TAG, "Jiripoca camera (ESP32-P4-EYE)");

    esp_err_t err = capture_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Capture init failed: %s", esp_err_to_name(err));
        return;
    }

    err = link_start();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Link init failed: %s", esp_err_to_name(err));
}
