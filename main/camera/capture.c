#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_cam_sensor_xclk.h"
#include "esp_check.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "linux/videodev2.h"

#include "camera.h"

static const char *TAG = "capture";

/* ESP32-P4-EYE board wiring (from the Espressif BSP / schematic v2.3) */
#define BOARD_SCCB_FREQ_HZ 100000
#define BOARD_CAM_XCLK     GPIO_NUM_11
#define BOARD_CAM_XCLK_HZ  20000000    // SC035HGS 640x480 48 fps mode expects a 20 MHz input clock
#define BOARD_PWR_CTRL     GPIO_NUM_12 // Camera LDO and 3V3_PERI rail (also powers the I2C pull-ups)
#define BOARD_CAM_RESET    GPIO_NUM_26
#define BOARD_C6_EN        GPIO_NUM_9  // Wi-Fi/BT module, kept off
#define BOARD_LCD_BL_N     GPIO_NUM_20 // P-MOS switch, high = backlight off
#define BOARD_FLASHLIGHT   GPIO_NUM_23

#define CAM_BUF_NUM       3
#define CAPTURE_PRIORITY  3 // Below the sensor tasks (5) sharing core 1
#define CAPTURE_CORE      1
#define CAPTURE_STACK     4096
#define STATS_INTERVAL_MS 5000

static int                            video_fd = -1;
static uint8_t                       *cam_buf[CAM_BUF_NUM];
static esp_h264_enc_handle_t          h264_enc;
static esp_h264_enc_param_hw_handle_t h264_param;

void board_power_init(void) {
    const struct {
        gpio_num_t pin;
        int        level;
    } outputs[] = {
        {BOARD_PWR_CTRL, 1}, {BOARD_CAM_RESET, 1}, {BOARD_C6_EN, 0}, {BOARD_LCD_BL_N, 1}, {BOARD_FLASHLIGHT, 0},
    };

    for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
        gpio_reset_pin(outputs[i].pin);
        gpio_set_direction(outputs[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(outputs[i].pin, outputs[i].level);
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Sensor power-up before SCCB access
}

static esp_err_t camera_init(i2c_master_bus_handle_t i2c) {
    esp_cam_sensor_xclk_handle_t xclk;
    esp_cam_sensor_xclk_config_t xclk_cfg = {
        .esp_clock_router_cfg =
            {
                .xclk_pin     = BOARD_CAM_XCLK,
                .xclk_freq_hz = BOARD_CAM_XCLK_HZ,
            },
    };
    ESP_RETURN_ON_ERROR(esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &xclk), TAG, "xclk alloc");
    ESP_RETURN_ON_ERROR(esp_cam_sensor_xclk_start(xclk, &xclk_cfg), TAG, "xclk start");

    esp_video_init_csi_config_t csi_cfg = {
        .sccb_config =
            {
                .init_sccb  = false,
                .i2c_handle = i2c,
                .freq       = BOARD_SCCB_FREQ_HZ,
            },
        .reset_pin = -1,
        .pwdn_pin  = -1,
    };
    esp_video_init_config_t video_cfg = {
        .csi = &csi_cfg,
    };
    return esp_video_init(&video_cfg);
}

static esp_err_t stream_open(void) {
    video_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (video_fd < 0) {
        ESP_LOGE(TAG, "Cannot open %s", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
        return ESP_FAIL;
    }

    // The ISP's YUV420 is the packed O_UYY_E_VYY layout the hardware H.264 encoder takes as input
    struct v4l2_format fmt = {
        .type                = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix.width       = CAM_WIDTH,
        .fmt.pix.height      = CAM_HEIGHT,
        .fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420,
    };
    if (ioctl(video_fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "YUV420 %dx%d not supported by the sensor", CAM_WIDTH, CAM_HEIGHT);
        return ESP_FAIL;
    }
    if (fmt.fmt.pix.width != CAM_WIDTH || fmt.fmt.pix.height != CAM_HEIGHT) {
        ESP_LOGE(TAG, "Sensor gave %lux%lu, expected %dx%d", fmt.fmt.pix.width, fmt.fmt.pix.height, CAM_WIDTH,
                 CAM_HEIGHT);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Sensor stream %dx%d YUV420", CAM_WIDTH, CAM_HEIGHT);

    struct v4l2_requestbuffers req = {
        .count  = CAM_BUF_NUM,
        .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0)
        return ESP_FAIL;

    for (int i = 0; i < CAM_BUF_NUM; i++) {
        struct v4l2_buffer buf = {
            .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index  = i,
        };
        if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) != 0)
            return ESP_FAIL;
        cam_buf[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, video_fd, buf.m.offset);
        if (!cam_buf[i] || ioctl(video_fd, VIDIOC_QBUF, &buf) != 0)
            return ESP_FAIL;
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return ioctl(video_fd, VIDIOC_STREAMON, &type) == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t encoder_init(void) {
    esp_h264_enc_cfg_hw_t cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop      = CAM_H264_GOP,
        .fps      = CAM_FPS,
        .res      = {.width = CAM_WIDTH, .height = CAM_HEIGHT},
        .rc       = {.bitrate = CAM_H264_BITRATE, .qp_min = CAM_H264_QP_MIN, .qp_max = CAM_H264_QP_MAX},
    };
    if (esp_h264_enc_hw_new(&cfg, &h264_enc) != ESP_H264_ERR_OK ||
        esp_h264_enc_hw_get_param_hd(h264_enc, &h264_param) != ESP_H264_ERR_OK ||
        esp_h264_enc_open(h264_enc) != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "H.264 encoder init failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void capture_task(void *arg) {
    uint32_t   seq        = 0;
    uint32_t   encoded    = 0;
    uint32_t   keyframes  = 0;
    uint64_t   bytes      = 0;
    bool       resync     = false; // an encoded frame was lost: P-frames are useless until the next keyframe
    TickType_t last_stats = xTaskGetTickCount();

    while (true) {
        struct v4l2_buffer buf = {
            .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(video_fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "DQBUF failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        uint32_t     capture_ms = (uint32_t)(esp_timer_get_time() / 1000);
        cam_frame_t *slot       = (buf.flags & V4L2_BUF_FLAG_DONE) ? frameq_acquire_write() : NULL;

        // No slot: the frame is skipped before encoding, so the H.264 stream stays decodable
        if (!slot) {
            ioctl(video_fd, VIDIOC_QBUF, &buf);
            continue;
        }

        esp_h264_enc_in_frame_t in = {
            .raw_data = {.buffer = cam_buf[buf.index], .len = CAM_WIDTH * CAM_HEIGHT * 3 / 2},
            .pts      = capture_ms,
        };
        esp_h264_enc_out_frame_t out = {
            .raw_data = {.buffer = slot->data, .len = slot->size},
        };
        esp_h264_err_t err = esp_h264_enc_process(h264_enc, &in, &out);

        // The encoder reads the camera buffer, so it is only returned after encoding
        ioctl(video_fd, VIDIOC_QBUF, &buf);

        if (err != ESP_H264_ERR_OK) {
            ESP_LOGW(TAG, "H.264 frame lost (%d), forcing a keyframe", err);
            esp_h264_enc_force_idr(&h264_param->base);
            resync = true;
            continue;
        }

        bool keyframe = out.frame_type == ESP_H264_FRAME_TYPE_IDR || out.frame_type == ESP_H264_FRAME_TYPE_I;
        if (resync && !keyframe)
            continue;
        resync = false;

        frameq_commit(out.length, seq++, capture_ms, keyframe);
        encoded++;
        keyframes += keyframe;
        bytes += out.length;

        TickType_t now = xTaskGetTickCount();
        if (now - last_stats >= pdMS_TO_TICKS(STATS_INTERVAL_MS)) {
            float secs = (now - last_stats) * portTICK_PERIOD_MS / 1000.0f;
            ESP_LOGI(TAG, "%.1f fps encoded, %.0f kbit/s, %lu keyframes, dropped %lu", encoded / secs,
                     bytes * 8 / 1000.0f / secs, keyframes, frameq_dropped());
            encoded    = 0;
            keyframes  = 0;
            bytes      = 0;
            last_stats = now;
        }
    }
}

esp_err_t capture_start(i2c_master_bus_handle_t i2c_bus) {
    ESP_RETURN_ON_ERROR(frameq_init(), TAG, "frame queue");
    ESP_RETURN_ON_ERROR(camera_init(i2c_bus), TAG, "camera");
    ESP_RETURN_ON_ERROR(encoder_init(), TAG, "h264");
    ESP_RETURN_ON_ERROR(stream_open(), TAG, "stream");

    BaseType_t ok =
        xTaskCreatePinnedToCore(capture_task, "capture", CAPTURE_STACK, NULL, CAPTURE_PRIORITY, NULL, CAPTURE_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
