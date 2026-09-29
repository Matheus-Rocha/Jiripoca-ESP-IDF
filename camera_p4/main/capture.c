#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/jpeg_encode.h"
#include "driver/ppa.h"
#include "esp_cam_sensor_xclk.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_private/esp_cache_private.h"
#include "esp_timer.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "linux/videodev2.h"

#include "cam_p4.h"

static const char *TAG = "capture";

/* ESP32-P4-EYE board wiring (from the Espressif BSP / schematic v2.3) */
#define BOARD_I2C_PORT      I2C_NUM_1
#define BOARD_I2C_SCL       GPIO_NUM_13
#define BOARD_I2C_SDA       GPIO_NUM_14
#define BOARD_SCCB_FREQ_HZ  100000
#define BOARD_CAM_XCLK      GPIO_NUM_11
#define BOARD_CAM_XCLK_HZ   24000000
#define BOARD_PWR_CTRL      GPIO_NUM_12 // Camera LDO and 3V3_PERI rail
#define BOARD_CAM_RESET     GPIO_NUM_26
#define BOARD_C6_EN         GPIO_NUM_9  // Wi-Fi/BT module, kept off
#define BOARD_SD_PWR_N      GPIO_NUM_46 // P-MOS switch, high = SD card unpowered
#define BOARD_LCD_BL_N      GPIO_NUM_20 // P-MOS switch, high = backlight off
#define BOARD_FLASHLIGHT    GPIO_NUM_23

#define CAM_BUF_NUM       3
#define CAPTURE_PRIORITY  5
#define CAPTURE_STACK     4096
#define STATS_INTERVAL_MS 5000

static int                   video_fd = -1;
static uint8_t              *cam_buf[CAM_BUF_NUM];
static uint32_t              cam_w, cam_h;
static uint8_t              *scaled_buf;
static size_t                scaled_size;
static ppa_client_handle_t   ppa_srm;
static jpeg_encoder_handle_t jpeg_enc;

static void board_power_init(void) {
    const struct {
        gpio_num_t pin;
        int        level;
    } outputs[] = {
        {BOARD_PWR_CTRL, 1}, {BOARD_CAM_RESET, 1},  {BOARD_C6_EN, 0},
        {BOARD_SD_PWR_N, 1}, {BOARD_LCD_BL_N, 1}, {BOARD_FLASHLIGHT, 0},
    };

    for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
        gpio_reset_pin(outputs[i].pin);
        gpio_set_direction(outputs[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(outputs[i].pin, outputs[i].level);
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Sensor power-up before SCCB access
}

static esp_err_t camera_init(void) {
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

    i2c_master_bus_handle_t     i2c;
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port                     = BOARD_I2C_PORT,
        .scl_io_num                   = BOARD_I2C_SCL,
        .sda_io_num                   = BOARD_I2C_SDA,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = false, // Board has external pull-ups
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_cfg, &i2c), TAG, "i2c bus");

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

    struct v4l2_format fmt = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
    if (ioctl(video_fd, VIDIOC_G_FMT, &fmt) != 0)
        return ESP_FAIL;

    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    if (ioctl(video_fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "RGB565 not supported");
        return ESP_FAIL;
    }
    cam_w = fmt.fmt.pix.width;
    cam_h = fmt.fmt.pix.height;
    ESP_LOGI(TAG, "Sensor stream %lux%lu RGB565", cam_w, cam_h);

    if (cam_w < CAM_CROP_WIDTH || cam_h < CAM_CROP_HEIGHT) {
        ESP_LOGE(TAG, "Sensor frame smaller than the %dx%d crop", CAM_CROP_WIDTH, CAM_CROP_HEIGHT);
        return ESP_FAIL;
    }

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

static esp_err_t processing_init(void) {
    ppa_client_config_t ppa_cfg = {.oper_type = PPA_OPERATION_SRM};
    ESP_RETURN_ON_ERROR(ppa_register_client(&ppa_cfg, &ppa_srm), TAG, "ppa");

    jpeg_encode_engine_cfg_t jpeg_cfg = {.timeout_ms = 70};
    ESP_RETURN_ON_ERROR(jpeg_new_encoder_engine(&jpeg_cfg, &jpeg_enc), TAG, "jpeg");

    size_t align = 0;
    ESP_RETURN_ON_ERROR(esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &align), TAG, "cache align");
    scaled_size = (CAM_OUT_WIDTH * CAM_OUT_HEIGHT * 2 + align - 1) & ~(align - 1);
    scaled_buf  = heap_caps_aligned_calloc(align, 1, scaled_size, MALLOC_CAP_SPIRAM);
    return scaled_buf ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t scale_frame(const uint8_t *src) {
    ppa_srm_oper_config_t srm = {
        .in.buffer         = src,
        .in.pic_w          = cam_w,
        .in.pic_h          = cam_h,
        .in.block_w        = CAM_CROP_WIDTH,
        .in.block_h        = CAM_CROP_HEIGHT,
        .in.block_offset_x = (cam_w - CAM_CROP_WIDTH) / 2,
        .in.block_offset_y = (cam_h - CAM_CROP_HEIGHT) / 2,
        .in.srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        .out.buffer        = scaled_buf,
        .out.buffer_size   = scaled_size,
        .out.pic_w         = CAM_OUT_WIDTH,
        .out.pic_h         = CAM_OUT_HEIGHT,
        .out.srm_cm        = PPA_SRM_COLOR_MODE_RGB565,
        .rotation_angle    = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x           = (float)CAM_OUT_WIDTH / CAM_CROP_WIDTH,
        .scale_y           = (float)CAM_OUT_HEIGHT / CAM_CROP_HEIGHT,
        .mode              = PPA_TRANS_MODE_BLOCKING,
    };
    return ppa_do_scale_rotate_mirror(ppa_srm, &srm);
}

static void capture_task(void *arg) {
    uint32_t   quality    = CAM_JPEG_QUALITY_INIT;
    uint32_t   seq        = 0;
    uint32_t   encoded    = 0;
    uint64_t   bytes      = 0;
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

        uint32_t  capture_ms = (uint32_t)(esp_timer_get_time() / 1000);
        esp_err_t err        = ESP_FAIL;
        if (buf.flags & V4L2_BUF_FLAG_DONE)
            err = scale_frame(cam_buf[buf.index]);

        // Give the buffer back before the slow part so the sensor never stalls
        ioctl(video_fd, VIDIOC_QBUF, &buf);
        if (err != ESP_OK)
            continue;

        cam_frame_t *slot = frameq_acquire_write();
        if (!slot)
            continue;

        jpeg_encode_cfg_t enc = {
            .width         = CAM_OUT_WIDTH,
            .height        = CAM_OUT_HEIGHT,
            .src_type      = JPEG_ENCODE_IN_FORMAT_RGB565,
            .sub_sample    = JPEG_DOWN_SAMPLING_YUV420,
            .image_quality = quality,
        };
        uint32_t len = 0;
        err = jpeg_encoder_process(jpeg_enc, &enc, scaled_buf, CAM_OUT_WIDTH * CAM_OUT_HEIGHT * 2, slot->data,
                                   slot->size, &len);

        if (err != ESP_OK || len > CAM_LINK_FRAME_MAX) {
            quality = quality > CAM_JPEG_QUALITY_MIN + 10 ? quality - 10 : CAM_JPEG_QUALITY_MIN;
            continue;
        }

        // Keep frames near the target size: the S3 link and SD budget depend on it
        if (len > CAM_JPEG_TARGET_BYTES && quality > CAM_JPEG_QUALITY_MIN)
            quality -= 2;
        else if (len < CAM_JPEG_TARGET_BYTES * 3 / 4 && quality < CAM_JPEG_QUALITY_MAX)
            quality += 1;

        frameq_commit(len, seq++, capture_ms);
        encoded++;
        bytes += len;

        TickType_t now = xTaskGetTickCount();
        if (now - last_stats >= pdMS_TO_TICKS(STATS_INTERVAL_MS)) {
            float secs = (now - last_stats) * portTICK_PERIOD_MS / 1000.0f;
            ESP_LOGI(TAG, "%.1f fps encoded, avg %lu B/frame, quality %lu, dropped %lu", encoded / secs,
                     encoded ? (uint32_t)(bytes / encoded) : 0, quality, frameq_dropped());
            encoded    = 0;
            bytes      = 0;
            last_stats = now;
        }
    }
}

esp_err_t capture_start(void) {
    board_power_init();
    ESP_RETURN_ON_ERROR(frameq_init(), TAG, "frame queue");
    ESP_RETURN_ON_ERROR(camera_init(), TAG, "camera");
    ESP_RETURN_ON_ERROR(processing_init(), TAG, "ppa/jpeg");
    ESP_RETURN_ON_ERROR(stream_open(), TAG, "stream");

    BaseType_t ok = xTaskCreatePinnedToCore(capture_task, "capture", CAPTURE_STACK, NULL, CAPTURE_PRIORITY, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
