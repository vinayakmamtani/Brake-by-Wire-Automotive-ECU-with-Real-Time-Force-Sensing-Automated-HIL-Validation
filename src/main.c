#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

// --- Hardware Pin Definitions ---
#define HX711_DT_PIN         GPIO_NUM_16
#define HX711_SCK_PIN        GPIO_NUM_17
#define I2C_MASTER_SDA_IO    GPIO_NUM_4
#define I2C_MASTER_SCL_IO    GPIO_NUM_15
#define I2C_MASTER_NUM       I2C_NUM_0
#define OLED_ADDR            0x3C

static const char *TAG = "BBW_ECU";

// --- Calibration & Signal Parameters ---
#define TARE_SAMPLES         20
#define DEADBAND_COUNTS      15000
#define MAX_STROKE_COUNTS    210000
#define FILTER_ALPHA         0.25f

// Thread-safe telemetry container
typedef struct {
    int32_t raw_adc;
    int32_t net_counts;
    int brake_pct;
    bool fault_active;
} bbw_telemetry_t;

static bbw_telemetry_t g_telemetry = {0};
static SemaphoreHandle_t g_telemetry_mutex = NULL;

// 128x64 display buffer (1024 bytes)
static uint8_t oled_buffer[1024];

// --- Low-Level HX711 Bit-Bang Driver ---
static int32_t hx711_read(void) {
    while (gpio_get_level(HX711_DT_PIN) == 1) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    int32_t data = 0;
    for (int i = 0; i < 24; i++) {
        gpio_set_level(HX711_SCK_PIN, 1);
        data = (data << 1) | gpio_get_level(HX711_DT_PIN);
        gpio_set_level(HX711_SCK_PIN, 0);
    }
    gpio_set_level(HX711_SCK_PIN, 1);
    gpio_set_level(HX711_SCK_PIN, 0);

    // 24-bit Two's complement sign extension
    if (data & 0x800000) {
        data |= 0xFF000000;
    }
    return data;
}

// --- I2C Master & SSD1306 Display Driver ---
static esp_err_t oled_write_cmd(uint8_t cmd) {
    i2c_cmd_handle_t link = i2c_cmd_link_create();
    i2c_master_start(link);
    i2c_master_write_byte(link, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, 0x00, true);
    i2c_master_write_byte(link, cmd, true);
    i2c_master_stop(link);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, link, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(link);
    return ret;
}

static void ssd1306_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    const uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        oled_write_cmd(init_cmds[i]);
    }
}

// Push local RAM buffer to SSD1306 display
static void oled_flush(void) {
    oled_write_cmd(0x21); oled_write_cmd(0); oled_write_cmd(127);
    oled_write_cmd(0x22); oled_write_cmd(0); oled_write_cmd(7);

    for (int page = 0; page < 8; page++) {
        i2c_cmd_handle_t link = i2c_cmd_link_create();
        i2c_master_start(link);
        i2c_master_write_byte(link, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(link, 0x40, true);
        i2c_master_write(link, &oled_buffer[page * 128], 128, true);
        i2c_master_stop(link);
        i2c_master_cmd_begin(I2C_MASTER_NUM, link, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(link);
    }
}

// Renders the dynamic progress bar (0 to 100%)
static void oled_draw_gauge(int pct, bool fault) {
    memset(oled_buffer, 0x00, sizeof(oled_buffer));

    // Outer border of the gauge (Page 3 and Page 4, x: 10 to 118)
    for (int x = 10; x <= 118; x++) {
        oled_buffer[2 * 128 + x] |= 0x01; // Top border line
        oled_buffer[5 * 128 + x] |= 0x80; // Bottom border line
    }
    for (int p = 2; p <= 5; p++) {
        oled_buffer[p * 128 + 10] |= 0xFF;  // Left border wall
        oled_buffer[p * 128 + 118] |= 0xFF; // Right border wall
    }

    if (fault) {
        // Stripe pattern across the gauge on fault
        for (int p = 3; p <= 4; p++) {
            for (int x = 14; x <= 114; x += 4) {
                oled_buffer[p * 128 + x] = 0xAA;
            }
        }
    } else {
        // Fill inner gauge proportional to brake percentage (width = 100 pixels)
        int fill_width = (pct * 100) / 100;
        for (int x = 14; x < (14 + fill_width); x++) {
            oled_buffer[3 * 128 + x] = 0xFF;
            oled_buffer[4 * 128 + x] = 0xFF;
        }
    }

    oled_flush();
}

// --- Task 1: Safety-Critical Control Loop (Core 1, 50 Hz) ---
void bbw_control_task(void *pvParameters) {
    gpio_set_direction(HX711_DT_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(HX711_SCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(HX711_SCK_PIN, 0);

    vTaskDelay(pdMS_TO_TICKS(500));

    // Dynamic Tare Baseline
    int64_t tare_acc = 0;
    for (int i = 0; i < TARE_SAMPLES; i++) {
        tare_acc += hx711_read();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    int32_t tare_offset = (int32_t)(tare_acc / TARE_SAMPLES);
    ESP_LOGI(TAG, "Calibration Locked: Baseline Offset = %ld", tare_offset);

    float filtered_net = 0.0f;

    while (1) {
        int32_t raw = hx711_read();
        bool fault = false;

        // Plausibility Check (Open wire / saturation rail)
        if (raw < -8000000 || raw > 8000000) {
            fault = true;
        }

        int32_t net = raw - tare_offset;
        filtered_net = (FILTER_ALPHA * (float)net) + ((1.0f - FILTER_ALPHA) * filtered_net);

        float active_force = filtered_net - DEADBAND_COUNTS;
        if (active_force < 0.0f) active_force = 0.0f;

        int pct = 0;
        if (!fault) {
            pct = (int)((active_force / (float)(MAX_STROKE_COUNTS - DEADBAND_COUNTS)) * 100.0f);
            if (pct > 100) pct = 100;
            if (pct < 0) pct = 0;
        } else {
            pct = 0; // Safe state
        }

        if (xSemaphoreTake(g_telemetry_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_telemetry.raw_adc = raw;
            g_telemetry.net_counts = (int32_t)filtered_net;
            g_telemetry.brake_pct = pct;
            g_telemetry.fault_active = fault;
            xSemaphoreGive(g_telemetry_mutex);
        }

        ESP_LOGI(TAG, "RAW: %ld | NET: %ld | BRAKE: %d%%%s",
                 raw, (int32_t)filtered_net, pct, fault ? " [FAULTSAFE]" : "");

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// --- Task 2: Display Rendering (Core 0, 10 Hz) ---
void display_task(void *pvParameters) {
    ssd1306_init();

    while (1) {
        bbw_telemetry_t local_state = {0};
        if (xSemaphoreTake(g_telemetry_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            local_state = g_telemetry;
            xSemaphoreGive(g_telemetry_mutex);
        }

        oled_draw_gauge(local_state.brake_pct, local_state.fault_active);
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz refresh prevents bus flooding
    }
}

void app_main(void) {
    g_telemetry_mutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(bbw_control_task, "BBW_Control", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(display_task,     "Display_Task", 4096, NULL, 1, NULL, 0);
}