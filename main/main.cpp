#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include <lvgl.h>
#include "lv_conf.h"
#include "demos/lv_demos.h"

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

// Portrait
#define TFT_WIDTH   320
#define TFT_HEIGHT  480

#define LV_TICK_PERIOD_MS           10
#define LV_TASK_HANDLER_PERIOD_MS   10


using namespace lgfx::v1;

/*** Setup screen resolution for LVGL ***/
static constexpr uint16_t screenWidth = TFT_HEIGHT;
static constexpr uint16_t screenHeight = TFT_WIDTH;
static SemaphoreHandle_t xGuiSemaphore = NULL;

static lv_disp_draw_buf_t draw_buf;

static lv_disp_t *disp;
static lv_theme_t *theme_current;
static lv_color_t bg_theme_color;

static LGFX lcd; // declare display variable

static int64_t tick_prev = 0;
static int64_t tick_duration_min_us = 0;
static int64_t tick_duration_max_us = 0;
static int64_t tick_counter = 0;

static const char *TAG = "main";

extern "C" { void app_main(); }

uint32_t lv_tick_cb_prio = 0;


/* Setting up tick task for lvgl */
static void lv_tick_cb(void*)
{
    lv_tick_cb_prio = uxTaskPriorityGet(NULL);
//    lv_tick_inc(LV_TICK_PERIOD_MS);
    const int64_t tick = esp_timer_get_time();
    if (tick_counter == 1) {
        tick_duration_min_us = tick_duration_max_us = tick - tick_prev;
    } else if (tick_counter > 1) {
        const int64_t duration_us = tick - tick_prev;
        if (duration_us < tick_duration_min_us) tick_duration_min_us = duration_us;
        if (duration_us > tick_duration_max_us) tick_duration_max_us = duration_us;
    }
    tick_prev = tick;
    tick_counter++;
}

bool lvgl_acquire(TickType_t xTicksToWait)
{
    return xSemaphoreTake(xGuiSemaphore, xTicksToWait) == pdTRUE;
}

void lvgl_release(void)
{
    xSemaphoreGive(xGuiSemaphore);
}


static void lv_task_handler_cb(void*)
{
    /* Try to take the semaphore, call lvgl related function on success */
    if (lvgl_acquire(pdMS_TO_TICKS(10))) {
        lv_task_handler();
        lvgl_release();
    }
}


static void lv_set_global_buffer() {
    static const uint32_t lv_bufsize = ((uint32_t) screenWidth) * 20;
    static EXT_RAM_BSS_ATTR lv_color_t buf[lv_bufsize];
    static EXT_RAM_BSS_ATTR lv_color_t buf2[lv_bufsize];

    lv_disp_draw_buf_init(&draw_buf, buf, buf2, lv_bufsize);
}


static void bsp_start_timer(esp_timer_handle_t *timer,
        const char *name,
        esp_timer_cb_t callback,
        void *arg,
        uint64_t period_ms) {
    const esp_timer_create_args_t timer_args = {
            .callback = callback,
            .arg = arg,
            .name = name,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(*timer, period_ms * 1000LL));
    ESP_LOGI(TAG, "%s timer started", name);
}


// Display callback to flush the buffer to screen
void display_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    lcd.startWrite();
    lcd.setAddrWindow(area->x1, area->y1, w, h);
    lcd.pushPixels((uint16_t *)&color_p->full, w * h, true);
    lcd.endWrite();

    lv_disp_flush_ready(disp);
}


// Touchpad callback to read the touchpad
static void touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    uint16_t touchX, touchY;
    bool touched = lcd.getTouch(&touchX, &touchY);

    if (!touched)
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        // Set the coordinates
        data->point.x = touchX;
        data->point.y = touchY;
    }
}


static void lv_tick_timer_setup(void*) {
    // called in the highest priority task
    esp_timer_handle_t lv_tick_timer;
    bsp_start_timer(&lv_tick_timer, "lv_tick", lv_tick_cb, NULL, LV_TICK_PERIOD_MS);
    vTaskDelete(NULL);  // we are done
}


static esp_err_t lv_display_start()
{
    // Setting display to landscape
    lcd.setRotation(1);

    lcd.setBrightness(128);
    lcd.setColorDepth(rgb888_3Byte);

    lcd.fillScreen(TFT_BLACK);

    /* LVGL : Setting up buffer to use for display */
    lv_set_global_buffer();

    /*** LVGL : Setup & Initialize the display device driver ***/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = display_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.sw_rotate = 1;
    disp = lv_disp_drv_register(&disp_drv);

    //*** LVGL : Setup & Initialize the input device driver ***
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // Setup theme
    theme_current = lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_BLUE),
                                          lv_palette_main(LV_PALETTE_RED),
                                          LV_USE_THEME_DEFAULT, /*Light or dark mode*/
                                          &lv_font_montserrat_14);

    // lv_disp_set_theme(disp, th); /*Assign the theme to the display*/
    bg_theme_color = theme_current->flags & LV_USE_THEME_DEFAULT ? lv_palette_darken(LV_PALETTE_GREY, 4) : lv_palette_lighten(LV_PALETTE_GREY, 1);

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
//    esp_timer_handle_t lv_tick_timer;
//    bsp_start_timer(&lv_tick_timer, "lv_tick", lv_tick_cb, NULL, LV_TICK_PERIOD_MS);

    xTaskCreatePinnedToCore(lv_tick_timer_setup, "lv_tick_timer_setup", 5120, NULL, 0, NULL, PRO_CPU_NUM);

    xGuiSemaphore = xSemaphoreCreateMutex();
    xSemaphoreGive(xGuiSemaphore);

//    esp_timer_handle_t lv_task_handler_timer;
//    bsp_start_timer(&lv_task_handler_timer, "lv_task_handler", lv_task_handler_cb, NULL, LV_TASK_HANDLER_PERIOD_MS);

    return ESP_OK;
}


static void lv_tick_cb_check(void*) {
    ESP_LOGI(TAG, "lv_tick_cb period min %.3f, max %.3f ms prio %lu", tick_duration_min_us / 1000.0, tick_duration_max_us / 1000.0, lv_tick_cb_prio);
}


static void lv_display_init() {
    if (!lcd.init()) {        // Initialize LovyanGFX
        ESP_LOGW(TAG, "lcd.init() failed");
        return;
    }
    ESP_LOGI(TAG, "lcd.init() OK");
    lv_init();         // Initialize lvgl
    ESP_LOGI(TAG, "lv_init OK");

    lv_display_start();

    esp_timer_handle_t timer;
    const esp_timer_create_args_t timer_args = {
            .callback = lv_tick_cb_check,
            .name = "lv_tick_cb_check",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000LL));

    lv_demo_benchmark();
    ESP_LOGI(TAG, "lv_demo_benchmark started");
}


void app_main(void)
{
    lv_display_init();

    while (1) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(LV_TASK_HANDLER_PERIOD_MS));
    }
}
