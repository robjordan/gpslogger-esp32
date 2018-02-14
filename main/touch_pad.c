#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_pad.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"

#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_THRESH_PERCENT  (99)

static bool s_pad_activated[TOUCH_PAD_MAX];
static uint32_t s_pad_init_val[TOUCH_PAD_MAX];

extern const char *TAG;

/*
  Read values sensed at all available touch pads.
  Use 2 / 3 of read value as the threshold
  to trigger interrupt when the pad is touched.
  Note: this routine demonstrates a simple way
  to configure activation threshold for the touch pads.
  Do not touch any pads when this routine
  is running (on application start).
 */
static void tp_set_thresholds(void)
{
    uint16_t touch_value;
    //delay some time in order to make the filter work and get a initial value
    vTaskDelay(500/portTICK_PERIOD_MS);

    for (int i = 0; i<TOUCH_PAD_MAX; i++) {
        //read filtered value
        touch_pad_read_filtered(i, &touch_value);
        s_pad_init_val[i] = touch_value;
        ESP_LOGI(TAG, "test init touch val: %d\n", touch_value);
        //set interrupt threshold.
        ESP_ERROR_CHECK(touch_pad_set_thresh(i, touch_value * 2 / 3));

    }
}

/*
  Check if any of touch pads has been activated
  by reading a table updated by rtc_intr()
  If so, then print it out on a serial monitor.
  Clear related entry in the table afterwards

  In interrupt mode, the table is updated in touch ISR.

  In filter mode, we will compare the current filtered value with the initial one.
  If the current filtered value is less than 99% of the initial value, we can
  regard it as a 'touched' event.
  When calling touch_pad_init, a timer will be started to run the filter.
  This mode is designed for the situation that the pad is covered
  by a 2-or-3-mm-thick medium, usually glass or plastic.
  The difference caused by a 'touch' action could be very small, but we can still use
  filter mode to detect a 'touch' event.
 */
static void tp_read_task(void *pvParameter)
{
    static int show_message;
    while (1) {
	//filter mode, disable touch interrupt
	touch_pad_intr_disable();
	for (int i = 0; i < TOUCH_PAD_MAX; i++) {
	    uint16_t value = 0;
	    touch_pad_read_filtered(i, &value);
	    if (value < s_pad_init_val[i] * TOUCH_THRESH_PERCENT / 100) {
		ESP_LOGI(TAG, "T%d activated!", i);
		ESP_LOGI(TAG, "value: %d; init val: %d\n", value, s_pad_init_val[i]);
		vTaskDelay(200 / portTICK_PERIOD_MS);
		show_message = 1;
	    }
	}

        vTaskDelay(10 / portTICK_PERIOD_MS);

        // If no pad is touched, every couple of seconds, show a message
        // that application is running
        if (show_message++ % 500 == 0) {
            ESP_LOGI(TAG, "Waiting for any pad being touched...");
        }
    }
}

/*
  Handle an interrupt triggered when a pad is touched.
  Recognize what pad has been touched and save it in a table.
 */
static void tp_rtc_intr(void * arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    //clear interrupt
    touch_pad_clear_status();
    for (int i = 0; i < TOUCH_PAD_MAX; i++) {
        if ((pad_intr >> i) & 0x01) {
            s_pad_activated[i] = true;
        }
    }
}

/*
 * Before reading touch pad, we need to initialize the RTC IO.
 */
static void tp_touch_pad_init()
{
    for (int i = 0;i< TOUCH_PAD_MAX;i++) {
        //init RTC IO and mode for touch pad.
        touch_pad_config(i, TOUCH_THRESH_NO_USE);
    }
}

void tp_main() {
    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_LOGI(TAG, "Initializing touch pad");
    touch_pad_init();

    // Initialize and start a software filter to detect slight change of capacitance.
    touch_pad_filter_start(10);
    // Set measuring time and sleep time
    // In this case, measurement will sustain 0xffff / 8Mhz = 8.19ms
    // Meanwhile, sleep time between two measurement will be 0x1000 / 150Khz = 27.3 ms
    touch_pad_set_meas_time(0x1000, 0xffff);

    //set reference voltage for charging/discharging
    // In this case, the high reference valtage will be 2.4V - 1.5V = 0.9V
    // The low reference voltage will be 0.8V, so that the procedure of charging
    // and discharging would be very fast.
    touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V8, TOUCH_HVOLT_ATTEN_1V5);
    // Init touch pad IO
    tp_touch_pad_init();
    // Set thresh hold
    tp_set_thresholds();
    // Register touch interrupt ISR
    touch_pad_isr_register(tp_rtc_intr, NULL);

    // Start a task to show what pads have been touched
    xTaskCreate(&tp_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
}
