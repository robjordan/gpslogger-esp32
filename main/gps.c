#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define NAV_PVT_ID (0x01+(0x07<<8))
#define NAV_PVT_VALID_DATE 0b001
#define NAV_PVT_VALID_TIME 0b010
#define GPS_BLINK_LED GPIO_NUM_27

static QueueHandle_t uart0_queue;
extern const char *TAG;

typedef struct {
  uint16_t id;
  uint16_t length;
  union {
    struct {
      uint32_t iTOW;
      uint16_t year;
      uint8_t month;
      uint8_t day;
      uint8_t hour;
      uint8_t min;
      uint8_t sec;
      uint8_t valid;
      uint32_t tAcc;
      int32_t nano;
      uint8_t fixType;
      uint8_t flags;
      uint8_t flags2;
      uint8_t numSV;
      int32_t lon;
      int32_t lat;
      int32_t height;
      int32_t hMSL;
      uint32_t hAcc;
      uint32_t vAcc;
      int32_t velN;
      int32_t velE;
      int32_t velD;
      int32_t gSpeed;
      int32_t headMot;
      uint32_t sAcc;
      uint32_t headAcc;
      uint16_t pDOP;
      uint8_t reserved1[6];
      int32_t headVeh;
      int16_t magDec;
      int16_t magAcc;
    } navPVT; 
  } payload;
  uint16_t checksum;
} gps_message;


void gps_blink(unsigned ontime, unsigned offtime, unsigned repeats) {
    while (repeats > 0) {
	gpio_set_level(GPS_BLINK_LED, 1);
	vTaskDelay(ontime / portTICK_PERIOD_MS);
	gpio_set_level(GPS_BLINK_LED, 0);
	vTaskDelay(offtime / portTICK_PERIOD_MS);
	repeats--;
    }
}

uint16_t gps_msg_checksum(uint8_t *content, uint32_t len) {
  uint8_t ckA = 0, ckB = 0;
  uint16_t checksum = 0;

  for (int i = 0; i < len; i++) {
    ckA += content[i];
    ckB += ckA;
    // Serial.printf("i: %d, ckA: 0x%02x, ckB: 0x%02x\n", i, ckA, ckB);
  }
  checksum = ckB;
  checksum <<= 8;
  checksum += ckA;
  return (checksum);
}

static FILE *gps_create_gps_file(gps_message *msg) {
    FILE *fp;
    char datestamp[] = "/spiffs/YYYYMMDD-HHMMSS.gps";
    struct tm now;		/* to be filled with GPS time */
    time_t t;			/* seconds since epoch */
    struct timeval tv;		/* required as parameter to settimeofday() */
    
    /* most likely the RTC will not have been set, so we'll end up with a wrong timestamp on the file in the directory. */
    /* so, let's set the RTC using the GPS data. */
    now.tm_sec = msg->payload.navPVT.sec; /* seconds */
    now.tm_min = msg->payload.navPVT.min; /* minutes */
    now.tm_hour = msg->payload.navPVT.hour;	/* hours */
    now.tm_mday = msg->payload.navPVT.day;	/* day of month */
    now.tm_mon = msg->payload.navPVT.month - 1;	/* month */
    now.tm_year = msg->payload.navPVT.year - 1900;	/* year */
    now.tm_isdst = 0;					/* no daylight savings in effect (because it's CUT) */
    t = mktime(&now);
    ESP_LOGI(TAG, "mktime() returned %d.", (unsigned int)t);
    tv.tv_sec = t;
    tv.tv_usec = 0;
    if (settimeofday(&tv, NULL) < 0) {
	ESP_LOGE(TAG, "Failed to settimeofday().");
    }
    
    /* now construct a filename that will indicate the time/date of the first GPS track point */
    sprintf(datestamp,
	    "/spiffs/%04d%02d%02d-%02d%02d%02d.gps",
	    msg->payload.navPVT.year,
	    msg->payload.navPVT.month,
	    msg->payload.navPVT.day,
	    msg->payload.navPVT.hour,
	    msg->payload.navPVT.min,
	    msg->payload.navPVT.sec);
    fp = fopen(datestamp, "w");
    if (fp == NULL) {
	ESP_LOGE(TAG, "Failed to open file for writing, %s", datestamp);
	return NULL;
    }
    return (fp);
}

static void gps_write_gps_msg_to_file(gps_message *msg) {
    static FILE *fp = NULL;
    size_t nwritten = 0;
    
    if (fp == NULL) {
	fp = gps_create_gps_file(msg);
    }
    if (fp != NULL) {
	/* write, in binary format, everything in the message from "year" up to, but not including, "hMSL" */
	nwritten = fwrite(&msg->payload.navPVT.year, 1, (void *)&msg->payload.navPVT.hMSL - (void *)&msg->payload.navPVT.year, fp);
	ESP_LOGI(TAG,"Wrote %d bytes to file", nwritten); 
	fflush(fp);
    }
    return;
    
}


static void gps_handle_message(uint8_t *buf, size_t len) {
    /* let's make the simplifying assumption that UBX messages won't span multiple UART events. */
    /* it may not always be true, but empirically it seems to be the case. */
    static bool fix = false;
    
    /* search for 0xb5 0x62 as the signature for the start of a UBX message */
    uint8_t *p = buf;
    while ((p != NULL) && (p < (buf + len))) {
	if ((p = memchr(p, 0xb5, (buf + len) - p)) != NULL) {
	    if (*(p+1) == 0x62) {
		/* we've found the start of a UBX message */
		gps_message *msg = (gps_message *)(p+2);
		if (msg->id != NAV_PVT_ID) {
		    /* it's not a message we can handle, log, ignore, and skip it */
		    ESP_LOGW(TAG, "Message id: 0x%02x 0x%02x not handled.\n", msg->id>>8, msg->id&0xFF);
		    
		} else if ((p + 2 + 2 + 2) > (buf + len)) {
		    /* it's too short to even contain a payload length. skip it */
		    ESP_LOGW(TAG, "Message id: 0x%02x 0x%02x too short (no payload length).\n", msg->id>>8, msg->id&0xFF);
		    
		} else if ((p + 2 + 2 + msg->length + 2) > (buf + len)) {
		    /* it's long enough for a payload length, but the message is incomplete  */
		    ESP_LOGW(TAG, "Message id: 0x%02x 0x%02x incomplete.\n", msg->id>>8, msg->id&0xFF);
		    
		} else if (msg->checksum != gps_msg_checksum(p+2, 2+2+msg->length)) {
		    /* it looks like a NAV-PVT but there's a checksum error */
		    ESP_LOGW(TAG, "Message id: 0x%02x 0x%02x checksum error: msg=0x%04x calc=0x%04x.\n",
			     msg->id>>8, msg->id&0xFF, msg->checksum, gps_msg_checksum(p+2, 2+2+msg->length));
		    ESP_LOG_BUFFER_HEXDUMP(TAG, p, 2+2+2+msg->length+2, ESP_LOG_INFO);
		    
		} else {
		    /* we have a NAV-PVT, the only message we need, and it's format is valid (saying nothing about the GPS data validity) */
		    ESP_LOGI(TAG,
			     "NAV-PVT: %02d/%02d/%02d %02d:%02d:%02d, fix: %d, fixType: %d, numSV: %d, lat, lon: %d.%07d,%d.%07d checksum: 0x%04x", 
			     msg->payload.navPVT.year, 
			     msg->payload.navPVT.month, 
			     msg->payload.navPVT.day,
			     msg->payload.navPVT.hour, 
			     msg->payload.navPVT.min, 
			     msg->payload.navPVT.sec,
			     msg->payload.navPVT.flags & 0x01,
			     msg->payload.navPVT.fixType,
			     msg->payload.navPVT.numSV,
			     (msg->payload.navPVT.lat) / 10000000, abs((msg->payload.navPVT.lat) % 10000000),
			     (msg->payload.navPVT.lon) / 10000000, abs((msg->payload.navPVT.lon) % 10000000),
			     msg->checksum);
		    /* check that BOTH the validDate and validTime flags are turned on */
		    if ((msg->payload.navPVT.valid & NAV_PVT_VALID_DATE) && (msg->payload.navPVT.valid & NAV_PVT_VALID_TIME)) {
			/* the first few messages sometimes have spurious data including date/time */
			gps_write_gps_msg_to_file(msg);
			fix = (msg->payload.navPVT.flags & 0x01);
			   
		    } else {
			ESP_LOGW(TAG, "Invalid date or time flag set: 0x%02x.\n",
				 msg->payload.navPVT.valid);
		    }
		}
		
		/* blink slow if no fix, fast if fix */
		if (fix) {
		    gps_blink(10, 10, 1);
		    fix = false; /* reset in case the next message is invalid */
		} else {
		    gps_blink(500, 500, 1);
		}
		
		/* skip to next message in buffer (if there is one) */
		p += 2 + 2 + 2 + msg->length + 2; /* signature + id + length + payload-length + checksum  */
		
		
	    }
	}
    }
    
    
    
}

static void gps_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
		    ESP_LOG_BUFFER_HEXDUMP(TAG, dtmp, event.size, ESP_LOG_INFO);
		    gps_handle_message(dtmp, event.size);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                     break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void gps_init() {
    // const char UBLOX_BAUD_RATE[] = {
    // 0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x01,0x00,0x00,
    // 0x00,0x00,0x00,0xF0,0xCA // 115200 baud
    // };
    const char UBLOX_INIT[] = {
	// Disable NMEA
	0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
	0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
	0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
	0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
	0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
	0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

	// Disable UBX
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off
	0xB5,0x62,0x06,0x01,0x08,0x00,0x05,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x15,0xD2, //ACK-ACK off

	// Enable UBX
	0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
	// 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
	// 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

	// Rate
	// 0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
	// 0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
	// 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
	0xB5,0x62,0x06,0x08,0x06,0x00,0x88,0x13,0x01,0x00,0x01,0x00,0xB1,0x49, //(0.2hz)

	// GNSS type configuration
	0xB5,0x62,0x06,0x3E,0x24,0x00,0x00,0x00,0x16,0x04,0x00,0x04,0xFF,0x00,0x01,0x00,0x00,0x01,0x01,0x01,0x03,0x00,0x01,
	0x00,0x00,0x01,0x05,0x00,0x03,0x00,0x00,0x00,0x00,0x01,0x06,0x08,0xFF,0x00,0x00,0x00,0x00,0x01,0xA5,0x39,

	// Power Save mode
	0xB5,0x62,0x06,0x11,0x02,0x00,0x08,0x01,0x22,0x92
    };

    // uart_write_bytes(UART_NUM, UBLOX_BAUD_RATE, sizeof UBLOX_BAUD_RATE);
    // ESP_ERROR_CHECK(uart_flush(UART_NUM));
    // vTaskDelay(1000 / portTICK_RATE_MS); /* allow time for acks to come in */
    // ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM, 115200));
    uint32_t baud_rate;
    uart_get_baudrate(UART_NUM, &baud_rate);
    ESP_LOGI(TAG, "baud rate: %d", baud_rate);
    uart_write_bytes(UART_NUM, UBLOX_INIT, sizeof UBLOX_INIT);
    return;
}

void gps_main()
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // Set UART pins 
    uart_set_pin(UART_NUM, 26, 25, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Install UART driver, and get the queue.
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    gps_init();		/* specify what messages we want / don't want */

    // Setup a GPIO to blink LED when a valid GPS reading is received
    gpio_pad_select_gpio(GPS_BLINK_LED);
    ESP_ERROR_CHECK(gpio_set_direction(GPS_BLINK_LED, GPIO_MODE_OUTPUT));

    //Create a task to handler UART event from ISR
    xTaskCreate(gps_event_task, "gps_event_task", 2048, NULL, 12, NULL);
}
