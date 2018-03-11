/* HTTP GET Example using plain POSIX sockets

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "offline-live1.services.u-blox.com"
#define WEB_PORT 80
#define WEB_URL "http://offline-live1.services.u-blox.com/GetOfflineData.ashx?token=Se2UkiiyM0WL6O4jLi5O2w;gnss=gps;period=5;resolution=1"

extern const char *TAG;

static const char *REQUEST = "GET " WEB_URL " HTTP/1.0\r\n"
    "Host: "WEB_SERVER"\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "\r\n";

extern void gps_handle_ubx_message(uint8_t *buf, size_t len);


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}


/* static void http_get_write_file(uint8_t *buf, size_t len) { */
/*     static FILE *fp = NULL; */
/*     size_t nwritten = 0; */
/*     const char *fname = "/spiffs/assistnow.ubx"; */
    
/*     if (fp == NULL) { */
/* 	fp = fopen(fname, "w"); */
/*     } */
/*     if (fp != NULL) { */
/* 	nwritten = fwrite(buf, 1, len, fp); */
/* 	ESP_LOGI(TAG,"Wrote %d bytes to file: %s", nwritten, fname);  */
/* 	fflush(fp); */
/*     } */
/*     return; */
/* } */

static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    uint8_t recv_buf[1024];

    /* Wait for the callback to set the CONNECTED_BIT in the
       event group.
    */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
			false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");

    int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

    if(err != 0 || res == NULL) {
	ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    /* Code to print the resolved IP.

       Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
    addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
    ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

    s = socket(res->ai_family, res->ai_socktype, 0);
    if(s < 0) {
	ESP_LOGE(TAG, "... Failed to allocate socket.");
	freeaddrinfo(res);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "... allocated socket");

    if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
	ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
	close(s);
	freeaddrinfo(res);
	vTaskDelay(4000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "... connected");
    freeaddrinfo(res);

    if (write(s, REQUEST, strlen(REQUEST)) < 0) {
	ESP_LOGE(TAG, "... socket send failed");
	close(s);
	vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "... socket send success");

    struct timeval receiving_timeout;
    receiving_timeout.tv_sec = 5;
    receiving_timeout.tv_usec = 0;
    if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
		   sizeof(receiving_timeout)) < 0) {
	ESP_LOGE(TAG, "... failed to set socket receiving timeout");
	close(s);
	vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "... set socket receiving timeout success");

    /* Read HTTP response, which will contain http headers, then UBX messages (ephemeris data) */
    uint16_t msglen;
    bool lastheader = false;
    bool firstheader = true;
    unsigned i;
    int status = 0;
    const char *http = "HTTP";

    /* to read the http headers, let's be crude and read one char at a time until we see a CR/LF */
    while (!lastheader) {
	bzero(recv_buf, sizeof(recv_buf));
	for (i = 0; i < sizeof recv_buf; i++) {
	    if ((r = read(s, &recv_buf[i], 1)) <= 0)
		break;
	    
	    if ((i > 0) && (recv_buf[i] == 0x0a) && (recv_buf[i-1] == 0x0d)) {
		/* we've received the end of a header line */
		/* the first line must, by RFC7230, comprise "HTTP/<digit>.<digit> <status code> <status text> */
		if (firstheader && (memcmp(http, recv_buf, strlen(http)) == 0)) {
		    const char *p;
		    for (p = (const char *)recv_buf; *p != ' '; p++)
			;
		    status = atoi(p);
		    firstheader = false;
		    ESP_LOGI(TAG, "HTTP status: %d", status);
		}
		if (i == 1) {
		    /* this header must be just CR LF, it's the last one */
		    lastheader = true;
		}
		ESP_LOGI(TAG, "HTTP header: %s", recv_buf);
		break;
	    }
	}
    }

    /* We've consumed the http headers, now read the UBX data, if status was 200 */
    if (status == 200) {

	do {
	    /* make use of our knowledge of the generic format of UBX messages */
	    r = read(s, recv_buf, 6);
	    if (r == 6) {
		msglen = (recv_buf[5]<<8) + recv_buf[4];
		// ESP_LOGI(TAG, "calling socket read(0x%08x, 0x%08x, %d)", s, (uint32_t)&recv_buf[6], msglen + 2);
		r += read(s, &recv_buf[6], msglen + 2); /* don't forget to read the checksum */
		ESP_LOGD(TAG, "socket read returned %d", r);
		// ESP_LOG_BUFFER_HEXDUMP(TAG, recv_buf, r, ESP_LOG_INFO);
		gps_handle_ubx_message(recv_buf, r);
	    }
	} while(r > 0);
    }

    close(s);
}

void http_get_main()
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    // xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);
    http_get_task(NULL);
    ESP_ERROR_CHECK( esp_wifi_stop() );
    ESP_ERROR_CHECK( esp_wifi_deinit() );
    
}
