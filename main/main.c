/* gpslogger-esp32
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>


const char *TAG = "gpslogger";

extern void tp_main();
extern void uart_main();
extern void spiffs_main();

void app_main() {
    spiffs_main();		/* mount the SPIFFS partition as a file system */
    tp_main();
    uart_main();
}
