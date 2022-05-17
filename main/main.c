/* Mesh Internal Communication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "UIApp.h"

/*******************************************************
 *                Variable Definitions
 *******************************************************/
static const char *TAG = "main";

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    // xTaskCreate(LcdInit, "lcd", 4 * 1024,NULL, 15, NULL);
   /*void GSL2038Init();
   GSL2038Init();
   void getCord();
   while(1)
    getCord();*/

    UIInit();
    ESP_LOGI(TAG, "Free heap, current: %d, minimum: %d", esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
}
