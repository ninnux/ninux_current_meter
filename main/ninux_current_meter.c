/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include <u8g2.h>
#include "u8g2_esp32_hal.h"
#define PIN_SDA 4 
#define PIN_SCL 15
#define PIN_RESET 16


/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 *
 */

//#define GPIO_OUTPUT_IO_0    18
//#define GPIO_OUTPUT_IO_1    19
//#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     22 
#define GPIO_INPUT_IO_1     0
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0


#define CURRENT_REFERENCE  0.1707



static xQueueHandle gpio_evt_queue = NULL;
struct timeval now;
float value1;
float value2;


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    uint32_t oldtime=0;
    uint32_t elapsed=0;
    float elapsed_sec=0;
    int counter=0;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
	    if(io_num==0){
		counter+=1;
 		gettimeofday(&now, NULL);
 		int time = now.tv_sec;
 		int utime = now.tv_usec;
		uint32_t nowtime=(time*1000000)+utime;
		elapsed=nowtime-oldtime;
		elapsed_sec=(float) elapsed/1000000;
    		printf("passati %d usec = %f sec\n",elapsed,elapsed_sec);
    		oldtime=nowtime;
		value1=counter*CURRENT_REFERENCE;
		value2=614.4/elapsed_sec;
    		printf("corrente consumanta %fmAh\n",value1);
		printf("in media %fmA\n",value2);
		
	    }else{
            	printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
	    }
        }
    }
}

void app_main()
{
    gpio_config_t io_conf;
//    //disable interrupt
//    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
//  //set as output mode
//    io_conf.mode = GPIO_MODE_OUTPUT;
//  //bit mask of the pins that you want to set,e.g.GPIO18/19
//    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
//  //disable pull-down mode
//  io_conf.pull_down_en = 0;
//  //disable pull-up mode
//    io_conf.pull_up_en = 0;
//  //configure GPIO with the given settings
//    gpio_config(&io_conf);

    //interrupt of rising edge
    //io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    ////io_conf.pull_up_en = 0;
    ////io_conf.pull_down_en =1;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en =0;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    ////gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(GPIO_INPUT_IO_1, GPIO_INTR_NEGEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

 //   //remove isr handler for gpio number.
 //   gpio_isr_handler_remove(GPIO_INPUT_IO_0);
 //   //hook isr handler for specific gpio pin again
 //   gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    // DISPLAY INIT
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda   = PIN_SDA;
    u8g2_esp32_hal.scl  = PIN_SCL;
    u8g2_esp32_hal.reset  = PIN_RESET;
    u8g2_esp32_hal_init(u8g2_esp32_hal);
    int counter=0;
    
    u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_ssd1306_i2c_128x32_univision_f(
    	&u8g2,
    	U8G2_R0,
    	//u8x8_byte_sw_i2c,
    	u8g2_esp32_i2c_byte_cb,
    	u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
    u8x8_SetI2CAddress(&u8g2.u8x8,0x78);
    
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
    
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    u8g2_ClearBuffer(&u8g2);
    //u8g2_DrawBox(&u8g2, 0, 26, 50,6);
    //u8g2_DrawFrame(&u8g2, 0,26,100,6);
    
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
    
    char line1[20];
    char line2[20];
    // END DISPLAY INIT

    int cnt = 0;
    while(1) {
        //counter+=1;
        u8g2_ClearBuffer(&u8g2);
        sprintf(line1,"%.4f mAh",value1);
        sprintf(line2,"%.4f mA",value2);
        u8g2_DrawStr(&u8g2, 2,15,line1);
        u8g2_DrawStr(&u8g2, 2,30,line2);
        u8g2_SendBuffer(&u8g2);
        //printf("cnt: %d\n", cnt++);
        vTaskDelay(5000 / portTICK_RATE_MS);
        //gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        //gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);
    }
}

