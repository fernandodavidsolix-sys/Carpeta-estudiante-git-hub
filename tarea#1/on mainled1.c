#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define pin_led 2

void app_main(void)
{
    gpio_set_direction(pin_led, GPIO_MODE_OUTPUT);
    printf("Iniciando ....\n");

    static uint8_t estado_salida = 0;

    while(true)
    { 
        
        estado_salida = !estado_salida;

        if(estado_salida)
        {
            printf("_ON\n");
        }
        else
        {
            printf("_OFF\n");
        }

        gpio_set_level(pin_led, estado_salida);
        vTaskDelay(1000/portTICK_PERIOD_MS);

    }
    
}
