#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

// Configuración tareas 3
#define TAG "TASK_DEMO"

// Pines GPIO - Adaptados para ESP32 DevKit v1
#define LED_RED_GPIO     GPIO_NUM_2    // LED integrado
#define LED_GREEN_GPIO   GPIO_NUM_4    // Pin disponible
#define LED_BLUE_GPIO    GPIO_NUM_5    // Pin disponible  
#define BUTTON_GPIO      GPIO_NUM_18   // Pin disponible

// Tiempos (ms)
#define TASK1_DELAY      1000
#define TASK2_DELAY      500
#define TASK3_DELAY      2000

// Colas y semáforos
static QueueHandle_t xQueue1 = NULL;
static QueueHandle_t xQueue2 = NULL;
static SemaphoreHandle_t xMutex = NULL;

// Estructura para mensajes entre tareas
typedef struct {
    uint32_t counter;
    char task_name[16];
    TickType_t timestamp;
} task_message_t;

// Estructura global del sistema
typedef struct {
    uint32_t task1_counter;
    uint32_t task2_counter;
    uint32_t task3_counter;
    bool button_state;
    bool system_running;
} system_global_t;

static system_global_t g_system = {
    .task1_counter = 0,
    .task2_counter = 0,
    .task3_counter = 0,
    .button_state = false,
    .system_running = true
};

// Prototipos de funciones
void task1_led_controller(void *pvParameters);
void task2_sensor_reader(void *pvParameters);
void task3_message_processor(void *pvParameters);
void system_monitor_task(void *pvParameters);
void setup_gpio(void);
void setup_queues_semaphores(void);

// Tarea 1: Controlador de LED (Prioridad Media)
void task1_led_controller(void *pvParameters) {
    ESP_LOGI(TAG, "Tarea 1 (LED Controller) iniciada");
    
    bool led_state = false;
    uint32_t blink_pattern = 0;
    
    while (g_system.system_running) {
        // Control de LED con patrón
        led_state = !led_state;
        gpio_set_level(LED_RED_GPIO, led_state);
        
        // Cambiar patrón cada 10 ciclos
        if (++blink_pattern >= 10) {
            blink_pattern = 0;
            gpio_set_level(LED_GREEN_GPIO, !gpio_get_level(LED_GREEN_GPIO));
        }
        
        // Enviar mensaje a Tarea 3
        task_message_t msg = {
            .counter = g_system.task1_counter++,
            .timestamp = xTaskGetTickCount()
        };
        strlcpy(msg.task_name, "TASK1", sizeof(msg.task_name));
        
        if (xQueueSend(xQueue1, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Tarea 1: Cola llena, mensaje perdido");
        }
        
        ESP_LOGI(TAG, "T1: LED=%d, Counter=%"PRIu32, led_state, g_system.task1_counter);
        vTaskDelay(pdMS_TO_TICKS(TASK1_DELAY));
    }
    
    ESP_LOGI(TAG, "Tarea 1 finalizada");
    vTaskDelete(NULL);
}

// Tarea 2: Lector de Sensores (Prioridad Alta)
void task2_sensor_reader(void *pvParameters) {
    ESP_LOGI(TAG, "Tarea 2 (Sensor Reader) iniciada");
    
    bool last_button_state = false;
    uint32_t debounce_counter = 0;
    
    while (g_system.system_running) {
        // Leer botón con anti-rebote
        bool current_button = gpio_get_level(BUTTON_GPIO);
        
        if (current_button != last_button_state) {
            debounce_counter++;
            if (debounce_counter >= 3) { // 150ms de debounce
                g_system.button_state = current_button;
                last_button_state = current_button;
                debounce_counter = 0;
                
                // Enviar evento de botón
                task_message_t msg = {
                    .counter = g_system.task2_counter++,
                    .timestamp = xTaskGetTickCount()
                };
                strlcpy(msg.task_name, "TASK2_BTN", sizeof(msg.task_name));
                
                if (xQueueSend(xQueue2, &msg, pdMS_TO_TICKS(50)) == pdTRUE) {
                    ESP_LOGI(TAG, "T2: Botón %s", current_button ? "PRESIONADO" : "LIBERADO");
                }
            }
        } else {
            debounce_counter = 0;
        }
        
        // Control LED azul según estado del botón
        gpio_set_level(LED_BLUE_GPIO, g_system.button_state);
        
        vTaskDelay(pdMS_TO_TICKS(TASK2_DELAY));
    }
    
    ESP_LOGI(TAG, "Tarea 2 finalizada");
    vTaskDelete(NULL);
}

// Tarea 3: Procesador de Mensajes (Prioridad Baja)
void task3_message_processor(void *pvParameters) {
    ESP_LOGI(TAG, "Tarea 3 (Message Processor) iniciada");
    
    task_message_t received_msg;
    TickType_t last_timestamp = 0;
    uint32_t messages_processed = 0;
    
    while (g_system.system_running) {
        // Procesar mensajes de la cola 1 (Tarea 1)
        if (xQueueReceive(xQueue1, &received_msg, pdMS_TO_TICKS(500)) == pdTRUE) {
            if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                TickType_t delay = received_msg.timestamp - last_timestamp;
                last_timestamp = received_msg.timestamp;
                
                ESP_LOGI(TAG, "T3: Msg from %s, Count=%"PRIu32", Delay=%"PRIu32" ticks",
                        received_msg.task_name, received_msg.counter, delay);
                
                messages_processed++;
                g_system.task3_counter = messages_processed;
                xSemaphoreGive(xMutex);
            }
        }
        
        // Procesar mensajes de la cola 2 (Tarea 2 - Botón)
        if (xQueueReceive(xQueue2, &received_msg, 0) == pdTRUE) {
            ESP_LOGW(TAG, "T3: EVENTO BOTÓN - Counter=%"PRIu32, received_msg.counter);
        }
        
        // Estadísticas cada 10 mensajes
        if (messages_processed % 10 == 0 && messages_processed > 0) {
            ESP_LOGI(TAG, "T3: Estadísticas - Mensajes procesados: %"PRIu32, messages_processed);
        }
        
        vTaskDelay(pdMS_TO_TICKS(TASK3_DELAY));
    }
    
    ESP_LOGI(TAG, "Tarea 3 finalizada. Total mensajes: %"PRIu32, messages_processed);
    vTaskDelete(NULL);
}

// Tarea de monitoreo del sistema
void system_monitor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Tarea Monitor del Sistema iniciada");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    UBaseType_t high_watermark;
    
    while (1) {
        // Monitorear uso de memoria de las tareas
        ESP_LOGI(TAG, "=== MONITOR DEL SISTEMA ===");
        ESP_LOGI(TAG, "Contadores - T1:%"PRIu32" T2:%"PRIu32" T3:%"PRIu32,
                g_system.task1_counter, g_system.task2_counter, g_system.task3_counter);
        ESP_LOGI(TAG, "Estado botón: %s", g_system.button_state ? "ON" : "OFF");
        
        // Mostrar watermark de las tareas
        high_watermark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG, "Monitor stack watermark: %"PRIu32, high_watermark);
        
        // Verificar colas
        ESP_LOGI(TAG, "Cola1 msgs: %"PRIu32, uxQueueMessagesWaiting(xQueue1));
        ESP_LOGI(TAG, "Cola2 msgs: %"PRIu32, uxQueueMessagesWaiting(xQueue2));
        
        ESP_LOGI(TAG, "=== FIN MONITOR ===");
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10000)); // Cada 10 segundos
    }
}

// Configuración de GPIO
void setup_gpio(void) {
    // Configurar LEDs como salidas
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_RED_GPIO) | (1ULL << LED_GREEN_GPIO) | (1ULL << LED_BLUE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Configurar botón como entrada con pull-up
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);
    
    // Estado inicial de LEDs
    gpio_set_level(LED_RED_GPIO, 0);
    gpio_set_level(LED_GREEN_GPIO, 0);
    gpio_set_level(LED_BLUE_GPIO, 0);
    
    ESP_LOGI(TAG, "GPIO configurado correctamente");
}

// Configuración de colas y semáforos
void setup_queues_semaphores(void) {
    // Crear colas para comunicación entre tareas
    xQueue1 = xQueueCreate(10, sizeof(task_message_t));
    xQueue2 = xQueueCreate(5, sizeof(task_message_t));
    
    // Crear mutex para acceso a recursos compartidos
    xMutex = xSemaphoreCreateMutex();
    
    if (xQueue1 == NULL || xQueue2 == NULL || xMutex == NULL) {
        ESP_LOGE(TAG, "Error creando colas/semáforos");
        return;
    }
    
    ESP_LOGI(TAG, "Colas y semáforos creados exitosamente");
}

void app_main(void) {
    // Inicializar NVS (necesario para WiFi y otros componentes)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "=== INICIANDO SISTEMA CON 3 TAREAS FreeRTOS ===");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    // Inicializar hardware
    setup_gpio();
    setup_queues_semaphores();
    
    // Crear las 3 tareas principales
    xTaskCreate(
        task1_led_controller,    // Función de la tarea
        "LED_Controller",        // Nombre de la tarea
        4096,                   // Tamaño del stack
        NULL,                   // Parámetros
        2,                      // Prioridad (Media)
        NULL                    // Handle de la tarea
    );
    
    xTaskCreate(
        task2_sensor_reader,
        "Sensor_Reader",
        4096,
        NULL,
        3,                      // Prioridad más alta
        NULL
    );
    
    xTaskCreate(
        task3_message_processor,
        "Message_Processor",
        4096,
        NULL,
        1,                      // Prioridad más baja
        NULL
    );
    
    // Tarea adicional de monitoreo
    xTaskCreate(
        system_monitor_task,
        "System_Monitor",
        4096,
        NULL,
        1,
        NULL
    );
    
    ESP_LOGI(TAG, "Todas las tareas creadas. Sistema operativo.");
    ESP_LOGI(TAG, "Configuración:");
    ESP_LOGI(TAG, "- LED Rojo (GPIO %d): Parpadeo 1s", LED_RED_GPIO);
    ESP_LOGI(TAG, "- LED Verde (GPIO %d): Cambio cada 10s", LED_GREEN_GPIO);
    ESP_LOGI(TAG, "- LED Azul (GPIO %d): Controlado por botón", LED_BLUE_GPIO);
    ESP_LOGI(TAG, "- Botón (GPIO %d): Pull-up interno", BUTTON_GPIO);
    
    // Tarea principal puede finalizar - FreeRTOS se encarga del scheduling
    vTaskDelete(NULL);
}