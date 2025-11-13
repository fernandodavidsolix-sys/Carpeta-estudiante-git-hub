//PROYECTO Fernando SOLIS REYES  20175688

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"

#define TAG "portonproyect_fsm"

#define MOTOR_ARR GPIO_NUM_5
#define MOTOR_CIE GPIO_NUM_18
#define BUZZER     GPIO_NUM_27
#define LAMP       GPIO_NUM_26
#define BTN_ARR    GPIO_NUM_33
#define BTN_CIE    GPIO_NUM_32
#define BTN_STOP   GPIO_NUM_12
#define FIN_ARR    GPIO_NUM_17
#define FIN_CIE    GPIO_NUM_14
#define LDR_PIN    GPIO_NUM_13

#define NVS_NAMESPACE "wifi_cfg"
#define DEF_WIFI_SSID "Wall_e"
#define DEF_WIFI_PASS "allmight"
#define DEF_MQTT_SERVER "mqtt://test.mosquitto.org:1883"
#define DEF_MQTT_CMD "itla_fernan/cmd"
#define DEF_MQTT_STATE "itla_fernan/estado"

typedef struct {
    char ssid[32];
    char pass[64];
    char mqtt_server[128];
    char mqtt_cmd[64];
    char mqtt_state[64];
} app_config_t;

static app_config_t cfg_global;

#define EST_INIT  0
#define EST_WAIT  1
#define EST_CERN  2
#define EST_CER   3
#define EST_ABIN  4
#define EST_ABI   5
#define EST_ERR   6
#define EST_EMER  7
#define EST_STOP  8

volatile int EST_ACT = EST_INIT;
volatile int EST_SIG = EST_INIT;
volatile int EST_ANT = EST_INIT;

static const int FSM_PERIOD_MS = 100;
static const int MOV_TIMEOUT_MS = 20000;
static const int BLINK_FAST_MS = 250;
static const int BLINK_SLOW_MS = 500;
static const int ERR_BUZZER_BLINK_MS = 500;
static const int EMER_RESET_MS = 5000;

static TimerHandle_t fsm_timer = NULL;
static TimerHandle_t mov_timer = NULL;
static TimerHandle_t blink_timer = NULL;
static TimerHandle_t reset_timer = NULL;

volatile struct INPUT {
    unsigned int LSA : 1;
    unsigned int LSC : 1;
    unsigned int CA  : 1;
    unsigned int CC  : 1;
    unsigned int FC  : 1;
    unsigned int BTN_A : 1;
    unsigned int BTN_C : 1;
    unsigned int BTN_S : 1;
} in;

volatile struct OUTPUT {
    unsigned int MC : 1;
    unsigned int MA : 1;
    unsigned int BZ : 1;
    unsigned int LAMP : 1;
} out;

static EventGroupHandle_t wifi_events;
#define WIFI_OK   BIT0
#define WIFI_FAIL BIT1

static esp_mqtt_client_handle_t mqtt_client = NULL;
static httpd_handle_t web_server = NULL;

int Func_INIT();
int Func_WAIT();
int Func_CERN();
int Func_CER();
int Func_ABIN();
int Func_ABI();
int Func_ERR();
int Func_EMER();
int Func_STOP();

static void actualizar_io_hw(void);
static bool leer_btn_hw(gpio_num_t p);
static bool leer_fin_hw(gpio_num_t p);
static void gpio_setup_hw(void);
static void load_config(app_config_t *cfg);
static void save_config(const app_config_t *cfg);
static void start_mqtt(void);
static void mqtt_cb(void *handler_args, esp_event_base_t base, int32_t id, void *d);
static httpd_handle_t start_webserver(void);

static const char *estado_to_str(int e)
{
    switch (e) {
        case EST_INIT: return "inicio";
        case EST_WAIT: return "espera";
        case EST_CERN: return "cerrando";
        case EST_CER:  return "cerrado";
        case EST_ABIN: return "abriendo";
        case EST_ABI:  return "abierto";
        case EST_ERR:  return "error";
        case EST_EMER: return "emergencia";
        case EST_STOP: return "detenido";
        default: return "desconocido";
    }
}

static void mqtt_publicar_estado(int estado)
{
    if (mqtt_client) {
        const char *s = estado_to_str(estado);
        esp_mqtt_client_publish(mqtt_client, cfg_global.mqtt_state, s, 0, 1, 0);
        ESP_LOGI(TAG, "MQTT publish estado: %s", s);
    }
}

/* NVS */
static void save_config(const app_config_t *cfg)
{
    nvs_handle_t nvsh;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvsh) == ESP_OK) {
        nvs_set_str(nvsh, "ssid", cfg->ssid);
        nvs_set_str(nvsh, "pass", cfg->pass);
        nvs_set_str(nvsh, "mqtt_server", cfg->mqtt_server);
        nvs_set_str(nvsh, "mqtt_cmd", cfg->mqtt_cmd);
        nvs_set_str(nvsh, "mqtt_state", cfg->mqtt_state);
        nvs_commit(nvsh);
        nvs_close(nvsh);
        ESP_LOGI(TAG, "Configuración guardada en NVS");
    } else {
        ESP_LOGE(TAG, "EMITIR  Error guardando NVS");
    }
}

static void load_config(app_config_t *cfg)
{
    nvs_handle_t nvsh;
    bool found = true;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvsh) == ESP_OK) {
        size_t len;
        len = sizeof(cfg->ssid);
        if (nvs_get_str(nvsh, "ssid", cfg->ssid, &len) != ESP_OK) found = false;
        len = sizeof(cfg->pass);
        if (nvs_get_str(nvsh, "pass", cfg->pass, &len) != ESP_OK) found = false;
        len = sizeof(cfg->mqtt_server);
        if (nvs_get_str(nvsh, "mqtt_server", cfg->mqtt_server, &len) != ESP_OK) found = false;
        len = sizeof(cfg->mqtt_cmd);
        if (nvs_get_str(nvsh, "mqtt_cmd", cfg->mqtt_cmd, &len) != ESP_OK) found = false;
        len = sizeof(cfg->mqtt_state);
        if (nvs_get_str(nvsh, "mqtt_state", cfg->mqtt_state, &len) != ESP_OK) found = false;
        nvs_close(nvsh);
    } else {
        found = false;
    }
    if (!found) {
        strncpy(cfg->ssid, DEF_WIFI_SSID, sizeof(cfg->ssid));
        strncpy(cfg->pass, DEF_WIFI_PASS, sizeof(cfg->pass));
        strncpy(cfg->mqtt_server, DEF_MQTT_SERVER, sizeof(cfg->mqtt_server));
        strncpy(cfg->mqtt_cmd, DEF_MQTT_CMD, sizeof(cfg->mqtt_cmd));
        strncpy(cfg->mqtt_state, DEF_MQTT_STATE, sizeof(cfg->mqtt_state));
        save_config(cfg);
        ESP_LOGW(TAG, "No NVS - usando por defecto");
    } else {
        ESP_LOGI(TAG, "Configuración cargada desde NVS");
    }
}

/* GPIO */
static void gpio_setup_hw(void)
{
    gpio_config_t cfg = {0};

    cfg.intr_type = GPIO_INTR_DISABLE;
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pin_bit_mask = ((uint64_t)1 << MOTOR_ARR) | ((uint64_t)1 << MOTOR_CIE) | ((uint64_t)1 << BUZZER) | ((uint64_t)1 << LAMP);
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&cfg);

    cfg.mode = GPIO_MODE_INPUT;
    cfg.pin_bit_mask = ((uint64_t)1 << BTN_ARR) | ((uint64_t)1 << BTN_CIE) | ((uint64_t)1 << BTN_STOP) | ((uint64_t)1 << LDR_PIN);
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&cfg);

    cfg.pin_bit_mask = ((uint64_t)1 << FIN_ARR) | ((uint64_t)1 << FIN_CIE);
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&cfg);

    out.MA = out.MC = out.BZ = out.LAMP = 0;
    actualizar_io_hw();
}

static void actualizar_io_hw(void)
{
    gpio_set_level(MOTOR_ARR, out.MA);
    gpio_set_level(MOTOR_CIE, out.MC);
    gpio_set_level(BUZZER, out.BZ);
    gpio_set_level(LAMP, out.LAMP);
}

static bool leer_btn_hw(gpio_num_t p) { return gpio_get_level(p) == 0; }
static bool leer_fin_hw(gpio_num_t p) { return gpio_get_level(p) == 0; }

/* TIMERS */
static void vTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    in.BTN_A = leer_btn_hw(BTN_ARR);
    in.BTN_C = leer_btn_hw(BTN_CIE);
    in.BTN_S = leer_btn_hw(BTN_STOP);

    in.LSA = leer_fin_hw(FIN_ARR);
    in.LSC = leer_fin_hw(FIN_CIE);
    in.FC  = leer_btn_hw(LDR_PIN);

    /* Si ambos limit switch finales activos -> error */
    if (in.LSA && in.LSC && EST_SIG != EST_ERR) {
        EST_SIG = EST_ERR;
    }

    actualizar_io_hw();
}

static void mov_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (EST_SIG == EST_ABIN || EST_SIG == EST_CERN) {
        EST_SIG = EST_ERR;
    }
}

static uint8_t blink_mode = 0;

static void blink_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (blink_mode == 1) {
        out.LAMP = !out.LAMP;
    } else if (blink_mode == 2) {
        out.BZ = !out.BZ;
    }
    actualizar_io_hw();
}

static void reset_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    EST_SIG = EST_WAIT;
}

/* MQTT */
static void start_mqtt(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = cfg_global.mqtt_server,
        .session.keepalive = 60
    };
    mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_cb, NULL);
    esp_mqtt_client_start(mqtt_client);
}

static void mqtt_cb(void *handler_args, esp_event_base_t base, int32_t id, void *d)
{
    esp_mqtt_event_handle_t e = d;
    mqtt_client = e->client;
    switch ((esp_mqtt_event_id_t)id) {
        case MQTT_EVENT_CONNECTED:
            esp_mqtt_client_subscribe(mqtt_client, cfg_global.mqtt_cmd, 0);
            break;
        case MQTT_EVENT_DATA:
        {
            char msg[128] = {0};
            int l = (e->data_len < (int)sizeof(msg) - 1) ? e->data_len : (int)sizeof(msg) - 1;
            strncpy(msg, e->data, l);
            msg[l] = '\0';
            char *p = msg;
            while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++;
            if (p != msg) memmove(msg, p, strlen(p) + 1);
            for (int i = 0; msg[i]; i++) if (msg[i] >= 'A' && msg[i] <= 'Z') msg[i] += 'a' - 'A';

            if (EST_SIG == EST_ERR) {
                if (strcmp(msg, "emergencia") == 0) {
                    EST_SIG = EST_WAIT; /* emergencia = reset -> WAIT */
                }
                break;
            }

            if (strcmp(msg, "abrir") == 0) {
                if (!(EST_SIG == EST_ABIN || EST_SIG == EST_ABI)) {
                    EST_ANT = EST_SIG;
                    EST_SIG = EST_ABIN;
                    mqtt_publicar_estado(EST_ABIN);
                }
            } else if (strcmp(msg, "cerrar") == 0) {
                if (!(EST_SIG == EST_CERN || EST_SIG == EST_CER)) {
                    EST_ANT = EST_SIG;
                    EST_SIG = EST_CERN;
                    mqtt_publicar_estado(EST_CERN);
                }
            } else if (strcmp(msg, "stop") == 0) {
                if (EST_SIG == EST_STOP) {
                    if (EST_ANT == EST_ABIN) EST_SIG = EST_CERN;
                    else if (EST_ANT == EST_CERN) EST_SIG = EST_ABIN;
                } else if (EST_SIG == EST_ABIN || EST_SIG == EST_CERN) {
                    EST_ANT = EST_SIG;
                    EST_SIG = EST_STOP;
                }
            } else if (strcmp(msg, "reset") == 0) {
                EST_SIG = EST_INIT;
            } else if (strcmp(msg, "emergencia") == 0) {
                EST_SIG = EST_EMER;
                mqtt_publicar_estado(EST_EMER);
            }
        }
        break;
        default: break;
    }
}

/* WEB */
static const char *html_page =
"<!DOCTYPE html><html lang='es'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Control Portón</title><style>html,body{height:100%;margin:0}body{font-family:Arial;background:#111;color:#fff;display:flex;align-items:center;justify-content:center} .card{background:#0f1720;padding:18px;border-radius:12px;max-width:420px;width:100%;box-sizing:border-box;text-align:center} .btn{background:#0b63ff;border:none;color:#fff;padding:12px 16px;border-radius:8px;margin:6px;cursor:pointer;min-width:100px} .state{margin-top:12px;padding:10px;background:#081122;border-radius:8px;text-align:center}</style></head><body>"
"<div class='card'><h2>Control Portón</h2>"
"<div><button class='btn' onclick=\"enviar('/abrir')\">Abrir</button><button class='btn' onclick=\"enviar('/cerrar')\">Cerrar</button></div>"
"<div><button class='btn' onclick=\"enviar('/stop')\">Stop</button><button class='btn' onclick=\"enviar('/emergencia')\">Emergencia</button></div>"
"<div class='state'>Estado: <span id='estado'>--</span></div>"
"<script>async function refresh(){try{let r=await fetch('/estado'); if(r.ok){let s=await r.text(); document.getElementById('estado').innerText=s;}}catch(e){} }async function enviar(u){try{await fetch(u);}catch(e){} refresh();} setInterval(refresh,1500); refresh();</script></div></body></html>";

static esp_err_t root_get(httpd_req_t *req) { httpd_resp_set_type(req, "text/html"); return httpd_resp_send(req, html_page, strlen(html_page)); }
static esp_err_t estado_get(httpd_req_t *req)
{
    const char *s = estado_to_str(EST_SIG);
    httpd_resp_sendstr(req, s);
    return ESP_OK;
}
static esp_err_t cmd_get(httpd_req_t *req)
{
    if (EST_SIG == EST_ERR) {
        if (strcmp(req->uri, "/emergencia") == 0) {
            EST_SIG = EST_WAIT; /* emergencia desde web resetea a WAIT */
        }
        httpd_resp_sendstr(req, "OK");
        return ESP_OK;
    }
    if (strcmp(req->uri, "/abrir") == 0) {
        if (!(EST_SIG == EST_ABIN || EST_SIG == EST_ABI)) { EST_ANT = EST_SIG; EST_SIG = EST_ABIN; }
    } else if (strcmp(req->uri, "/cerrar") == 0) {
        if (!(EST_SIG == EST_CERN || EST_SIG == EST_CER)) { EST_ANT = EST_SIG; EST_SIG = EST_CERN; }
    } else if (strcmp(req->uri, "/stop") == 0) {
        if (EST_SIG == EST_STOP) {
            if (EST_ANT == EST_ABIN) EST_SIG = EST_CERN;
            else if (EST_ANT == EST_CERN) EST_SIG = EST_ABIN;
        } else if (EST_SIG == EST_ABIN || EST_SIG == EST_CERN) {
            EST_ANT = EST_SIG;
            EST_SIG = EST_STOP;
        }
    } else if (strcmp(req->uri, "/emergencia") == 0) {
        EST_SIG = EST_EMER;
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_get, .user_ctx = NULL};
        httpd_uri_t estado = {.uri = "/estado", .method = HTTP_GET, .handler = estado_get, .user_ctx = NULL};
        httpd_uri_t cmd_abrir = {.uri = "/abrir", .method = HTTP_GET, .handler = cmd_get, .user_ctx = NULL};
        httpd_uri_t cmd_cerrar = {.uri = "/cerrar", .method = HTTP_GET, .handler = cmd_get, .user_ctx = NULL};
        httpd_uri_t cmd_stop = {.uri = "/stop", .method = HTTP_GET, .handler = cmd_get, .user_ctx = NULL};
        httpd_uri_t cmd_emerg = {.uri = "/emergencia", .method = HTTP_GET, .handler = cmd_get, .user_ctx = NULL};
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &estado);
        httpd_register_uri_handler(server, &cmd_abrir);
        httpd_register_uri_handler(server, &cmd_cerrar);
        httpd_register_uri_handler(server, &cmd_stop);
        httpd_register_uri_handler(server, &cmd_emerg);
        ESP_LOGI(TAG, "Webserver iniciado");
    } else {
        ESP_LOGE(TAG, "Fallo al iniciar webserver");
    }
    return server;
}

/* WIFI */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        static int retry = 0;
        if (retry < 6) { esp_wifi_connect(); retry++; }
        else xEventGroupSetBits(wifi_events, WIFI_FAIL);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_events, WIFI_OK);
    }
}

static void wifi_init(void)
{
    wifi_events = xEventGroupCreate();
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, cfg_global.ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, cfg_global.pass, sizeof(wifi_config.sta.password));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

/* FSM FUNTIONS */
int Func_INIT()
{
    EST_ANT = EST_ACT;
    EST_ACT = EST_INIT;

    gpio_setup_hw();

    fsm_timer = xTimerCreate("fsm", pdMS_TO_TICKS(FSM_PERIOD_MS), pdTRUE, NULL, vTimerCallback);
    mov_timer = xTimerCreate("mov", pdMS_TO_TICKS(MOV_TIMEOUT_MS), pdFALSE, NULL, mov_timer_cb);
    blink_timer = xTimerCreate("blink", pdMS_TO_TICKS(BLINK_SLOW_MS), pdTRUE, NULL, blink_timer_cb);
    reset_timer = xTimerCreate("reset", pdMS_TO_TICKS(EMER_RESET_MS), pdFALSE, NULL, reset_timer_cb);

    xTimerStart(fsm_timer, 0);

    load_config(&cfg_global);
    wifi_init();

    EventBits_t bits = xEventGroupWaitBits(wifi_events, WIFI_OK | WIFI_FAIL, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_OK) {
        start_mqtt();
        web_server = start_webserver();
    } else {
        ESP_LOGE(TAG, "No se pudo conectar a WiFi, no good");
    }

    const esp_console_config_t console_config = { .max_cmdline_args = 8, .max_cmdline_length = 256 };
    esp_console_init(&console_config);

    mqtt_publicar_estado(EST_INIT);

    return EST_CER;
}

int Func_WAIT()
{
    EST_ANT = EST_ACT;
    EST_ACT = EST_WAIT;

    xTimerStop(blink_timer, 0);
    blink_mode = 0;
    out.LAMP = 0;
    out.MA = out.MC = out.BZ = 0;
    actualizar_io_hw();

    while (1) {
        if (EST_SIG != EST_WAIT) return EST_SIG;
        /* si ambos LIMIT SWITCH finales activos en espera error */
        if (in.LSA && in.LSC) {
            EST_SIG = EST_ERR;
            return EST_SIG;
        }
        if (in.FC && in.LSC) {
            EST_ANT = EST_SIG;
            EST_SIG = EST_ABIN;
            return EST_SIG;
        }
        vTaskDelay(pdMS_TO_TICKS(FSM_PERIOD_MS));
    }
}

int Func_CERN()
{
    EST_ANT = EST_ACT;
    EST_ACT = EST_CERN;

    out.MA = 0;
    out.MC = 1;
    out.BZ = 0;
    actualizar_io_hw();

    xTimerStop(mov_timer, 0);
    xTimerChangePeriod(mov_timer, pdMS_TO_TICKS(MOV_TIMEOUT_MS), 0);
    xTimerStart(mov_timer, 0);

    blink_mode = 1;
    xTimerChangePeriod(blink_timer, pdMS_TO_TICKS(BLINK_SLOW_MS), 0);
    xTimerStart(blink_timer, 0);

    while (1) {
        if (EST_SIG != EST_CERN) { xTimerStop(mov_timer, 0); xTimerStop(blink_timer, 0); blink_mode = 0; out.LAMP = 0; actualizar_io_hw(); return EST_SIG; }
        if (in.FC) { xTimerStop(mov_timer, 0); xTimerStop(blink_timer, 0); blink_mode = 0; out.LAMP = 0; actualizar_io_hw(); EST_SIG = EST_ABIN; return EST_SIG; }
        if (in.LSC) { xTimerStop(mov_timer, 0); xTimerStop(blink_timer, 0); blink_mode = 0; out.LAMP = 0; actualizar_io_hw(); EST_SIG = EST_CER; return EST_SIG; }
        if (EST_SIG == EST_ABIN) { xTimerStop(mov_timer, 0); xTimerStop(blink_timer, 0); blink_mode = 0; out.LAMP = 0; actualizar_io_hw(); return EST_SIG; }
        if (in.LSA && in.LSC) { xTimerStop(mov_timer, 0); xTimerStop(blink_timer, 0); blink_mode = 0; out.LAMP = 0; actualizar_io_hw(); EST_SIG = EST_ERR; return EST_SIG; }
        vTaskDelay(pdMS_TO_TICKS(FSM_PERIOD_MS));
    }
}

int Func_CER()
{
    EST_ANT = EST_ACT;
    EST_ACT = EST_CER;

    xTimerStop(blink_timer, 0);
    blink_mode = 0;
    out.LAMP = 0;

    out.MA = out.MC = out.BZ = 0;
    actualizar_io_hw();

    if (in.FC) {
        EST_ANT = EST_SIG;
        EST_SIG = EST_ABIN;
        return EST_SIG;
    }

    mqtt_publicar_estado(EST_CER);

    return EST_WAIT;
}

int Func_ABIN()
{
    EST_ANT = EST_ACT;
    EST_ACT = EST_ABIN;

    out.MA = 1;
    out.MC = 0;
    out.BZ = 0;
    actualizar_io_hw();

    xTimerStop(mov_timer, 0);
    xTimerChangePeriod(mov_timer, pdMS_TO_TICKS(MOV_TIMEOUT_MS), 0);
    xTimerStart(mov_timer, 0);

    blink_mode = 1;
    xTimerChangePeriod(blink_timer, pdMS_TO_TICKS(BLINK_FAST_MS), 0);
    xTimerStart(blink_timer, 0);

    while (1) {
        if (EST_SIG != EST_ABIN) { xTimerStop(mov_timer, 0); xTimerStop(blink_timer, 0); blink_mode = 0; out.LAMP = 0; actualizar_io_hw(); return EST_SIG; }
        if (in.LSA) { xTimerStop(mov_timer, 0); xTimerStop(blink_timer, 0); blink_mode = 0; out.LAMP = 0; actualizar_io_hw(); EST_SIG = EST_ABI; return EST_SIG; }
        if (in.FC)  { xTimerStop(mov_timer, 0); xTimerStop(blink_timer, 0); blink_mode = 0; out.LAMP = 0; actualizar_io_hw(); EST_SIG = EST_EMER; return EST_SIG; }
        if (EST_SIG == EST_CERN) { xTimerStop(mov_timer, 0); xTimerStop(blink_timer, 0); blink_mode = 0; out.LAMP = 0; actualizar_io_hw(); return EST_SIG; }
        if (in.LSA && in.LSC) { xTimerStop(mov_timer, 0); xTimerStop(blink_timer, 0); blink_mode = 0; out.LAMP = 0; actualizar_io_hw(); EST_SIG = EST_ERR; return EST_SIG; }
        vTaskDelay(pdMS_TO_TICKS(FSM_PERIOD_MS));
    }
}

int Func_ABI()
{
    EST_ANT = EST_ACT;
    EST_ACT = EST_ABI;

    xTimerStop(blink_timer, 0);
    blink_mode = 0;
    out.LAMP = 1;
    out.MA = out.MC = out.BZ = 0;
    actualizar_io_hw();

    mqtt_publicar_estado(EST_ABI);

    return EST_WAIT;
}

int Func_ERR()
{
    EST_ANT = EST_ACT;
    EST_ACT = EST_ERR;

    out.MA = out.MC = 0;
    out.BZ = 0;
    blink_mode = 2;
    xTimerChangePeriod(blink_timer, pdMS_TO_TICKS(ERR_BUZZER_BLINK_MS), 0);
    xTimerStart(blink_timer, 0);
    actualizar_io_hw();

    while (1) {
        if (EST_SIG != EST_ERR) {
            xTimerStop(blink_timer, 0);
            blink_mode = 0;
            out.BZ = 0;
            out.LAMP = 0;
            actualizar_io_hw();
            return EST_SIG;
        }
        vTaskDelay(pdMS_TO_TICKS(FSM_PERIOD_MS));
    }
}

int Func_EMER()
{
    EST_ANT = EST_ACT;
    EST_ACT = EST_EMER;

    out.MA = out.MC = 0;
    out.BZ = 1;
    out.LAMP = 1;
    blink_mode = 0;
    xTimerStop(blink_timer, 0);
    actualizar_io_hw();

    xTimerStop(reset_timer, 0);
    xTimerChangePeriod(reset_timer, pdMS_TO_TICKS(EMER_RESET_MS), 0);
    xTimerStart(reset_timer, 0);

    while (1) {
        if (EST_SIG != EST_EMER) {
            xTimerStop(reset_timer, 0);
            out.BZ = 0;
            out.LAMP = 0;
            actualizar_io_hw();
            return EST_SIG;
        }
        vTaskDelay(pdMS_TO_TICKS(FSM_PERIOD_MS));
    }
}

int Func_STOP()
{
    EST_ANT = EST_ACT;
    EST_ACT = EST_STOP;

    xTimerStop(blink_timer, 0);
    blink_mode = 0;
    out.MA = out.MC = out.BZ = out.LAMP = 0;
    actualizar_io_hw();

    while (1) {
        if (EST_SIG != EST_STOP) return EST_SIG;
        vTaskDelay(pdMS_TO_TICKS(FSM_PERIOD_MS));
    }
}

void app_main(void)
{
    nvs_flash_erase();
    nvs_flash_init();
    load_config(&cfg_global);

    fsm_timer = NULL;
    mov_timer = NULL;
    blink_timer = NULL;
    reset_timer = NULL;

    while (1) {
        switch (EST_SIG) {
            case EST_INIT: EST_SIG = Func_INIT(); break;
            case EST_WAIT: EST_SIG = Func_WAIT(); break;
            case EST_CERN: EST_SIG = Func_CERN(); break;
            case EST_CER:  EST_SIG = Func_CER();  break;
            case EST_ABIN: EST_SIG = Func_ABIN(); break;
            case EST_ABI:  EST_SIG = Func_ABI();  break;
            case EST_ERR:  EST_SIG = Func_ERR();  break;
            case EST_EMER: EST_SIG = Func_EMER(); break;
            case EST_STOP: EST_SIG = Func_STOP(); break;
            default: EST_SIG = EST_INIT; break;
        }
    }
}
