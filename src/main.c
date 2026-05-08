#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_task_wdt.h"
#include "esp_rom_sys.h"

/* ================= CONFIG ================= */

#define ADC_LDR     ADC_CHANNEL_7   // GPIO35
#define ADC_TEMP    ADC_CHANNEL_6   // GPIO34

#define LED_PIN     GPIO_NUM_26
#define RELAY_PIN   GPIO_NUM_25

#define IN1 GPIO_NUM_18
#define IN2 GPIO_NUM_19
#define IN3 GPIO_NUM_21
#define IN4 GPIO_NUM_22

#define UART_PORT UART_NUM_0

#define LDR_MIN 150
#define LDR_MAX 3500

#define LAMP_INVERT 0

/* ================= PWM CONFIG ================= */

#define LEDC_TIMER         LEDC_TIMER_0
#define LEDC_MODE          LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL       LEDC_CHANNEL_0
#define LEDC_DUTY_RES      LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY     500

/* ================= VARIABLES ================= */

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc_cali_handle;

volatile float Tc = 25.0f;
volatile float T  = 0.0f;

float luz = 0.0f;
float led_percent = 0.0f;

/* ================= MOTOR ================= */

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_HEAT,
    MOTOR_VENT_LOW,
    MOTOR_VENT_MED,
    MOTOR_VENT_HIGH,
} motor_state_t;

typedef struct {
    int in1;
    int in2;
    int in3;
    int in4;
} step_t;

static const step_t seq[8] = {
    {1,0,0,0},
    {1,0,1,0},
    {0,0,1,0},
    {0,1,1,0},
    {0,1,0,0},
    {0,1,0,1},
    {0,0,0,1},
    {1,0,0,1}
};

volatile motor_state_t g_motor_state = MOTOR_STOP;
volatile int g_lamp = 0;

/* ================= UART ================= */

void uart_init() {

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
}

/* ================= UART TASK ================= */

void uart_rx_task(void *arg) {

    char buf[64];
    int idx = 0;

    while (1) {

        uint8_t c;
        int len = uart_read_bytes(UART_PORT, &c, 1, pdMS_TO_TICKS(10));

        if (len > 0) {

            uart_write_bytes(UART_PORT, (const char *)&c, 1);

            if (c == '\n' || c == '\r') {

                if (idx > 0) {

                    buf[idx] = '\0';

                    if (strncmp(buf, "SET_TEMP:", 9) == 0) {

                        float val = atof(buf + 9);

                        if (val > -50 && val < 200) {

                            Tc = val;

                            char resp[64];

                            sprintf(resp,
                                    "\r\n>> Tc = %.1f C\r\n\r\n",
                                    Tc);

                            uart_write_bytes(UART_PORT,
                                             resp,
                                             strlen(resp));
                        }
                    }

                    idx = 0;
                }

            } else if (idx < 63) {

                buf[idx++] = (char)c;
            }
        }

        vTaskDelay(1);
    }
}

/* ================= ADC ================= */

void adc_init() {

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };

    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_oneshot_config_channel(adc1_handle,
                               ADC_TEMP,
                               &config);

    adc_oneshot_config_channel(adc1_handle,
                               ADC_LDR,
                               &config);

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    adc_cali_create_scheme_line_fitting(&cali_config,
                                        &adc_cali_handle);
}

/* ================= SENSORES ================= */

float leer_temperatura() {

    int raw, mv;
    int suma = 0;

    for (int i = 0; i < 64; i++) {

        adc_oneshot_read(adc1_handle,
                         ADC_TEMP,
                         &raw);

        adc_cali_raw_to_voltage(adc_cali_handle,
                                raw,
                                &mv);

        suma += mv;
    }

    return ((suma / 64) / 10.0f)- 7.3;
}

float leer_luz() {

    int raw;
    long suma = 0;

    for(int i = 0; i < 32; i++) {

        adc_oneshot_read(adc1_handle,
                         ADC_LDR,
                         &raw);

        suma += raw;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    raw = suma / 32;

    if (raw < LDR_MIN) raw = LDR_MIN;
    if (raw > LDR_MAX) raw = LDR_MAX;

    return ((float)(raw - LDR_MIN) /
           (LDR_MAX - LDR_MIN)) * 100.0f;
}

/* ================= LEDS ================= */

void set_led(float p) {

    if      (p < 20) led_percent = 100;
    else if (p < 30) led_percent = 80;
    else if (p < 40) led_percent = 60;
    else if (p < 60) led_percent = 50;
    else if (p < 80) led_percent = 30;
    else              led_percent = 0;


    uint32_t duty = (led_percent * 1023) / 100;

    ledc_set_duty(LEDC_MODE,
                  LEDC_CHANNEL,
                  duty);

    ledc_update_duty(LEDC_MODE,
                     LEDC_CHANNEL);
}

/* ================= LAMPARA ================= */

void lamp_init(void) {

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << RELAY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&cfg);

    gpio_set_level(RELAY_PIN,
                   LAMP_INVERT ? 1 : 0);
}

void lamp_set(int on) {

    g_lamp = on ? 1 : 0;

    int level = LAMP_INVERT ? !g_lamp : g_lamp;

    gpio_set_level(RELAY_PIN, level);
}

/* ================= MOTOR ================= */

void motor_init(void) {

    gpio_config_t cfg = {
        .pin_bit_mask =
            (1ULL<<IN1) |
            (1ULL<<IN2) |
            (1ULL<<IN3) |
            (1ULL<<IN4),

        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&cfg);

    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
}

void motor_off(void) {

    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
}

/* ================= LOGICA TEMPERATURA ================= */

void aplicar_temperatura(float T, float Tc) {

    if (T < Tc - 1) {

        g_motor_state = MOTOR_HEAT;
        lamp_set(1);

    } else if (T <= Tc + 1) {

        g_motor_state = MOTOR_STOP;
        lamp_set(0);

    } else if (T < Tc + 3) {

        g_motor_state = MOTOR_VENT_LOW;
        lamp_set(0);

    } else if (T <= Tc + 5) {

        g_motor_state = MOTOR_VENT_MED;
        lamp_set(0);

    } else {

        g_motor_state = MOTOR_VENT_HIGH;
        lamp_set(0);
    }
}

/* ================= MOTOR TASK ================= */

void motor_task(void *arg) {

    int idx = 0;
    int counter = 0;

    while (1) {

        motor_state_t s = g_motor_state;

        if (s == MOTOR_STOP) {

            motor_off();

            vTaskDelay(pdMS_TO_TICKS(50));

            continue;
        }

        int clockwise;
        uint32_t delay_us;

        switch (s) {

            case MOTOR_HEAT:
                clockwise = 1;
                delay_us = 10000;
                break;

            case MOTOR_VENT_LOW:
                clockwise = 0;
                delay_us = 10000;
                break;

            case MOTOR_VENT_MED:
                clockwise = 0;
                delay_us = 3333;
                break;

            case MOTOR_VENT_HIGH:
                clockwise = 0;
                delay_us = 1667;
                break;

            default:
                motor_off();
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
        }

        gpio_set_level(IN1, seq[idx].in1);
        gpio_set_level(IN2, seq[idx].in2);
        gpio_set_level(IN3, seq[idx].in3);
        gpio_set_level(IN4, seq[idx].in4);

        idx = clockwise ?
              (idx + 1) % 8 :
              (idx + 7) % 8;

        esp_rom_delay_us(delay_us);

        if (++counter >= 100) {

            counter = 0;

            vTaskDelay(1);
        }
    }
}

/* ================= MAIN ================= */

void app_main() {

    esp_task_wdt_deinit();

    uart_init();
    adc_init();

    motor_init();
    lamp_init();

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .duty_resolution = LEDC_DUTY_RES,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = LED_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&channel);

    xTaskCreatePinnedToCore(
        motor_task,
        "motor",
        4096,
        NULL,
        5,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        uart_rx_task,
        "uart_rx",
        2048,
        NULL,
        4,
        NULL,
        0
    );

    vTaskDelay(pdMS_TO_TICKS(100));

    uart_write_bytes(
        UART_PORT,
        "\r\n==============================\r\n"
        " SISTEMA DOMOTICO ESP32\r\n"
        "==============================\r\n"
        " Use SET_TEMP:XX\r\n"
        "==============================\r\n\r\n",
        118
    );

    char buffer[200];

    while (1) {

        T = leer_temperatura();

        luz = leer_luz();

        aplicar_temperatura(T, Tc);

        set_led(luz);


        char motor_msg[64];

        switch(g_motor_state) {

            case MOTOR_STOP:
                strcpy(motor_msg, "Motor apagado");
                break;

            case MOTOR_HEAT:
                strcpy(motor_msg, "Motor horario 100 steps/s");
                break;

            case MOTOR_VENT_LOW:
                strcpy(motor_msg, "Motor antihorario 100 steps/s");
                break;

            case MOTOR_VENT_MED:
                strcpy(motor_msg, "Motor antihorario 300 steps/s");
                break;

            case MOTOR_VENT_HIGH:
                strcpy(motor_msg, "Motor antihorario 600 steps/s");
                break;

            default:
                strcpy(motor_msg, "Motor desconocido");
                break;
        }
        
        
        sprintf(buffer,
            "Tc: %.1f C | "
            "T: %.2f C | "
            "LUZ: %.1f%% | "
            "LED: %.1f%% | "
            "LAMP:%s | "
            "MOTOR:%s\r\n",

            Tc,
            T,
            led_percent,
            luz,
            g_lamp ? "ON" : "OFF",
            motor_msg
        );

        uart_write_bytes(UART_PORT,
                         buffer,
                         strlen(buffer));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


