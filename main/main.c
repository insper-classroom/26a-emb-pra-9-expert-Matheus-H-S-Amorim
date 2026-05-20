
 
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "mpu6050.h"
#include "Fusion.h"
 
#define SAMPLE_PERIOD      0.01f   /* 10 ms → 100 Hz */
#define SAMPLE_PERIOD_MS   10
 
#define I2C_SDA_GPIO  4
#define I2C_SCL_GPIO  5
#define MPU_ADDRESS   0x68
 
#define LED_R_PIN  13
#define LED_G_PIN  11
#define LED_B_PIN  12
 
#define MOUSE_SCALE       3
#define DEAD_ZONE         7.0f
#define PEAK_THRESHOLD    2000
#define RETURN_THRESHOLD -1000
#define CLICK_TIMEOUT_MS  300
#define CLICK_COOLDOWN_MS 300
 

#define CORE_0  (1 << 0)
#define CORE_1  (1 << 1)
 

#define PIN_INSTR_MPU    2
#define PIN_INSTR_FUSAO  3
#define PIN_INSTR_LED    6
#define PIN_INSTR_UART   7
 
static inline void instr_init(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}
 

#define STACK_MPU     512    
#define STACK_FUSAO  1024    
#define STACK_LED     256    
#define STACK_UART    256    
#define STACK_MONITOR 512
 
// Tipos de dados
typedef struct {
    float acel_x, acel_y, acel_z;
    float giro_x, giro_y, giro_z;
} dados_mpu_t;
 
typedef struct {
    float rolagem, arfagem, guinada;
    bool  clique;
} dados_fusao_t;
 
typedef struct {
    uint8_t vermelho, verde, azul;
} cor_t;
 
typedef struct {
    float x, y;
    bool  clique;
} dados_pos_t;
 
//Filas
QueueHandle_t fila_mpu;
QueueHandle_t fila_cor;
QueueHandle_t fila_pos;
 
TaskHandle_t h_mpu, h_fusao, h_led, h_uart;
 //Helpers PWM
static void pwm_inicializar_pino(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);
    pwm_set_enabled(slice, true);
}
 
static void pwm_definir_ciclo(uint pin, uint8_t duty) {
    uint slice   = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, duty);
}
 
 // Helpers MPU-6050
static void mpu6050_inicializar(void) {
    i2c_init(i2c_default, 400000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}
 
static void mpu6050_ler_bruto(int16_t acel[3], int16_t giro[3]) {
    uint8_t buffer[14];
    uint8_t reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer, 14, false);
    for (int i = 0; i < 3; i++) acel[i] = (buffer[i*2]   << 8) | buffer[i*2+1];
    for (int i = 0; i < 3; i++) giro[i] = (buffer[8+i*2] << 8) | buffer[8+i*2+1];
}
 
// Helper UART
static void uart_enviar(uint8_t axis, int16_t val) {
    uint8_t pkt[4] = {0xFF, axis, val & 0xFF, (val >> 8) & 0xFF};
    for (int i = 0; i < 4; i++) putchar_raw(pkt[i]);
}
 
//tasks aq abaixo  
/*  tarefa_mpu  (CORE 0, prioridade 3)  */
void tarefa_mpu(void *p) {
    instr_init(PIN_INSTR_MPU);
    mpu6050_inicializar();
 
    int16_t     acel[3], giro[3];
    dados_mpu_t dados;
    TickType_t  xLastWake = xTaskGetTickCount();
 
    while (1) {
        // gpio_put(PIN_INSTR_MPU, 1);   /* DESATIVADO para medir stack */
 
        mpu6050_ler_bruto(acel, giro);
 
        dados.acel_x = acel[0] / 16384.0f;
        dados.acel_y = acel[1] / 16384.0f;
        dados.acel_z = acel[2] / 16384.0f;
        dados.giro_x = giro[0] / 131.0f;
        dados.giro_y = giro[1] / 131.0f;
        dados.giro_z = giro[2] / 131.0f;
 
        xQueueOverwrite(fila_mpu, &dados);
 
        // gpio_put(PIN_INSTR_MPU, 0);   /* DESATIVADO para medir stack */
 
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
 
// tarefa_fusao  (CORE 0, prioridade 2) 
typedef enum { WAIT_PEAK, WAIT_RETURN } ClickState;
 
void tarefa_fusao(void *p) {
    instr_init(PIN_INSTR_FUSAO);
 
    FusionAhrs   ahrs;
    FusionAhrsInitialise(&ahrs);
 
    dados_mpu_t  dados_mpu;
    dados_fusao_t dados_fusao;
    cor_t        cor;
    dados_pos_t  posicao;
 
    float      vermelho_medio = 0, verde_medio = 0, azul_medio = 0;
    const float alpha = 0.1f;
 
    int16_t    x_anterior   = 0;
    bool       primeiro      = true;
    ClickState estado         = WAIT_PEAK;
    TickType_t tempo_pico     = 0;
    TickType_t ultimo_clique  = 0;
 
    while (1) {
        if (xQueueReceive(fila_mpu, &dados_mpu, portMAX_DELAY)) {
 
            // gpio_put(PIN_INSTR_FUSAO, 1);   /* DESATIVADO para medir stack */
 
            FusionVector giro = { .axis = { dados_mpu.giro_x, dados_mpu.giro_y, dados_mpu.giro_z } };
            FusionVector acel = { .axis = { dados_mpu.acel_x, dados_mpu.acel_y, dados_mpu.acel_z } };
 
            FusionAhrsUpdateNoMagnetometer(&ahrs, giro, acel, SAMPLE_PERIOD);
            FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
 
            dados_fusao.rolagem = e.angle.roll;
            dados_fusao.arfagem = e.angle.pitch;
            dados_fusao.guinada = e.angle.yaw;
            dados_fusao.clique  = false;
 
            int16_t    x_bruto = (int16_t)(dados_mpu.acel_x * 16384.0f);
            int16_t    delta   = x_bruto - x_anterior;
            TickType_t agora   = xTaskGetTickCount();
 
            if (!primeiro) {
                switch (estado) {
                    case WAIT_PEAK:
                        if (delta > PEAK_THRESHOLD &&
                            (agora - ultimo_clique) > pdMS_TO_TICKS(CLICK_COOLDOWN_MS)) {
                            estado     = WAIT_RETURN;
                            tempo_pico = agora;
                        }
                        break;
                    case WAIT_RETURN:
                        if ((agora - tempo_pico) > pdMS_TO_TICKS(CLICK_TIMEOUT_MS)) {
                            estado = WAIT_PEAK;
                        } else if (delta < RETURN_THRESHOLD) {
                            dados_fusao.clique = true;
                            ultimo_clique      = agora;
                            estado             = WAIT_PEAK;
                        }
                        break;
                }
            } else {
                primeiro = false;
            }
            x_anterior = x_bruto;
 
            float rolagem = fmaxf(-90, fminf(90, dados_fusao.rolagem));
            float arfagem = fmaxf(-90, fminf(90, dados_fusao.arfagem));
 
            float vermelho = (rolagem > 0) ? (rolagem / 90.0f) * 255 : 0;
            float azul     = (rolagem < 0) ? (-rolagem / 90.0f) * 255 : 0;
            float verde    = (arfagem < 0) ? (-arfagem / 90.0f) * 255 : 0;
 
            vermelho_medio = alpha * vermelho + (1.0f - alpha) * vermelho_medio;
            verde_medio    = alpha * verde    + (1.0f - alpha) * verde_medio;
            azul_medio     = alpha * azul     + (1.0f - alpha) * azul_medio;
 
            cor.vermelho = (uint8_t)vermelho_medio;
            cor.verde    = (uint8_t)verde_medio;
            cor.azul     = (uint8_t)azul_medio;
 
            posicao.x      = dados_fusao.rolagem;
            posicao.y      = dados_fusao.arfagem;
            posicao.clique = dados_fusao.clique;
 
            xQueueOverwrite(fila_cor, &cor);
            xQueueOverwrite(fila_pos, &posicao);
 
            // gpio_put(PIN_INSTR_FUSAO, 0);   /* DESATIVADO para medir stack */
        }
    }
}
 //tarefa_led / pwm_task  (CORE 1, prioridade 2)
void tarefa_led(void *p) {
    instr_init(PIN_INSTR_LED);
    pwm_inicializar_pino(LED_R_PIN);
    pwm_inicializar_pino(LED_G_PIN);
    pwm_inicializar_pino(LED_B_PIN);
 
    cor_t cor;
 
    while (1) {
        if (xQueueReceive(fila_cor, &cor, portMAX_DELAY)) {
            // gpio_put(PIN_INSTR_LED, 1);   /* DESATIVADO para medir stack */
 
            pwm_definir_ciclo(LED_R_PIN, cor.vermelho);
            pwm_definir_ciclo(LED_G_PIN, cor.verde);
            pwm_definir_ciclo(LED_B_PIN, cor.azul);
 
            // gpio_put(PIN_INSTR_LED, 0);   /* DESATIVADO para medir stack */
        }
    }
}
 
// tarefa_uart  (CORE 1, prioridade 2) 
void tarefa_uart(void *p) {
    instr_init(PIN_INSTR_UART);
 
    dados_pos_t posicao;
 
    while (1) {
        if (xQueueReceive(fila_pos, &posicao, portMAX_DELAY)) {
            // gpio_put(PIN_INSTR_UART, 1);    desliguei aq para medir stack 
 
            int16_t vel_x = (fabsf(posicao.x) > DEAD_ZONE) ? (int16_t)(posicao.x * MOUSE_SCALE) : 0;
            int16_t vel_y = (fabsf(posicao.y) > DEAD_ZONE) ? (int16_t)(posicao.y * MOUSE_SCALE) : 0;
 
            if (vel_x)          uart_enviar(0, vel_x);
            if (vel_y)          uart_enviar(1, vel_y);
            if (posicao.clique)  uart_enviar(2, 1);
 
            // gpio_put(PIN_INSTR_UART, 0);   desliqguei aq para medir stack 
        }
    }
}
 
void stack_monitor_task(void *p) {
    static TaskStatus_t tasks[16];
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        UBaseType_t n = uxTaskGetSystemState(tasks, 16, NULL);
        printf("+------------------+-------+------+\n");
        printf("| %-16s | %5s | core |\n", "task", "free");
        printf("+------------------+-------+------+\n");
        for (UBaseType_t i = 0; i < n; i++) {
            printf("| %-16s | %5u |  %u   |\n",
                   tasks[i].pcTaskName,
                   (unsigned)tasks[i].usStackHighWaterMark,
                   (unsigned)vTaskCoreAffinityGet(tasks[i].xHandle));
        }
        printf("+------------------+-------+------+\n");
        printf("| heap livre min   | %5u |\n",
               (unsigned)xPortGetMinimumEverFreeHeapSize());
        printf("+------------------+-------+------+\n\n");
    }
}
 
// main() c/  SMP dual-core
int main(void) {
    stdio_init_all();
 
    fila_mpu = xQueueCreate(1, sizeof(dados_mpu_t));
    fila_cor = xQueueCreate(1, sizeof(cor_t));
    fila_pos = xQueueCreate(1, sizeof(dados_pos_t));
 
    // Criar tasks e guardar handles para affinidade 
    xTaskCreate(tarefa_mpu,   "mpu_task",    STACK_MPU,    NULL, 3, &h_mpu);
    xTaskCreate(tarefa_fusao, "fusion_task", STACK_FUSAO,  NULL, 2, &h_fusao);
    xTaskCreate(tarefa_led,   "pwm_task",    STACK_LED,    NULL, 2, &h_led);
    xTaskCreate(tarefa_uart,  "uart_task",   STACK_UART,   NULL, 2, &h_uart);
 
  
    vTaskCoreAffinitySet(h_mpu,   CORE_0);
    vTaskCoreAffinitySet(h_fusao, CORE_0);
    vTaskCoreAffinitySet(h_led,   CORE_1);
    vTaskCoreAffinitySet(h_uart,  CORE_1);
 
    xTaskCreate(stack_monitor_task, "monitor", STACK_MONITOR, NULL, 1, NULL); /* ATIVO */
 
    vTaskStartScheduler();
    while (1);
}
 
