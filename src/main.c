/*
 * main.c
 * ---------------------------------------------------------------------------
 * Punto de entrada app_main() del Sistema Multiagente de Ingenieria de
 * Firmware para el receptor GNSS u-blox NEO-6M-0-001 (ROM 7.03 / UBX 13.03)
 * sobre ESP32-S3 con ESP-IDF y FreeRTOS.
 *
 * Restriccion de pines estricta (Seccion 2):
 *   - GPIO 4 (IO4): UART1 TX -> RX del NEO-6M
 *   - GPIO 5 (IO5): UART1 RX <- TX del NEO-6M (pull-up interno)
 *   - GPIO 6 (IO6): Entrada opcional TIMEPULSE / PPS 1 Hz (pull-down interno)
 * ---------------------------------------------------------------------------
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "gps_config.h"
#include "gps_hw.h"
#include "gps_data.h"
#include "gps_discovery.h"
#include "gps_console.h"
#include "gps_wifi.h"
#include "ubx_protocol.h"

void app_main(void)
{
    /* Desactivar buffering de stdout para impresion determinista a 115200 bps */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* 1. Inicializar repositorio central protegido por Mutex y EventGroup */
    gps_data_init();

    /* 2. Mostrar banner de arquitectura y verificar constructor Fletcher-8 */
    gps_console_print_boot_banner();
    int selftest_fails = ubx_selftest(gps_console_selftest_cb);
    if (selftest_fails == 0) {
        printf("  [AUTOTEST FLETCHER-8] Todas las tramas UBX de referencia verificadas al 100%%.\r\n\r\n");
    } else {
        printf("  [ERROR CRITICO] Fallo en autodiagnostico Fletcher-8 (%d tramas).\r\n\r\n", selftest_fails);
    }

    /* 3. Inicializar driver UART1 (GPIO4/GPIO5) e interrupcion PPS (GPIO6) */
    esp_err_t err = gps_hw_init(GPS_DEFAULT_ROM_BAUD);
    if (err != ESP_OK) {
        printf("  [ERROR HARDWARE] Fallo al inicializar UART1 en GPIO4/GPIO5: err=0x%x\r\n", (unsigned)err);
        return;
    }

    /* 4. Inicializar WiFi + Servidor HTTP REST / SSE / WebSocket en Core 0 (cada 10 s) */
    gps_wifi_init_and_start();

    /* 5. Lanzar las 3 tareas concurrentes FreeRTOS (Seccion 7.4):
     *    - Tarea 1: gps_rx_parser_task (Core 1, prioridad alta, stack 8192)
     *    - Tarea 2: gps_discovery_and_manager_task (Core 0, prioridad media, stack 8192)
     *    - Tarea 3: gps_telemetry_console_task (Core 0, prioridad baja, stack 10240)
     */
    BaseType_t ok1 = xTaskCreatePinnedToCore(
        gps_rx_parser_task,
        "gps_rx_parser",
        TASK_RX_STACK,
        NULL,
        TASK_RX_PRIO,
        NULL,
        TASK_RX_CORE
    );

    BaseType_t ok2 = xTaskCreatePinnedToCore(
        gps_discovery_and_manager_task,
        "gps_disc_mgr",
        TASK_MGR_STACK,
        NULL,
        TASK_MGR_PRIO,
        NULL,
        TASK_MGR_CORE
    );

    BaseType_t ok3 = xTaskCreatePinnedToCore(
        gps_telemetry_console_task,
        "gps_console",
        TASK_CON_STACK,
        NULL,
        TASK_CON_PRIO,
        NULL,
        TASK_CON_CORE
    );

    if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS) {
        printf("  [ERROR FREERTOS] No se pudieron crear todas las tareas (rx=%ld, mgr=%ld, con=%ld)\r\n",
               (long)ok1, (long)ok2, (long)ok3);
    }
}
