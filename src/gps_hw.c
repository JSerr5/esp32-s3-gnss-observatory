/*
 * gps_hw.c
 * ---------------------------------------------------------------------------
 * Agente 1 - Implementacion del driver UART1 (GPIO4 TX, GPIO5 RX), detector
 * de pulso PPS por interrupcion en GPIO6 (IRAM_ATTR) y tarea de recepcion
 * FreeRTOS de baja latencia fijada al Core 1.
 *
 * UNICAMENTE se referencian GPIO_NUM_4, GPIO_NUM_5 y GPIO_NUM_6.
 * ---------------------------------------------------------------------------
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "gps_config.h"
#include "gps_hw.h"
#include "gps_data.h"
#include "ubx_protocol.h"
#include "nmea_parser.h"

/* Cola de eventos del driver UART1 */
static QueueHandle_t s_uart_queue = NULL;
static uint32_t      s_current_baud = GPS_DEFAULT_ROM_BAUD;

/* Variables volatiles actualizadas desde la ISR de GPIO 6 (PPS) */
static volatile uint32_t s_pps_count       = 0;
static volatile int64_t  s_pps_last_us     = 0;
static volatile int64_t  s_pps_period_us   = 0;
static volatile int64_t  s_pps_jitter_us   = 0;
static volatile int64_t  s_pps_max_jit_us  = 0;
static volatile int64_t  s_pps_avg_jit_us  = 0;

/* -------------------------------------------------------------------------
 * ISR en IRAM para el pin GPIO 6 (TIMEPULSE / PPS de 1 Hz)
 * ------------------------------------------------------------------------- */
static void IRAM_ATTR gps_pps_isr_handler(void *arg)
{
    (void)arg;
    int64_t now_us = esp_timer_get_time();
    int64_t prev_us = s_pps_last_us;

    /* Filtro antirrebote: ignora pulsos espurios separados menos de 200 ms */
    if (prev_us != 0 && (now_us - prev_us) < 200000LL) {
        return;
    }

    s_pps_last_us = now_us;
    s_pps_count++;

    if (prev_us != 0) {
        int64_t period = now_us - prev_us;
        s_pps_period_us = period;
        /* Solo calcula jitter respecto a 1 Hz si el periodo ronda 1 segundo */
        if (period > 800000LL && period < 1200000LL) {
            int64_t jit = period - 1000000LL;
            int64_t abs_jit = (jit < 0) ? -jit : jit;
            s_pps_jitter_us = jit;
            if (abs_jit > s_pps_max_jit_us) {
                s_pps_max_jit_us = abs_jit;
            }
            if (s_pps_avg_jit_us == 0) {
                s_pps_avg_jit_us = abs_jit;
            } else {
                s_pps_avg_jit_us = (s_pps_avg_jit_us * 7LL + abs_jit) / 8LL;
            }
        }
    }
}

/* -------------------------------------------------------------------------
 * Inicializacion de GPIO 4, GPIO 5 (UART1) y GPIO 6 (PPS opcional)
 * ------------------------------------------------------------------------- */
esp_err_t gps_hw_init(uint32_t initial_baud)
{
    s_current_baud = initial_baud;

    const uart_config_t uart_cfg = {
        .baud_rate  = (int)initial_baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(
        GPS_UART_NUM,
        GPS_UART_RX_BUF,
        GPS_UART_TX_BUF,
        GPS_UART_EVT_QUEUE,
        &s_uart_queue,
        0
    );
    if (err != ESP_OK) {
        return err;
    }

    err = uart_param_config(GPS_UART_NUM, &uart_cfg);
    if (err != ESP_OK) {
        return err;
    }

    /* Asignacion estricta: TX = GPIO_NUM_4, RX = GPIO_NUM_5 */
    err = uart_set_pin(
        GPS_UART_NUM,
        GPS_PIN_TX,
        GPS_PIN_RX,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
    if (err != ESP_OK) {
        return err;
    }

    /* Pull-up interno en GPIO 5 (RX) para evitar ruido si el cable esta suelto */
    gpio_set_pull_mode(GPS_PIN_RX, GPIO_PULLUP_ONLY);

    /* Configuracion opcional del pin PPS en GPIO 6 con pull-down suave
     * para que permanezca en 0 logico si el usuario solo conecta 4 hilos. */
    const gpio_config_t pps_cfg = {
        .pin_bit_mask = (1ULL << GPS_PIN_PPS),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&pps_cfg);

    esp_err_t isr_err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isr_err == ESP_OK || isr_err == ESP_ERR_INVALID_STATE) {
        gpio_isr_handler_add(GPS_PIN_PPS, gps_pps_isr_handler, NULL);
    }

    gps_repo_t *r = gps_data_lock();
    r->current_baud = initial_baud;
    gps_data_unlock();

    return ESP_OK;
}

esp_err_t gps_hw_set_baud(uint32_t baud)
{
    uart_wait_tx_done(GPS_UART_NUM, pdMS_TO_TICKS(100));
    esp_err_t err = uart_set_baudrate(GPS_UART_NUM, baud);
    uart_flush_input(GPS_UART_NUM);
    if (s_uart_queue != NULL) {
        xQueueReset(s_uart_queue);
    }
    s_current_baud = baud;

    gps_repo_t *r = gps_data_lock();
    r->current_baud = baud;
    gps_data_unlock();

    return err;
}

uint32_t gps_hw_get_baud(void)
{
    return s_current_baud;
}

void gps_hw_flush_rx(void)
{
    uart_flush_input(GPS_UART_NUM);
    if (s_uart_queue != NULL) {
        xQueueReset(s_uart_queue);
    }
}

int gps_hw_send_raw(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return 0;
    }
    int written = uart_write_bytes(GPS_UART_NUM, (const char *)data, len);
    if (written > 0) {
        gps_repo_t *r = gps_data_lock();
        r->bytes_tx += (uint32_t)written;
        gps_data_unlock();
    }
    return written;
}

int gps_hw_send_ubx(uint8_t cls, uint8_t id, const void *payload, uint16_t len)
{
    uint8_t frame[128];
    if ((size_t)len + UBX_FRAME_OVERHEAD > sizeof(frame)) {
        return -1;
    }
    size_t n = ubx_build_frame(frame, sizeof(frame), cls, id, payload, len);
    if (n == 0) {
        return -1;
    }
    return gps_hw_send_raw(frame, n);
}

int gps_hw_rx_pin_level(void)
{
    return gpio_get_level(GPS_PIN_RX);
}

void gps_hw_get_pps(gps_pps_stat_t *out)
{
    if (out == NULL) {
        return;
    }
    int64_t now_us  = esp_timer_get_time();
    int64_t last_us = s_pps_last_us;

    out->pulse_count    = s_pps_count;
    out->last_edge_us   = last_us;
    out->last_period_us = s_pps_period_us;
    out->last_jitter_us = s_pps_jitter_us;
    out->max_jitter_us  = s_pps_max_jit_us;
    out->avg_jitter_us  = s_pps_avg_jit_us;
    out->pin_level      = gpio_get_level(GPS_PIN_PPS);
    out->active         = (last_us > 0) && ((now_us - last_us) < 2500000LL);
}

/* -------------------------------------------------------------------------
 * Tarea 1: gps_rx_parser_task (Core 1, prioridad alta, zero-malloc)
 * Alimenta en paralelo el decodificador binario UBX y el ASCII NMEA/PUBX.
 * ------------------------------------------------------------------------- */
static ubx_parser_t  s_ubx_parser;
static nmea_parser_t s_nmea_parser;
static uint8_t       s_rx_chunk[GPS_UART_RX_CHUNK];

void gps_rx_parser_task(void *arg)
{
    (void)arg;
    ubx_parser_init(&s_ubx_parser);
    nmea_parser_init(&s_nmea_parser);

    uart_event_t evt;
    for (;;) {
        if (xQueueReceive(s_uart_queue, &evt, pdMS_TO_TICKS(200)) != pdTRUE) {
            continue;
        }

        switch (evt.type) {
        case UART_DATA: {
            size_t rem = evt.size;
            while (rem > 0) {
                size_t to_read = (rem > sizeof(s_rx_chunk)) ? sizeof(s_rx_chunk) : rem;
                int n = uart_read_bytes(GPS_UART_NUM, s_rx_chunk, (uint32_t)to_read, pdMS_TO_TICKS(20));
                if (n <= 0) {
                    break;
                }
                rem -= (size_t)n;
                gps_data_on_bytes((uint32_t)n);

                for (int i = 0; i < n; i++) {
                    uint8_t b = s_rx_chunk[i];

                    /* 1) Alimentar maquina de estados binaria UBX */
                    ubx_feed_t ur = ubx_parser_feed(&s_ubx_parser, b);
                    if (ur == UBX_FEED_FRAME) {
                        gps_data_on_ubx(s_ubx_parser.cls, s_ubx_parser.id,
                                        s_ubx_parser.payload, s_ubx_parser.len);
                    }

                    /* 2) Alimentar maquina de estados ASCII NMEA/PUBX */
                    nmea_feed_t nr = nmea_parser_feed(&s_nmea_parser, (char)b);
                    if (nr == NMEA_FEED_SENTENCE) {
                        gps_data_on_nmea(&s_nmea_parser);
                    }
                }

                /* Actualizar contadores globales de los parsers */
                gps_repo_t *r = gps_data_lock();
                r->ubx_ok        = s_ubx_parser.frames_ok;
                r->ubx_ckerr     = s_ubx_parser.frames_ckerr;
                r->ubx_oversize  = s_ubx_parser.frames_oversize;
                r->nmea_ok       = s_nmea_parser.ok;
                r->nmea_ckerr    = s_nmea_parser.ckerr;
                r->nmea_overflow = s_nmea_parser.overflow;
                gps_data_unlock();
            }
            break;
        }

        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            gps_data_on_uart_error((int)evt.type);
            uart_flush_input(GPS_UART_NUM);
            xQueueReset(s_uart_queue);
            break;

        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
        case UART_BREAK:
            gps_data_on_uart_error((int)evt.type);
            break;

        default:
            break;
        }
    }
}
