// ===========================================================
// Librerias estandar y del framework ESP-IDF / FreeRTOS
// ===========================================================
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

// Librerias BLE (NimBLE)
#include "esp_bt.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// ===========================================================
// Etiqueta de log general del sistema
// ===========================================================
static const char *LOG_TAG = "LAB4_SYSTEM";

// ===========================================================
// Pines de LEDs y buzzer
// ===========================================================
#define PIN_LED_RED    25
#define PIN_LED_GREEN  26
#define PIN_LED_BLUE   27
#define PIN_BUZZER     32

// ===========================================================
// Configuracion I2C
// ===========================================================
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_CLOCK_FREQ  100000

// ===========================================================
// Direccion y registro del DS1307 (RTC)
// ===========================================================
#define DS1307_I2C_ADDR    0x68
#define DS1307_REG_SECONDS 0x00

// ===========================================================
// Configuracion del LCD I2C (PCF8574)
// ===========================================================
#define LCD_I2C_ADDR  0x27
#define LCD_FLAG_RS   0x01
#define LCD_FLAG_EN   0x04
#define LCD_FLAG_BL   0x08

// ===========================================================
// Pines y registros del RC522 (RFID por SPI)
// ===========================================================
#define RFID_PIN_MISO 19
#define RFID_PIN_MOSI 23
#define RFID_PIN_SCK  18
#define RFID_PIN_CS   5
#define RFID_PIN_RST  4

#define REG_COMMAND      0x01
#define REG_COM_IRQ      0x04
#define REG_ERROR        0x06
#define REG_FIFO_DATA    0x09
#define REG_FIFO_LEVEL   0x0A
#define REG_CONTROL      0x0C
#define REG_BIT_FRAMING  0x0D
#define REG_MODE         0x11
#define REG_TX_CONTROL   0x14
#define REG_TX_ASK       0x15
#define REG_T_MODE       0x2A
#define REG_T_PRESCALER  0x2B
#define REG_T_RELOAD_H   0x2C
#define REG_T_RELOAD_L   0x2D
#define REG_VERSION      0x37

#define CMD_IDLE         0x00
#define CMD_TRANSCEIVE   0x0C
#define CMD_SOFT_RESET   0x0F

#define PICC_REQA        0x26
#define PICC_ANTICOLL    0x93

// ===========================================================
// Nombre del dispositivo BLE
// ===========================================================
#define BLE_DEVICE_NAME "PanelHMI"

// ===========================================================
// Estados del sistema
// ===========================================================
typedef enum {
    STATE_LOCKED,
    STATE_ACTIVE
} system_state_t;

static system_state_t currentState = STATE_LOCKED;

// ===========================================================
// Handles de timers (one-shot y periodicos)
// ===========================================================
static esp_timer_handle_t timerRedBlink  = NULL;  // Parpadeo LED rojo (acceso denegado)
static esp_timer_handle_t timerGreenOff  = NULL;  // Apagar LED verde (sesion abierta)
static esp_timer_handle_t timerBuzzerOff = NULL;  // Apagar buzzer
static esp_timer_handle_t timerLcdReset  = NULL;  // Restaurar pantalla LCD al estado bloqueado

static bool isBlinking   = false;  // Indica si el LED rojo esta en secuencia de parpadeo
static int  blinkCounter = 0;      // Contador de toggles del parpadeo

// ===========================================================
// Handle SPI del RC522
// ===========================================================
static spi_device_handle_t rfidSpiHandle = NULL;

// UID autorizado para acceso
static uint8_t authorizedUid[4] = {0x63, 0x2F, 0x28, 0x07};

// ===========================================================
// Variables de conexion BLE
// ===========================================================
static uint8_t  bleAddrType;                                // Tipo de direccion MAC del ESP32
static uint16_t bleConnHandle = BLE_HS_CONN_HANDLE_NONE;    // Handle de conexion con el celular
static uint16_t bleTxValHandle;                              // Handle de la caracteristica TX
static bool     bleNotifyEnabled = false;                    // Indica si el celular esta suscrito

// Buffer y bandera para mensajes recibidos por BLE
static char bleMessage[17]  = "Sin mensajes";
static bool bleNewMessage   = false;

// ===========================================================
// UUIDs del servicio NUS (Nordic UART Service)
// ===========================================================
static const ble_uuid128_t nusServiceUuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,
                     0x93,0xf3,0xa3,0xb5,0x01,0x00,0x40,0x6e);

static const ble_uuid128_t nusRxUuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,
                     0x93,0xf3,0xa3,0xb5,0x02,0x00,0x40,0x6e);

static const ble_uuid128_t nusTxUuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,
                     0x93,0xf3,0xa3,0xb5,0x03,0x00,0x40,0x6e);

// ===========================================================
// Declaraciones anticipadas
// ===========================================================
static void ble_start_advertising(void);
static void lcd_clear_screen(void);
static void lcd_set_cursor_position(uint8_t col, uint8_t row);
static void lcd_print_padded(const char *str);

// ===========================================================
// Funciones utilitarias: conversion BCD <-> Decimal
// ===========================================================
static int bcd_to_decimal(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static uint8_t decimal_to_bcd(int decimal)
{
    return ((decimal / 10) << 4) | (decimal % 10);
}

// ===========================================================
// Inicializacion del bus I2C en modo maestro
// ===========================================================
static void i2c_initialize(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_I2C_SDA,
        .scl_io_num       = PIN_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLOCK_FREQ,
    };

    i2c_param_config(I2C_MASTER_PORT, &cfg);
    i2c_driver_install(I2C_MASTER_PORT, cfg.mode, 0, 0, 0);

    ESP_LOGI(LOG_TAG, "Bus I2C inicializado");
}

// ===========================================================
//                  DRIVER LCD (I2C, PCF8574)
// ===========================================================

// Envia un byte crudo al expansor I2C del LCD
static void lcd_send_raw_byte(uint8_t data)
{
    i2c_master_write_to_device(I2C_MASTER_PORT, LCD_I2C_ADDR,
                               &data, 1, pdMS_TO_TICKS(100));
}

// Genera un pulso en el pin Enable del LCD
static void lcd_pulse_enable(uint8_t data)
{
    lcd_send_raw_byte(data | LCD_FLAG_EN | LCD_FLAG_BL);
    esp_rom_delay_us(1000);

    lcd_send_raw_byte((data & ~LCD_FLAG_EN) | LCD_FLAG_BL);
    esp_rom_delay_us(1000);
}

// Envia un nibble (4 bits altos) al LCD
static void lcd_write_nibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble & 0xF0) | mode | LCD_FLAG_BL;
    lcd_pulse_enable(data);
}

// Envia un byte completo al LCD (dos nibbles consecutivos)
static void lcd_write_byte(uint8_t value, uint8_t mode)
{
    lcd_write_nibble(value & 0xF0, mode);
    lcd_write_nibble((value << 4) & 0xF0, mode);
}

// Envia un comando al LCD (RS = 0)
static void lcd_send_command(uint8_t cmd)
{
    lcd_write_byte(cmd, 0);
}

// Envia un dato al LCD (RS = 1)
static void lcd_send_data(uint8_t data)
{
    lcd_write_byte(data, LCD_FLAG_RS);
}

// Limpia toda la pantalla del LCD
static void lcd_clear_screen(void)
{
    lcd_send_command(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
}

// Secuencia de inicializacion del LCD en modo 4 bits, 2 lineas
static void lcd_initialize(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    // Secuencia de reset por software (datasheet HD44780)
    lcd_write_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Cambiar a modo 4 bits
    lcd_write_nibble(0x20, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configurar: 4 bits, 2 lineas, fuente 5x8
    lcd_send_command(0x28);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Apagar display temporalmente
    lcd_send_command(0x08);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Limpiar pantalla
    lcd_clear_screen();

    // Modo de entrada: incrementar cursor, sin desplazamiento
    lcd_send_command(0x06);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Encender display, cursor apagado
    lcd_send_command(0x0C);
    vTaskDelay(pdMS_TO_TICKS(5));
}

// Posiciona el cursor en la columna y fila indicadas
static void lcd_set_cursor_position(uint8_t col, uint8_t row)
{
    lcd_send_command((row == 0 ? 0x80 : 0xC0) + col);
}

// Imprime una cadena de texto en la posicion actual del cursor
static void lcd_print_string(const char *str)
{
    while (*str) {
        lcd_send_data((uint8_t)*str++);
    }
}

// Imprime exactamente 16 caracteres, rellenando con espacios si es necesario
static void lcd_print_padded(const char *str)
{
    char buffer[17];
    memset(buffer, ' ', 16);

    for (int i = 0; i < 16 && str[i] != '\0'; i++) {
        buffer[i] = str[i];
    }

    buffer[16] = '\0';
    lcd_print_string(buffer);
}

// ===========================================================
//                  DRIVER DS1307 (RTC por I2C)
// ===========================================================

// Escribe hora, minuto y segundo en el DS1307 (CH=0 para iniciar oscilador)
static void rtc_set_time(int hour, int min, int sec)
{
    uint8_t data[4];
    data[0] = 0x00;                        // Registro de segundos
    data[1] = decimal_to_bcd(sec) & 0x7F;  // Bit CH = 0 (oscilador activo)
    data[2] = decimal_to_bcd(min);
    data[3] = decimal_to_bcd(hour);

    i2c_master_write_to_device(I2C_MASTER_PORT, DS1307_I2C_ADDR,
                               data, 4, pdMS_TO_TICKS(200));
}

// Lee hora, minuto y segundo desde el DS1307
static void rtc_get_time(uint8_t *hour, uint8_t *min, uint8_t *sec)
{
    uint8_t reg = 0x00;
    uint8_t data[3] = {0};

    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_PORT,
        DS1307_I2C_ADDR,
        &reg, 1,
        data, 3,
        pdMS_TO_TICKS(200)
    );

    if (ret != ESP_OK) {
        *hour = 0;
        *min  = 0;
        *sec  = 0;
        return;
    }

    int s = bcd_to_decimal(data[0] & 0x7F);  // Mascara bit CH
    int m = bcd_to_decimal(data[1]);
    int h = bcd_to_decimal(data[2] & 0x3F);  // Mascara formato 24h

    // Validacion basica para evitar valores fuera de rango
    if (h > 23 || m > 59 || s > 59) {
        *hour = 0;
        *min  = 0;
        *sec  = 0;
        return;
    }

    *hour = (uint8_t)h;
    *min  = (uint8_t)m;
    *sec  = (uint8_t)s;
}

// ===========================================================
//                  DRIVER RC522 (RFID por SPI)
// ===========================================================

// Escribe un valor en un registro del RC522
static void rfid_write_register(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {(reg << 1) & 0x7E, val};

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx
    };

    spi_device_transmit(rfidSpiHandle, &t);
}

// Lee un valor desde un registro del RC522
static uint8_t rfid_read_register(uint8_t reg)
{
    uint8_t tx[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx[2] = {0};

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(rfidSpiHandle, &t);
    return rx[1];
}

// Activa bits especificos en un registro del RC522 (OR logico)
static void rfid_set_register_bits(uint8_t reg, uint8_t mask)
{
    rfid_write_register(reg, rfid_read_register(reg) | mask);
}

// Inicializa el modulo RC522: configura SPI, reset y registros base
static void rfid_initialize(void)
{
    spi_bus_config_t busCfg = {
        .miso_io_num   = RFID_PIN_MISO,
        .mosi_io_num   = RFID_PIN_MOSI,
        .sclk_io_num   = RFID_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_bus_initialize(SPI2_HOST, &busCfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devCfg = {
        .clock_speed_hz = 1000000,
        .mode           = 0,
        .spics_io_num   = RFID_PIN_CS,
        .queue_size     = 5
    };

    spi_bus_add_device(SPI2_HOST, &devCfg, &rfidSpiHandle);

    // Reset por hardware
    gpio_set_direction(RFID_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(RFID_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(RFID_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Reset por software
    rfid_write_register(REG_COMMAND, CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Configurar temporizador interno y modulacion
    rfid_write_register(REG_T_MODE, 0x80);
    rfid_write_register(REG_T_PRESCALER, 0xA9);
    rfid_write_register(REG_T_RELOAD_H, 0x03);
    rfid_write_register(REG_T_RELOAD_L, 0xE8);

    rfid_write_register(REG_TX_ASK, 0x40);
    rfid_write_register(REG_MODE, 0x3D);

    // Habilitar antena TX
    rfid_set_register_bits(REG_TX_CONTROL, 0x03);

    ESP_LOGI(LOG_TAG, "RC522 inicializado correctamente");
}

// Envia un REQA y verifica si hay una tarjeta presente en el campo
static bool rfid_detect_card(void)
{
    rfid_write_register(REG_BIT_FRAMING, 0x07);

    rfid_write_register(REG_COM_IRQ, 0x7F);
    rfid_write_register(REG_FIFO_LEVEL, 0x80);
    rfid_write_register(REG_FIFO_DATA, PICC_REQA);

    rfid_write_register(REG_COMMAND, CMD_TRANSCEIVE);
    rfid_write_register(REG_BIT_FRAMING, 0x87);

    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t irq = rfid_read_register(REG_COM_IRQ);
    return (irq & 0x20) ? true : false;
}

// Realiza anti-colision y lee el UID de 4 bytes de la tarjeta detectada
static bool rfid_read_card_uid(uint8_t *uid)
{
    uint8_t anticollCmd[2] = {PICC_ANTICOLL, 0x20};

    // Resetear bits de framing antes de enviar comando
    rfid_write_register(REG_BIT_FRAMING, 0x00);

    rfid_write_register(REG_COM_IRQ, 0x7F);
    rfid_write_register(REG_FIFO_LEVEL, 0x80);

    rfid_write_register(REG_FIFO_DATA, anticollCmd[0]);
    rfid_write_register(REG_FIFO_DATA, anticollCmd[1]);

    rfid_write_register(REG_COMMAND, CMD_TRANSCEIVE);

    // Activar transmision
    rfid_set_register_bits(REG_BIT_FRAMING, 0x80);

    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t fifoLen = rfid_read_register(REG_FIFO_LEVEL);
    if (fifoLen < 4) return false;

    for (int i = 0; i < 4; i++) {
        uid[i] = rfid_read_register(REG_FIFO_DATA);
    }

    return true;
}

// ===========================================================
//          CALLBACKS DE TIMERS (ONE-SHOT / PERIODICO)
// ===========================================================

// Callback periodico: parpadeo del LED rojo cuando el acceso es denegado
static void on_red_led_blink(void *arg)
{
    if (blinkCounter < 6) {
        gpio_set_level(PIN_LED_RED, (blinkCounter % 2 == 0) ? 1 : 0);
        blinkCounter++;
    } else {
        // Fin del parpadeo: LED rojo queda encendido (estado bloqueado)
        gpio_set_level(PIN_LED_RED, 1);
        esp_timer_stop(timerRedBlink);
        isBlinking = false;
    }
}

// Callback one-shot: apagar LED verde tras acceso concedido
static void on_green_led_off(void *arg)
{
    gpio_set_level(PIN_LED_GREEN, 0);
}

// Callback one-shot: apagar buzzer
static void on_buzzer_off(void *arg)
{
    gpio_set_level(PIN_BUZZER, 0);
}

// Callback one-shot: restaurar LCD al mensaje de panel bloqueado
static void on_lcd_reset(void *arg)
{
    lcd_clear_screen();
    lcd_set_cursor_position(0, 0);
    lcd_print_padded("Panel bloqueado");
    lcd_set_cursor_position(0, 1);
    lcd_print_padded("Acerque cred.");
}

// ===========================================================
//                      BLE (NimBLE - NUS)
// ===========================================================

// Callback de acceso RX: se invoca cuando el celular escribe un mensaje
static int ble_rx_access_callback(uint16_t connHdl, uint16_t attrHdl,
                                  struct ble_gatt_access_ctxt *ctx, void *arg)
{
    uint8_t rxData[64];
    uint16_t rxLen = 0;

    // Leer los bytes del buffer mbuf
    int rc = ble_hs_mbuf_to_flat(ctx->om, rxData, sizeof(rxData) - 1, &rxLen);
    if (rc != 0) return BLE_ATT_ERR_UNLIKELY;

    // Terminar la cadena recibida
    rxData[rxLen] = '\0';

    // Solo procesar si el sistema esta en estado activo
    if (currentState == STATE_ACTIVE) {
        strncpy(bleMessage, (char*)rxData, 16);
        bleMessage[16] = '\0';
        bleNewMessage = true;
        ESP_LOGI(LOG_TAG, "Mensaje BLE recibido: %s", bleMessage);
    }

    return 0;
}

// Callback de acceso TX: requerido por GATT pero sin logica adicional
static int ble_tx_access_callback(uint16_t connHdl, uint16_t attrHdl,
                                  struct ble_gatt_access_ctxt *ctx, void *arg)
{
    return 0;
}

// Tabla de definicion de servicios y caracteristicas GATT
static const struct ble_gatt_svc_def gattServices[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nusServiceUuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            // Caracteristica RX: el celular escribe aqui
            {
                .uuid      = &nusRxUuid.u,
                .access_cb = ble_rx_access_callback,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            // Caracteristica TX: el ESP32 notifica por aqui
            {
                .uuid       = &nusTxUuid.u,
                .access_cb  = ble_tx_access_callback,
                .val_handle = &bleTxValHandle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }  // Fin de caracteristicas
        },
    },
    { 0 }  // Fin de servicios
};

// Manejador de eventos GAP (conexion, desconexion, suscripcion, advertising)
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            bleConnHandle = event->connect.conn_handle;
            ESP_LOGI(LOG_TAG, "Dispositivo BLE conectado");
        } else {
            ble_start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(LOG_TAG, "Dispositivo BLE desconectado");
        bleConnHandle = BLE_HS_CONN_HANDLE_NONE;
        bleNotifyEnabled = false;
        ble_start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == bleTxValHandle) {
            bleNotifyEnabled = event->subscribe.cur_notify;
            ESP_LOGI(LOG_TAG, "Suscripcion TX: %s",
                     bleNotifyEnabled ? "ON" : "OFF");
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_start_advertising();
        return 0;

    default:
        return 0;
    }
}

// Configura e inicia el advertising BLE (visible como BLE_DEVICE_NAME)
static void ble_start_advertising(void)
{
    struct ble_hs_adv_fields advFields;
    struct ble_hs_adv_fields rspFields;
    struct ble_gap_adv_params advParams;

    // Paquete de advertising: flags + UUID del servicio NUS
    memset(&advFields, 0, sizeof(advFields));
    advFields.flags                = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    advFields.uuids128             = (ble_uuid128_t *)&nusServiceUuid;
    advFields.num_uuids128         = 1;
    advFields.uuids128_is_complete = 1;
    ble_gap_adv_set_fields(&advFields);

    // Scan response: nombre del dispositivo
    memset(&rspFields, 0, sizeof(rspFields));
    const char *deviceName      = ble_svc_gap_device_name();
    rspFields.name              = (uint8_t *)deviceName;
    rspFields.name_len          = strlen(deviceName);
    rspFields.name_is_complete  = 1;
    ble_gap_adv_rsp_set_fields(&rspFields);

    // Parametros de advertising: conectable, descubrible general
    memset(&advParams, 0, sizeof(advParams));
    advParams.conn_mode = BLE_GAP_CONN_MODE_UND;
    advParams.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(bleAddrType, NULL, BLE_HS_FOREVER,
                      &advParams, ble_gap_event_handler, NULL);

    ESP_LOGI(LOG_TAG, "Advertising BLE como: %s", BLE_DEVICE_NAME);
}

// Callback de sincronizacion: se ejecuta cuando el stack BLE esta listo
static void ble_sync_callback(void)
{
    ble_hs_id_infer_auto(0, &bleAddrType);
    ble_start_advertising();
}

// Callback de reset: se ejecuta si el stack BLE se reinicia
static void ble_reset_callback(int reason)
{
    ESP_LOGE(LOG_TAG, "Reset BLE, razon: %d", reason);
}

// Tarea de FreeRTOS que ejecuta el host BLE
static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Inicializacion completa del stack BLE
static void ble_initialize(void)
{
    // Inicializar memoria NVS (requerida por BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Liberar memoria del Bluetooth clasico (no se usa)
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    // Inicializar el puerto NimBLE
    nimble_port_init();

    // Inicializar servicios GAP y GATT
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Establecer nombre del dispositivo
    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);

    // Registrar callbacks de sincronizacion y reset
    ble_hs_cfg.sync_cb  = ble_sync_callback;
    ble_hs_cfg.reset_cb = ble_reset_callback;

    // Registrar tabla de servicios GATT
    ble_gatts_count_cfg(gattServices);
    ble_gatts_add_svcs(gattServices);

    // Arrancar tarea del host BLE en FreeRTOS
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(LOG_TAG, "BLE inicializado como %s", BLE_DEVICE_NAME);
}

// ===========================================================
//                      FUNCION PRINCIPAL
// ===========================================================
void app_main(void)
{
    // Configurar pines como salida
    gpio_set_direction(PIN_LED_RED,   GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_BLUE,  GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_BUZZER,    GPIO_MODE_OUTPUT);

    // Estado inicial: panel bloqueado, solo LED rojo encendido
    gpio_set_level(PIN_LED_RED,   1);
    gpio_set_level(PIN_LED_GREEN, 0);
    gpio_set_level(PIN_LED_BLUE,  0);
    gpio_set_level(PIN_BUZZER,    0);

    // Inicializar perifericos
    i2c_initialize();
    lcd_initialize();
    rtc_set_time(11, 36, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    rfid_initialize();

    ESP_LOGI(LOG_TAG, "Iniciando sistema...");
    ble_initialize();
    ESP_LOGI(LOG_TAG, "BLE iniciado");

    // Mostrar mensaje inicial en LCD
    lcd_set_cursor_position(0, 0);
    lcd_print_padded("Panel bloqueado");
    lcd_set_cursor_position(0, 1);
    lcd_print_padded("Acerque cred.");

    // Crear timers del sistema
    esp_timer_create_args_t argsBlink = {
        .callback = on_red_led_blink,
        .name     = "red_blink"
    };
    esp_timer_create(&argsBlink, &timerRedBlink);

    esp_timer_create_args_t argsGreen = {
        .callback = on_green_led_off,
        .name     = "green_off"
    };
    esp_timer_create(&argsGreen, &timerGreenOff);

    esp_timer_create_args_t argsBuzzer = {
        .callback = on_buzzer_off,
        .name     = "buzzer_off"
    };
    esp_timer_create(&argsBuzzer, &timerBuzzerOff);

    esp_timer_create_args_t argsLcd = {
        .callback = on_lcd_reset,
        .name     = "lcd_reset"
    };
    esp_timer_create(&argsLcd, &timerLcdReset);

    // Variables del bucle principal
    uint8_t hours = 0, minutes = 0, seconds = 0;
    uint8_t cardUid[4];

    // =================== BUCLE PRINCIPAL ===================
    while (1) {

        // LED azul: encendido solo cuando el sistema esta activo
        gpio_set_level(PIN_LED_BLUE,
            (currentState == STATE_ACTIVE) ? 1 : 0);

        // LED rojo: encendido en estado bloqueado (excepto durante parpadeo)
        if (!isBlinking) {
            gpio_set_level(PIN_LED_RED,
                (currentState == STATE_LOCKED) ? 1 : 0);
        }

        // En modo activo: mostrar mensaje BLE y hora en el LCD
        if (currentState == STATE_ACTIVE) {

            rtc_get_time(&hours, &minutes, &seconds);

            lcd_set_cursor_position(0, 0);
            lcd_print_padded(bleMessage);

            char timeStr[17];
            snprintf(timeStr, sizeof(timeStr),
                     "%02d:%02d:%02d", hours, minutes, seconds);

            lcd_set_cursor_position(0, 1);
            lcd_print_padded(timeStr);
        }

        ESP_LOGI(LOG_TAG, "Esperando tarjeta...");

        // Verificar si hay una tarjeta RFID presente
        if (rfid_detect_card()) {

            if (rfid_read_card_uid(cardUid)) {

                ESP_LOGI(LOG_TAG, "UID leido: %02X:%02X:%02X:%02X",
                         cardUid[0], cardUid[1], cardUid[2], cardUid[3]);

                // Comparar UID leido con el autorizado
                bool isAuthorized = true;
                for (int i = 0; i < 4; i++) {
                    if (cardUid[i] != authorizedUid[i]) {
                        isAuthorized = false;
                    }
                }

                lcd_clear_screen();

                if (isAuthorized) {

                    if (currentState == STATE_LOCKED) {

                        // ---------- ABRIR SESION ----------
                        currentState = STATE_ACTIVE;

                        // Apagar LED rojo (ya no esta bloqueado)
                        gpio_set_level(PIN_LED_RED, 0);

                        // Encender LED verde, se apagara en 1 segundo
                        gpio_set_level(PIN_LED_GREEN, 1);
                        esp_timer_start_once(timerGreenOff, 1000000);

                        // Activar buzzer, se apagara en 150 ms
                        gpio_set_level(PIN_BUZZER, 1);
                        esp_timer_start_once(timerBuzzerOff, 150000);

                        // Mostrar acceso concedido y hora en LCD
                        rtc_get_time(&hours, &minutes, &seconds);

                        lcd_set_cursor_position(0, 0);
                        lcd_print_padded("Acceso concedido");

                        char timeStr[17];
                        snprintf(timeStr, sizeof(timeStr),
                                 "%02d:%02d:%02d", hours, minutes, seconds);

                        lcd_set_cursor_position(0, 1);
                        lcd_print_padded(timeStr);

                    } else {

                        // ---------- CERRAR SESION ----------
                        currentState = STATE_LOCKED;

                        // Encender LED rojo (vuelve a estado bloqueado)
                        gpio_set_level(PIN_LED_RED, 1);

                        // Activar buzzer, se apagara en 200 ms
                        gpio_set_level(PIN_BUZZER, 1);
                        esp_timer_start_once(timerBuzzerOff, 200000);

                        // Mostrar mensaje temporal de cierre de sesion
                        lcd_set_cursor_position(0, 0);
                        lcd_print_padded("Sesion cerrada");
                        lcd_set_cursor_position(0, 1);
                        lcd_print_padded("Hasta pronto!");

                        // Restaurar LCD al mensaje bloqueado en 1.5 s
                        esp_timer_start_once(timerLcdReset, 1500000);
                    }

                } else {

                    // ---------- ACCESO DENEGADO ----------
                    currentState = STATE_LOCKED;

                    // Iniciar parpadeo del LED rojo
                    gpio_set_level(PIN_LED_RED, 0);
                    isBlinking   = true;
                    blinkCounter = 0;
                    esp_timer_start_periodic(timerRedBlink, 300000);

                    // Activar buzzer, se apagara en 2 segundos
                    gpio_set_level(PIN_BUZZER, 1);
                    esp_timer_start_once(timerBuzzerOff, 2000000);

                    // Mostrar acceso denegado en LCD
                    lcd_set_cursor_position(0, 0);
                    lcd_print_padded("Acceso denegado");
                    lcd_set_cursor_position(0, 1);
                    lcd_print_padded("UID invalido");

                    // Restaurar LCD al mensaje bloqueado en 3.5 s
                    esp_timer_start_once(timerLcdReset, 3500000);
                }

                // Espera anti-rebote tras lectura de tarjeta
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }

        // Periodo del bucle principal
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}