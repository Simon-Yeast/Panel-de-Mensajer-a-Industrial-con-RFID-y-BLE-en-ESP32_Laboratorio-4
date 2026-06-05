# Panel de Mensajería Industrial con RFID y BLE en ESP32 — Laboratorio 4

Sistema embebido desarrollado sobre un ESP32 para implementar un panel de mensajería industrial con acceso restringido. El sistema utiliza autenticación mediante credenciales RFID para habilitar sesiones de trabajo y permite la recepción inalámbrica de mensajes mediante Bluetooth Low Energy (BLE), los cuales son mostrados en una pantalla LCD junto con la hora obtenida desde un RTC DS1307.

---

## Componentes

| Categoría                | Elemento                        | Cantidad |
| ------------------------ | ------------------------------- | -------- |
| Microcontrolador         | ESP32 (AZ-Delivery DevKit V4)   | 1        |
| Comunicación inalámbrica | BLE (Nordic UART Service - NUS) | 1        |
| Identificación           | RFID RC522                      | 1        |
| Visualización            | LCD 16x2                        | 1        |
| Reloj en tiempo real     | RTC DS1307                      | 1        |
| Indicadores              | LED rojo                        | 1        |
| Indicadores              | LED verde                       | 1        |
| Indicadores              | LED azul                        | 1        |
| Alarma sonora            | Buzzer                          | 1        |

---

## Funcionamiento general

El sistema opera mediante una máquina de estados compuesta por tres modos principales:

### Estado bloqueado

Al iniciar el sistema:

* El panel permanece restringido.
* El LCD muestra el mensaje de bloqueo.
* El LED rojo permanece encendido.
* Se ignoran todos los mensajes recibidos por BLE.

### Autenticación RFID

Cuando una credencial es acercada al lector:

#### Credencial autorizada

* Se concede el acceso.
* Se registra la hora actual.
* El LED verde se activa temporalmente.
* Se emite una señal sonora corta.
* El sistema cambia al estado activo.

#### Credencial no autorizada

* El acceso es rechazado.
* El LED rojo parpadea.
* Se emite una señal sonora de advertencia.
* El sistema permanece bloqueado.

### Estado activo

Una vez autenticado el usuario:

* Se habilita la comunicación BLE.
* El LED azul permanece encendido.
* Los mensajes recibidos desde la aplicación móvil se muestran en el LCD.
* La segunda línea del LCD muestra continuamente la hora actual obtenida desde el RTC.

Si no se han recibido mensajes durante la sesión, el sistema muestra el texto:

```text
Sin mensajes
```

---

## Comunicación BLE

El ESP32 opera como periférico BLE utilizando el servicio:

```text
Nordic UART Service (NUS)
```

Nombre anunciado:

```text
PanelHMI
```

Los mensajes enviados desde la aplicación nRF Connect son recibidos y desplegados en la pantalla LCD.

---

## Gestión de sesión

Una credencial RFID autorizada puede utilizarse nuevamente para cerrar la sesión activa.

Al cerrar sesión:

* Se deshabilita la recepción de mensajes BLE.
* El LED azul se apaga.
* El LED rojo vuelve a encenderse.
* El LCD regresa al estado bloqueado.
* Los nuevos mensajes recibidos son descartados hasta una nueva autenticación.

---

## Compilación y flasheo

Requisitos:

* PlatformIO instalado.
* Framework ESP-IDF.
* ESP32 conectado mediante USB.

```bash
# Compilar
pio run

# Flashear
pio run --target upload

# Monitor serial
pio device monitor
```

Configuración de PlatformIO:

```ini
[env:az-delivery-devkit-v4]
platform = espressif32
board = az-delivery-devkit-v4
framework = espidf
monitor_speed = 115200
```

---

## Estructura del proyecto

```text
.
├── src/
│   ├── CMakeLists.txt
│   └── main.c
├── include/
├── lib/
├── platformio.ini
├── CMakeLists.txt
├── sdkconfig.defaults
└── README.md
```

```text
   |\---/|
   | ,_, |
    \_`_/-..----.
 ___/ `   ' ,""+ \
(__...'   __\    |`.___.';
  (_,...'(_,.`__)/'.....+
```
