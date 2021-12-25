#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

#include "IP5306.h"

#define PIN_CHARGING 8
#define PIN_CAPACITY 9
#define PIN_LED A2
#define PIN_BTN 3

#define PIN_CTRL 5

#define LEN_MAX_UART_CMD 8

// give an unconnected pin as RX, dont care because unused
SoftwareSerial haptic_ctrl(A1, PIN_CTRL);

struct {
  uint8_t min_ldr;
  uint8_t max_ldr;
  uint8_t min_motor;
  uint8_t max_motor;
} params;

void setup() {
  Wire.begin();
  // disable auto load detect
  IP5306_SetPowerOnLoadEnabled(0);
  IP5306_SetLightLoadShutdownTime(3);
  // charge current 450mA
  IP5306_SetVinCurrent(4);
  IP5306_SetChargingFullStopVoltage(0);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_CHARGING, OUTPUT);
  pinMode(PIN_CAPACITY, OUTPUT);

  pinMode(PIN_CTRL, OUTPUT);

  Serial.begin(1200);
  haptic_ctrl.begin(1200);
  Serial.println("hello!");

  // these 3 GPIO pins have a 330R resistor tied to VCC
  // we pulse these low to bypass the low-load cutoff
  PORTB |= _BV(7) | _BV(6);
  PORTD |= _BV(2);
  DDRB |= _BV(7) | _BV(6);
  DDRD |= _BV(2);

  if (EEPROM.read(0) != 0x7A) {
    // initialise to sane defaults
    EEPROM.write(0, 0x7A);
    EEPROM.write(1, 0);
    EEPROM.write(2, 255);
    EEPROM.write(3, 0);
    EEPROM.write(4, 255);

    params.min_ldr = 0;
    params.max_ldr = 255;
    params.min_motor = 0;
    params.max_motor = 255;
  } else {
    params.min_ldr = EEPROM.read(1);
    params.max_ldr = EEPROM.read(2);
    params.min_motor = EEPROM.read(3);
    params.max_motor = EEPROM.read(4);
  }
}

void loop() {
  static char uart_buf[LEN_MAX_UART_CMD];
  static uint8_t uart_buf_size = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' && uart_buf_size != 2) {
      if (uart_buf[0] == 'C') {
        switch (uart_buf[1]) {
          case 'l':
            params.min_ldr = uart_buf[2];
            EEPROM.write(1, uart_buf[2]);
            break;
          case 'L':
            params.max_ldr = uart_buf[2];
            EEPROM.write(2, uart_buf[2]);
            break;
          case 'm':
            params.min_motor = uart_buf[2];
            EEPROM.write(3, uart_buf[2]);
            break;
          case 'M':
            params.max_motor = uart_buf[2];
            EEPROM.write(4, uart_buf[2]);
            break;
        }
      } else if (uart_buf[0] == 'c') {
        unsigned int param;
        uart_buf[uart_buf_size] = 0;
        if (sscanf(&uart_buf[2], "%u", &param) == 1) {
          if (param <= 255) {
            bool ok = true;
            switch (uart_buf[1]) {
              case 'l':
                params.min_ldr = param;
                EEPROM.write(1, param);
                break;
              case 'L':
                params.max_ldr = param;
                EEPROM.write(2, param);
                break;
              case 'm':
                params.min_motor = param;
                EEPROM.write(3, param);
                break;
              case 'M':
                params.max_motor = param;
                EEPROM.write(4, param);
                break;
              default:
                ok = false;
                break;
            }

            if (ok) {
              Serial.println("ok");
            }
          } else {
            Serial.println("fail, param > 255");
          }
        } else {
          Serial.println("fail, sscanf");
        }
      } else if (uart_buf[0] == 'R') {
        Serial.print("Cl");
        Serial.write(params.min_ldr);
        Serial.println();
        Serial.print("CL");
        Serial.write(params.max_ldr);
        Serial.println();
        Serial.print("Cm");
        Serial.write(params.min_motor);
        Serial.println();
        Serial.print("CM");
        Serial.write(params.max_motor);
        Serial.println();
      } else if (uart_buf[0] == 'r') {
        Serial.print("Min LDR: ");
        Serial.print(params.min_ldr);
        Serial.println();
        Serial.print("Max LDR: ");
        Serial.print(params.max_ldr);
        Serial.println();
        Serial.print("Min Motor: ");
        Serial.print(params.min_motor);
        Serial.println();
        Serial.print("Max Motor: ");
        Serial.print(params.max_motor);
        Serial.println();
      }
      uart_buf_size = 0;
    } else {
      uart_buf[uart_buf_size++] = c;
      if (uart_buf_size >= LEN_MAX_UART_CMD) {
        uart_buf_size = 0;
      }
    }
  }

  // pulse pins to drain 45mA of current
  if (millis() % 60000 < 500) {
    digitalWrite(PIN_LED, HIGH);
    PORTB &= ~(_BV(7) | _BV(6));
    PORTD &= ~_BV(2);
  } else {
    digitalWrite(PIN_LED, LOW);
    PORTB |= _BV(7) | _BV(6);
    PORTD |= _BV(2);
  }

  bool charging = IP5306_GetPowerSource();
  bool full = IP5306_GetBatteryFull();
  digitalWrite(PIN_CHARGING, charging && !full);

  byte level = IP5306_GetLevelLeds();
  if (level <= 1) {
    digitalWrite(PIN_CAPACITY, LOW);
  } else if (level <= 3) {
    digitalWrite(PIN_CAPACITY, (millis() % 500) < 250);
  } else if (level <= 7) {
    digitalWrite(PIN_CAPACITY, (millis() % 2000) < 1000);
  } else if (level == 15) {
    digitalWrite(PIN_CAPACITY, HIGH);
  }

  static uint32_t last_write = 0;
  if (millis() - last_write > 500) {
    last_write = millis();

    haptic_ctrl.print("Cl");
    haptic_ctrl.write(params.min_ldr);
    haptic_ctrl.write('\n');
    delay(10);
    haptic_ctrl.print("CL");
    haptic_ctrl.write(params.max_ldr);
    haptic_ctrl.write('\n');
    delay(10);
    haptic_ctrl.print("Cm");
    haptic_ctrl.write(params.min_motor);
    haptic_ctrl.write('\n');
    delay(10);
    haptic_ctrl.print("CM");
    if (charging) {
      haptic_ctrl.write((uint8_t)0);
    } else {
      haptic_ctrl.write(params.max_motor);
    }
    haptic_ctrl.write('\n');
    delay(10);
  }
}
