#include "Arduino.h"
#include "IP5306.h"

#define PIN_CHARGING 8
#define PIN_CAPACITY 9
#define PIN_LED A2
#define PIN_BTN D3

#define PIN_CTRL D5

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

  Serial.begin(9600);
  Serial.println("hello!");

  // these 3 GPIO pins have a 330R resistor tied to VCC
  // we pulse these low to bypass the low-load cutoff
  PORTB |= _BV(7) | _BV(6);
  PORTD |= _BV(2);
  DDRB |= _BV(7) | _BV(6);
  DDRD |= _BV(2);
}

void loop() {
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

  if (IP5306_GetPowerSource()) {
    digitalWrite(PIN_CHARGING, HIGH);
  } else {
    digitalWrite(PIN_CHARGING, LOW);
  }

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
}
