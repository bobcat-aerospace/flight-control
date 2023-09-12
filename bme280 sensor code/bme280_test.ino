#include <Arduino.h>
#include <Bme280.h>

Bme280TwoWire sensor;

void setup() {
  Serial.begin(9600);
  Wire.begin(21, 22);

  Serial.println();

  sensor.begin(Bme280TwoWireAddress::Primary);
  sensor.setSettings(Bme280Settings::indoor());
}

void loop() {
  auto temperature = String(sensor.getTemperature()) + " °C";
  auto pressure = String(sensor.getPressure() / 100.0) + " hPa";

  String measurements = temperature + ", " + pressure;
  Serial.println(measurements);

}