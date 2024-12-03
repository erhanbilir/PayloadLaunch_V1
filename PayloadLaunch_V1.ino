#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include "Arduino.h"
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SimpleKalmanFilter.h>
#include <SPI.h>
#include <SD.h>
#include <stdlib.h>

#define SEA_LEVEL_PRESSURE      911.0f
#define SD_PIN                  49
#define GPS_BAUD    9600
#define LORA_BAUD   9600
#define SERIAL_BAUD 9600
#define GPS_SERIAL  Serial2
#define LORA_SERIAL Serial1

TinyGPSPlus gps;
HardwareSerial &ss2 = GPS_SERIAL;

Adafruit_BME280 bme;
SimpleKalmanFilter altitudeKalmanFilter(1.5, 1.5, 0.01);

typedef struct {
  float altitude;
  double gpsAltitude;
  double lat;
  double lng;
  float temp;
  float humidity;
} Signal;

// Bellek boşaltma ve dinamik yapı işlemi
void freeSignal(Signal* signal) {
  free(signal);
}

unsigned long previousMillis = 0;
const long interval = 330;  // Saniyede 5 veri paketi için 200 ms

float altitude;

void setup() {
  Serial.begin(SERIAL_BAUD);
  GPS_SERIAL.begin(GPS_BAUD);
  delay(2000);

  if (!bme.begin(0x77)) {
    Serial.println("BME280 sensörü bulunamadı!");
    while (1);
  }

  LORA_SERIAL.begin(LORA_BAUD);
  delay(2000);

  Serial.println("SD kart başlatıldı.");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    Signal* data = (Signal*)malloc(sizeof(Signal));
    
    if (data == NULL) {
      Serial.println("Bellek ayrımı başarısız!");
      return;
    }
    altitude = bme.readAltitude(SEA_LEVEL_PRESSURE);
    data->altitude = altitudeKalmanFilter.updateEstimate(altitude);
    data->temp = bme.readTemperature();
    data->humidity = bme.readHumidity();
    data->lat = gps.location.lat();
    data->lng = gps.location.lng();
    data->gpsAltitude = gps.altitude.meters();

    String strData = String(data->altitude) + "," +
                     String(data->gpsAltitude) + "," +
                     String(data->lat, 6) + "," +
                     String(data->lng, 6) + "," +
                     String(data->temp) + "," +
                     String(data->humidity);
    
    String strPacket = "STR," + strData + ",END";

    LORA_SERIAL.println(strPacket);
    LORA_SERIAL.flush(); // Tamponu boşalt

    freeSignal(data);

    Serial.println("Veri gönderildi ve bellek boşaltıldı.");
    smartDelay(10);
  }
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss2.available() > 0)
      gps.encode(ss2.read());
  } while (millis() - start < ms);
}
