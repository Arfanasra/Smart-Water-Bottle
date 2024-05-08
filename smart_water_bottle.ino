#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_AHTX0.h>


Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
Adafruit_AHTX0 aht;

int tof_offset = 30;  // 20cm
float temp_offset = 1.0;

#define BLYNK_TEMPLATE_ID "TMPL6zycz2Iin"
#define BLYNK_TEMPLATE_NAME "Smart Water Bottle"
#define BLYNK_AUTH_TOKEN "f3dZwCFPmFVHYJlICUdujySfqIGjL2B6"

/* Comment this out to disable prints and save space */
// #define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "wifi";
char pass[] = "password";

BlynkTimer timer;

String orientation;
float temperatureValue;

int distance;

String name;
int age, height, weight;

BLYNK_WRITE(V3) {
   name = param.asStr();
}

void getAllReading() {

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  sensors_event_t a, g, temp, humidity, temperature;

  mpu.getEvent(&a, &g, &temp);
  aht.getEvent(&humidity, &temperature);

  Serial.print(F("Distance (cm): "));
  distance = 25 - (measure.RangeMilliMeter - tof_offset) / 10;
  Serial.println(distance);

  // Serial.print("Accelerometer ");
  // Serial.print("X: ");
  // Serial.print(a.acceleration.x, 1);
  // Serial.print(" m/s^2, ");
  // Serial.print("Y: ");
  // Serial.print(a.acceleration.y, 1);
  // Serial.print(" m/s^2, ");
  // Serial.print("Z: ");
  // Serial.print(a.acceleration.z, 1);
  // Serial.println(" m/s^2");

  // Check orientation based on accelerometer data
  if (abs(a.acceleration.z - 9.8) < 1.0) {
    Serial.println("Horizontal orientation");
    orientation = "Horizontal";
  } else if (abs(a.acceleration.x - 9.8) < 1.0 || abs(a.acceleration.y - 9.8) < 1.0) {
    Serial.println("Vertical orientation");
    orientation = "Vertical";

  } else {
    Serial.println("Unknown orientation");
    orientation = "Unknown";
  }

  // Serial.print("Gyroscope ");
  // Serial.print("X: ");
  // Serial.print(g.gyro.x, 1);
  // Serial.print(" rps, ");
  // Serial.print("Y: ");
  // Serial.print(g.gyro.y, 1);
  // Serial.print(" rps, ");
  // Serial.print("Z: ");
  // Serial.print(g.gyro.z, 1);
  // Serial.println(" rps");

  Serial.print("Temperature: ");
  temperatureValue = (temperature.temperature - temp_offset);
  Serial.print(temperatureValue);
  Serial.print(" C ");
  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println("% rH");

  Serial.println("");
  // delay(500);
}



void myTimerEvent() {
  getAllReading();

  Blynk.virtualWrite(V0, orientation);
  Blynk.virtualWrite(V1, temperatureValue);
  Blynk.virtualWrite(V2, distance);

  // Blynk.virtualWrite(V3, name);

}

void setup() {
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(1000L, myTimerEvent);

  Wire.begin();

  if (!lox.begin()) {
    Serial.println(F("Failed to initialize VL53L0X sensor!"));
    while (true)
      ;
  }

  if (!mpu.begin()) {
    Serial.println(F("Failed to initialize MPU6050 sensor!"));
    while (true)
      ;
  }

  if (!aht.begin()) {
    Serial.println(F("Failed to initialize AHT10 sensor!"));
    while (true)
      ;
  }
}

void loop() {
  Blynk.run();
  timer.run();
}
