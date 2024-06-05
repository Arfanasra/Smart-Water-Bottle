#define BLYNK_TEMPLATE_ID "TMPL6zycz2Iin"
#define BLYNK_TEMPLATE_NAME "Smart Water Bottle"
#define BLYNK_AUTH_TOKEN "f3dZwCFPmFVHYJlICUdujySfqIGjL2B6"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <BlynkSimpleEsp32.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
Adafruit_AHTX0 aht;

float bottle_height = 25.0;
float tof_offset;
float temp_offset = 1.0;
float upright_z_reference = 9.8;  // Default value assuming bottle is upright

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "wifi";
char pass[] = "password";

String orientation;
float orientationDegree;
float temperatureValue, humidityValue;
int waterLevel;
String name;
int age, height, weight;

int temperatureThreshold = 32, humidityThreshold = 80, waterBalanceThreshold = 100;

int reminderIntervalMin = 1, dataRefreshRateSec = 5;

unsigned long lastReminderTime = 0;     // Variable to store the last reminder time
unsigned long lastDataRefreshTime = 0;  // Variable to store the last data refresh time

BLYNK_WRITE(V3) {
  name = param.asStr();
  Serial.print("Name changed to ");
  Serial.println(name);
}

BLYNK_WRITE(V4) {
  age = param.asInt();
  Serial.print("Age changed to ");
  Serial.println(age);
}

BLYNK_WRITE(V5) {
  height = param.asInt();
  Serial.print("Height changed to ");
  Serial.println(height);
}

BLYNK_WRITE(V6) {
  weight = param.asInt();
  Serial.print("Weight changed to ");
  Serial.println(weight);
}

BLYNK_WRITE(V8) {
  if (param.asInt() == 1) {  // If button is pressed
    calibrateBottle();
  }
}

BLYNK_WRITE(V10) {
  upright_z_reference = param.asFloat();
  Serial.print("Retrieved calibrated upright Z reference from Blynk: ");
  Serial.println(upright_z_reference);
}

BLYNK_WRITE(V12) {
  tof_offset = param.asFloat();
  Serial.print("ToF Offset changed to ");
  Serial.println(tof_offset);
}

BLYNK_WRITE(V15) {
  temperatureThreshold = param.asInt();
  Serial.print("Temperature Threshold changed to ");
  Serial.println(temperatureThreshold);
}

BLYNK_WRITE(V16) {
  humidityThreshold = param.asInt();
  Serial.print("Humidity Threshold changed to ");
  Serial.println(humidityThreshold);
}

BLYNK_WRITE(V17) {
  waterBalanceThreshold = param.asInt();
  Serial.print("Water Balance Threshold changed to ");
  Serial.println(waterBalanceThreshold);
}

BLYNK_WRITE(V18) {
  reminderIntervalMin = param.asInt();
  Serial.print("Reminder Interval Min Threshold changed to ");
  Serial.println(reminderIntervalMin);
}

BLYNK_WRITE(V19) {
  dataRefreshRateSec = param.asInt();
  Serial.print("Data Refresh Rate Sec Threshold changed to ");
  Serial.println(dataRefreshRateSec);
}

void calibrateBottle() {
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  upright_z_reference = a.acceleration.z;
  Serial.print("Calibrated upright Z reference: ");
  Serial.println(upright_z_reference);

  Blynk.virtualWrite(V10, upright_z_reference);
}

void getAllReading() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  sensors_event_t a, g, temp, humidity, temperature;

  mpu.getEvent(&a, &g, &temp);
  aht.getEvent(&humidity, &temperature);

  Serial.print(F("Water Level (cm): "));
  waterLevel = bottle_height - (measure.RangeMilliMeter - tof_offset) / 10.0;
  Serial.println(waterLevel);

  // Calculate the tilt angles using the accelerometer data
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  // Adjust angle calculation using the calibrated upright Z reference
  orientationDegree = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  orientationDegree -= atan2(0, upright_z_reference) * 180.0 / PI;

  // Calculate individual axis angles
  float angleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float angleY = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  float angleZ = atan2(sqrt(accelX * accelX + accelY * accelY), accelZ) * 180.0 / PI;

  Serial.print("Orientation Angle: ");
  Serial.println(orientationDegree);

  // Print individual axis angles
  Serial.print("Angle X: ");
  Serial.print(angleX);
  Serial.print("°, Angle Y: ");
  Serial.print(angleY);
  Serial.print("°, Angle Z: ");
  Serial.print(angleZ);
  Serial.println("°");

  // Check orientation based on accelerometer data
  if (abs(accelZ - upright_z_reference) < 1.0) {
    Serial.println("Vertical orientation");
    orientation = "Vertical";
  } else if (abs(accelX) < 1.0 && abs(accelY) < 1.0) {
    Serial.println("Horizontal orientation");
    orientation = "Horizontal";
  } else {
    Serial.println("Unknown orientation");
    orientation = "Unknown";
  }

  Serial.print("Temperature: ");
  temperatureValue = (temperature.temperature - temp_offset);
  Serial.print(temperatureValue);
  Serial.print(" C ");
  Serial.print("Humidity: ");
  humidityValue = humidity.relative_humidity;
  Serial.print(humidityValue);
  Serial.println("% rH");

  Serial.println("Current Data Refresh Rate: Once every" +String(dataRefreshRateSec) + "Sec");
  Serial.println("Current Reminder Interval: Once every" +String(reminderIntervalMin) + "Min");

  

  Serial.println("");
}

void myTimerEvent() {
  getAllReading();

  if (orientation == "Vertical" && waterLevel > 0) {
    Blynk.virtualWrite(V2, waterLevel);
  }

  Blynk.virtualWrite(V0, orientation);
  Blynk.virtualWrite(V1, temperatureValue);
  Blynk.virtualWrite(V11, humidityValue);

  Blynk.virtualWrite(V9, orientationDegree);

  float recommendedIntake = recommendWaterIntake(age, height, weight, temperatureValue, humidityValue);

  // Update Blynk with the recommended intake
  Blynk.virtualWrite(V13, recommendedIntake);
  int waterBalance = waterLevel * 5.5 * 5.5;
  Blynk.virtualWrite(V14, waterBalance);

  if (waterBalance > 0 && waterBalance < waterBalanceThreshold) {
    Blynk.logEvent("user_refill", "Water level is low. " + String(waterBalance) + " ml");
    Serial.println("user_refill notification sent");
  }

  if (temperatureValue > temperatureThreshold) {
    Blynk.logEvent("drink_temperature", "Temperature is higher than usual stay hydrated. " + String(temperatureValue) + " °C");
    Serial.println("drink_temperature notification sent");
  }

  if (humidityValue > humidityThreshold) {
    Blynk.logEvent("drink_humidity", "Today is quite humid! " + String(humidityValue) + " %");
    Serial.println("drink_humidity notification sent");
  }
}

float recommendWaterIntake(int age, int height, int weight, float temperature, float humidity) {
  // Constants for water intake calculation
  const float baseIntake = 2000.0;       // Base water intake in milliliters
  const float temperatureFactor = 0.04;  // Increase intake by 40ml for every degree Celsius above 20°C
  const float humidityFactor = 0.02;     // Increase intake by 20ml for every 10% increase in humidity

  // Calculate water intake based on age, height, and weight
  float ageFactor = 0.0;
  if (age <= 30) {
    ageFactor = 40.0;  // Younger individuals generally need more water
  } else {
    ageFactor = 20.0;  // Older individuals may need slightly less water
  }

  // Calculate recommended water intake based on temperature and humidity
  float temperatureAdjustment = (temperature - 20.0) * temperatureFactor;
  float humidityAdjustment = (humidity - 50.0) / 10.0 * humidityFactor;

  // Calculate total recommended water intake
  float totalIntake = baseIntake + ageFactor + temperatureAdjustment + humidityAdjustment;

  // Adjust for height and weight
  totalIntake += height * 0.35;  // Add 0.35 ml for every cm of height
  totalIntake += weight * 0.03;  // Add 0.03 ml for every kg of weight

  return totalIntake;
}

void remindToDrink() {
  Blynk.logEvent("drink_reminder");
  Serial.println("drink_reminder notification sent");
}

void setup() {
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

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
    Serial.println(F("Failed to initialize AHT20 sensor!"));
    while (true)
      ;
  }
}

void loop() {
  Blynk.run();

  unsigned long currentTime = millis();

  if (currentTime - lastReminderTime >= reminderIntervalMin * 60 * 1000) {
    remindToDrink();
    lastReminderTime = currentTime;
  }

  if (currentTime - lastDataRefreshTime >= dataRefreshRateSec * 1000) {
    myTimerEvent();
    lastDataRefreshTime = currentTime;
  }
}
