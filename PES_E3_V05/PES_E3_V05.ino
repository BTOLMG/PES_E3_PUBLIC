#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include <FastLED.h>

#define LED_PIN 13
#define NUM_LEDS 12

CRGB leds[NUM_LEDS];

VL53L0X sensor1;
VL53L0X sensor2;
int a;
int b;
const int pinSensor1 = 32;
const int pinSensor2 = 33;

MPU6050 mpu;
const int buzzer = 15;

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

TFT_eSPI tft = TFT_eSPI();

// UUIDs
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

// Grafiek variabelen
const int graphWidth = 150;  // Breedte van de grafiek
const int graphHeight = 100; // Hoogte van de grafiek
const int graphX = 20;       // X-positie van de grafiek
const int graphY = 20;       // Y-positie van de grafiek
const int maxDataPoints = 1000;
float dataPointsX[maxDataPoints];
float dataPointsY[maxDataPoints];
int dataIndex = 0;

// Projectielbeweging variabelen
const float g = 9.81;  // Zwaartekracht (m/s²)
float v0;              // Beginsnelheid (m/s)
float angle;           // Hoek (radialen)
float x0 = 0.0;        // Beginpositie x (m)
float _y0 = 0.500;     // Beginpositie y (m)

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

void drawGraph() {
  tft.fillRect(graphX, graphY, graphWidth, graphHeight, TFT_BLACK);  // Wis de oude grafiek
  tft.drawRect(graphX, graphY, graphWidth, graphHeight, TFT_WHITE);  // Teken de grafiekrand

  if (dataIndex > 1) {
    // Bepaal de maximale waarden voor x en y
    float maxX = 0;
    float maxY = 0;
    for (int i = 0; i < dataIndex; i++) {
      if (dataPointsX[i] > maxX) maxX = dataPointsX[i];
      if (dataPointsY[i] > maxY) maxY = dataPointsY[i];
    }

    // Schaal factoren
    float scaleX = graphWidth / maxX;
    float scaleY = graphHeight / maxY;

    Serial.println("GRAPH BEGINT");
    for (int i = 1; i < dataIndex; i++) {
      // Schaal de x- en y-waarden naar de grafiekgrootte
      int x1 = graphX + (int)(dataPointsX[i - 1] * scaleX);
      int y1 = graphY + graphHeight - (int)(dataPointsY[i - 1] * scaleY);
      int x2 = graphX + (int)(dataPointsX[i] * scaleX);
      int y2 = graphY + graphHeight - (int)(dataPointsY[i] * scaleY);

      // Zorg ervoor dat de lijnen binnen de grafiek blijven
      x1 = constrain(x1, graphX, graphX + graphWidth);
      y1 = constrain(y1, graphY, graphY + graphHeight);
      x2 = constrain(x2, graphX, graphX + graphWidth);
      y2 = constrain(y2, graphY, graphY + graphHeight);

      Serial.println((String)x1 + " " + (String)y1 + " " + (String)x2 + " " + (String)y2);
      tft.drawLine(x1, y1, x2, y2, TFT_GREEN);
    }
  }
}

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    String value = pLedCharacteristic->getValue();  // Get the value as String
    if (value.length() > 0) {
      Serial.print("Received data: ");
      Serial.println(value);

      // Parse the received data
      int separatorIndex1 = value.indexOf('&');
      int separatorIndex2 = value.indexOf('&', separatorIndex1 + 1);
      int separatorIndex3 = value.indexOf('&', separatorIndex2 + 1);
      int separatorIndex4 = value.indexOf('&', separatorIndex3 + 1);
      int separatorIndex5 = value.indexOf('&', separatorIndex4 + 1);
      int separatorIndex6 = value.indexOf('&', separatorIndex5 + 1);

      if (separatorIndex1 != -1 && separatorIndex2 != -1 && separatorIndex3 != -1) {
        String distanceStr = value.substring(0, separatorIndex1);
        String heightStr = value.substring(separatorIndex1 + 1, separatorIndex2);
        String leftRightStr = value.substring(separatorIndex2 + 1, separatorIndex3);
        String needToBuzzer = value.substring(separatorIndex3 + 1, separatorIndex4);
        String ledCommands = value.substring(separatorIndex4 + 1, separatorIndex5); // Bijv: "1:255,0,0|2:0,255,0|3:0,0,255"

        if(needToBuzzer == "1"){
          digitalWrite(buzzer, HIGH);
        }else{
          digitalWrite(buzzer, LOW);
        }

        // Split op '|' voor meerdere commando's
        int startIdx = 0;
        while (true) {
          int pipePos = ledCommands.indexOf('|', startIdx);
          String cmd = (pipePos == -1) ? ledCommands.substring(startIdx) : ledCommands.substring(startIdx, pipePos);

          if (cmd.length() > 0) {
            // Parse LED:R,G,B
            int colonPos = cmd.indexOf(':');
            if (colonPos == -1) return; // Fout: geen colon gevonden

            int ledNum = cmd.substring(0, colonPos).toInt();
            String colorStr = cmd.substring(colonPos + 1);

            // Split de rest in 4 delen (R,G,B,A)
            int parts[4];
            int lastIndex = 0;
            for (int i = 0; i < 4; i++) {
              int nextComma = colorStr.indexOf(',', lastIndex);
              if (nextComma == -1 && i < 3) return; // Fout: niet genoeg delen
              
              String partStr = colorStr.substring(lastIndex, nextComma != -1 ? nextComma : colorStr.length());
              parts[i] = i < 3 ? partStr.toInt() : (int)(partStr.toFloat() * 255); // Converteer opacity naar 0-255
              
              lastIndex = nextComma + 1;
            }

            // Zorg ervoor dat waarden binnen bereik zijn
            parts[0] = constrain(parts[0], 0, 255); // R
            parts[1] = constrain(parts[1], 0, 255); // G
            parts[2] = constrain(parts[2], 0, 255); // B
            parts[3] = constrain(parts[3], 0, 255); // A (nu 0-255)

            Serial.printf("LED %d: R=%d, G=%d, B=%d, A=%d\n", ledNum, parts[0], parts[1], parts[2], parts[3]);

            // Pas de kleur aan met opacity
            CRGB color = CRGB(
              (parts[0] * parts[3]) >> 8, // Gelijk aan * (opacity/255)
              (parts[1] * parts[3]) >> 8,
              (parts[2] * parts[3]) >> 8
            );
            ChangeLED(ledNum, color);
          }

          if (pipePos == -1) break;
          startIdx = pipePos + 1;
        }

        float afstand = distanceStr.toFloat();
        float hoogte = heightStr.toFloat();
        float linksRechts = leftRightStr.toFloat();

        Serial.print("Parsed values - Distance: ");
        Serial.print(afstand);
        Serial.print(", Height: ");
        Serial.print(hoogte);
        Serial.print(", Left/Right: ");
        Serial.println(linksRechts);

        // Bereken de beginsnelheid en hoek
        v0 = sqrt(afstand * g / sin(2 * angle));  // Vereenvoudigde formule voor maximale afstand
        angle = atan(hoogte / afstand);           // Hoek berekenen op basis van hoogte en afstand

        // Wis de oude data
        dataIndex = 0;
        float t = 0;  // Tijdvariabele
        float y = _y0;
        while (y>=0) {
          float x = x0 + v0 * cos(angle) * t;
          y = v0 * sin(angle) * t - 0.5 * g * t * t + _y0;

          if (y < 0) {
            dataPointsX[dataIndex] = x;
            dataPointsY[dataIndex] = 0;
            break;
          };  // Stop als het projectiel de grond raakt

          dataPointsX[dataIndex] = x;
          dataPointsY[dataIndex] = y;
          dataIndex++;

          t += 0.1;  // Verhoog de tijd met 0.1 seconde
        }

        // Teken de grafiek
        drawGraph();

        // Toon de waarden op het scherm
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(graphX + graphWidth + 10, graphY + 20);
        tft.print("X: ");
        tft.print(afstand);
        tft.print(" m");

        tft.setCursor(graphX + graphWidth + 10, graphY + 40);
        tft.print("Y: ");
        tft.print(hoogte);
        tft.print(" m");

        tft.setCursor(graphX + graphWidth + 10, graphY + 60);
        tft.print("L/R: ");
        tft.print(linksRechts);
        tft.print(" m");
      } else {
        Serial.println("Data niet correct binnengekregen");
      }
    }
  }
};

void ChangeLED(int led, CRGB color){
  if (led >= 0 && led < NUM_LEDS) {
    leds[led] = color;  // Specifieke LED
  }
  FastLED.show();  // Update de LEDs
}

// Initialize MPU6050
void initMPU6050() {
  Wire.begin();
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    while (1);
  }
  mpu.CalibrateGyro();
  Serial.println("MPU6050 initialized.");
}

void SensorSetup(){
  digitalWrite(pinSensor1, HIGH);
  delay(150);
  sensor1.init(true);

  delay(100);

  sensor1.setAddress((uint8_t)01);
  digitalWrite(pinSensor2, HIGH);
  delay(150);
  sensor2.init(true);
}

void setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  Serial.begin(115200);
  Wire.begin();

  initMPU6050();

  pinMode(pinSensor1,OUTPUT);
  pinMode(pinSensor2,OUTPUT);

  digitalWrite(pinSensor1, LOW);
  digitalWrite(pinSensor2, LOW);

  SensorSetup();

  sensor1.setTimeout(500);
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor1!");
    while (1) {}
  }

  sensor2.setTimeout(500);
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor2!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor1.startContinuous();
  sensor2.startContinuous();

  tft.init();
  tft.initDMA();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
    LED_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE);

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  // notify changed value
  if (deviceConnected) {
    String value = updateSensors();

    pSensorCharacteristic->setValue(String(value).c_str());
    pSensorCharacteristic->notify();
    Serial.print("New value notified: ");
    Serial.println(value);
    delay(300);  // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}

String updateSensors(){
  String resultString = "";

  String strDistance = updateDistance();
  String strTilt = updateTilt();

  resultString = strDistance + "&" + strTilt;

  return resultString;
}

String updateDistance(){
  String result = ""; 
  if (sensor1.timeoutOccurred()||sensor2.timeoutOccurred()) { 
    result = "TIMEOUT"; 
  }else{
    result = (String)(sensor1.readRangeContinuousMillimeters());
    result += "&"+(String)(sensor2.readRangeContinuousMillimeters());    
  }
  return result;
}

String updateTilt(){
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Normalize accelerometer values
  float normAccelX = ax / 16384.0; // 16384 is 1g for default sensitivity
  float normAccelY = ay / 16384.0;
  float normAccelZ = az / 16384.0;

  // Calculate pitch and roll
  float pitch = atan2(normAccelY, normAccelZ) * 180.0 / PI;
  float roll = atan2(-normAccelX, sqrt(normAccelY * normAccelY + normAccelZ * normAccelZ)) * 180.0 / PI;

  // Format the result as a string
  String result = String(pitch, 2);
  return result;
}