#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <TFT_eSPI.h>

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

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

      if (separatorIndex1 != -1 && separatorIndex2 != -1 && separatorIndex3 != -1) {
        String distanceStr = value.substring(0, separatorIndex1);
        String heightStr = value.substring(separatorIndex1 + 1, separatorIndex2);
        String leftRightStr = value.substring(separatorIndex2 + 1, separatorIndex3);

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

void setup() {
  Serial.begin(115200);

  tft.init();
  tft.initDMA();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

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
    pSensorCharacteristic->setValue(String(value).c_str());
    pSensorCharacteristic->notify();
    value++;
    Serial.print("New value notified: ");
    Serial.println(value);
    delay(3000);  // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
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