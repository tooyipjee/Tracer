#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServerAdv = NULL;
BLECharacteristic * pTxAccX;
BLECharacteristic * pTxAccY;
BLECharacteristic * pTxAccZ;
BLECharacteristic * pTxGyr;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t X = 0;
uint8_t Y = 0;
uint8_t Z = 0;
uint8_t gyr[3] = {32,64,96};
uint8_t gX = 0;
uint8_t gY = 0;
uint8_t gZ = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_ACCX_UUID "fb6cf981-31cc-4f36-af06-1f2f3e919840"
#define CHARACTERISTIC_ACCY_UUID "35b17f66-73d1-4c92-92f6-9032ef1987d3"
#define CHARACTERISTIC_ACCZ_UUID "3cab9341-e65b-46e9-83ed-c8a7f2f841c2"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServerAdv) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServerAdv) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};
void setupBluetoothAdv()
{
    // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  pServerAdv = BLEDevice::createServer();
  pServerAdv->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServerAdv->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxAccX = pService->createCharacteristic(
										CHARACTERISTIC_ACCX_UUID,
										BLECharacteristic::PROPERTY_NOTIFY
									);                     
  pTxAccX->addDescriptor(new BLE2902());
    // Create a BLE Characteristic
  pTxAccY = pService->createCharacteristic(
										CHARACTERISTIC_ACCY_UUID,
										BLECharacteristic::PROPERTY_NOTIFY
									);                     
  pTxAccY->addDescriptor(new BLE2902());
    // Create a BLE Characteristic
  pTxAccZ = pService->createCharacteristic(
										CHARACTERISTIC_ACCZ_UUID,
										BLECharacteristic::PROPERTY_NOTIFY
									);                     
  pTxAccZ->addDescriptor(new BLE2902());
   // Create a BLE Characteristic
  pTxGyr = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);                     
  pTxGyr->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();
  // Start advertising
  pServerAdv->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
};

void updateBluetoothAdv(float X, float Y, float Z)
{
      if (deviceConnected) {
        pTxAccX->setValue(X);
        pTxAccX->notify();
        pTxAccY->setValue(Y);
        pTxAccY->notify();
        pTxAccZ->setValue(Z);
        pTxAccZ->notify();
        // X++;Y++;Z++;
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServerAdv->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}