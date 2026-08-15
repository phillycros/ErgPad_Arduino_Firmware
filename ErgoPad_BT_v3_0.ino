//ERGO PAD v1.0 Firmware
// Test change for GitHub

#include <ArduinoBLE.h>

// ---------------------------------------------------------
// BLE services
// ---------------------------------------------------------

BLEService ledService(
  "19B10000-E8F2-537E-4F6C-D104768A1214"
);

BLEService forceService(
  "91b71286-ffd7-409a-a6bd-0457a59f0677"
);

// ---------------------------------------------------------
// BLE characteristics
// ---------------------------------------------------------

BLEByteCharacteristic switchCharacteristic(
  "19B10001-E8F2-537E-4F6C-D104768A1214",
  BLERead | BLEWrite
);

BLEIntCharacteristic force1Characteristic(
  "5ae0d501-6fb7-4b9e-9a98-25d50500f2d5",
  BLERead | BLEWrite | BLENotify
);

BLEIntCharacteristic force2Characteristic(
  "5ae0d502-6fb7-4b9e-9a98-25d50500f2d5",
  BLERead | BLEWrite | BLENotify
);

BLEIntCharacteristic force3Characteristic(
  "5ae0d503-6fb7-4b9e-9a98-25d50500f2d5",
  BLERead | BLEWrite | BLENotify
);

BLEIntCharacteristic force4Characteristic(
  "5ae0d504-6fb7-4b9e-9a98-25d50500f2d5",
  BLERead | BLEWrite | BLENotify
);

BLEIntCharacteristic force5Characteristic(
  "5ae0d505-6fb7-4b9e-9a98-25d50500f2d5",
  BLERead | BLEWrite | BLENotify
);

// ---------------------------------------------------------
// FSR sensor pins
// ---------------------------------------------------------

const int fsrAnalogPin1 = A1;
const int fsrAnalogPin2 = A2;
const int fsrAnalogPin3 = A3;
const int fsrAnalogPin4 = A4;
const int fsrAnalogPin5 = A5;

// ---------------------------------------------------------
// Sensor and smoothing settings
// ---------------------------------------------------------

// Read all five sensors every 10 milliseconds.
const unsigned long SENSOR_READ_INTERVAL_MS = 10;

// Send all five smoothed values every 2 seconds.
const unsigned long SENSOR_SEND_INTERVAL_MS = 2000;

// Default 10-bit ADC range is 0 to 1023.
const int SENSOR_THRESHOLD = 100;
const int SENSOR_MAXIMUM = 1023;

// Exponential moving average smoothing factor.
const float SMOOTHING_ALPHA = 0.15;

// ---------------------------------------------------------
// Timing variables
// ---------------------------------------------------------

unsigned long previousSensorReadTime = 0;
unsigned long previousSensorSendTime = 0;

// ---------------------------------------------------------
// Smoothed sensor values
// ---------------------------------------------------------

float smoothedSensor1 = 0.0;
float smoothedSensor2 = 0.0;
float smoothedSensor3 = 0.0;
float smoothedSensor4 = 0.0;
float smoothedSensor5 = 0.0;

// Tracks whether each smoothed value has been initialized.
bool sensor1Initialized = false;
bool sensor2Initialized = false;
bool sensor3Initialized = false;
bool sensor4Initialized = false;
bool sensor5Initialized = false;

// ---------------------------------------------------------
// Function declarations
// ---------------------------------------------------------

int readForceSensor(int analogPin);

float updateSmoothedValue(
  float currentSmoothedValue,
  int newReading,
  bool &initialized
);

void readAndSmoothSensors();
void sendSmoothedSensorValues();
void resetSmoothedSensorValues();

// ---------------------------------------------------------
// Setup
// ---------------------------------------------------------

void setup() {

  Serial.begin(9600);

  pinMode(LEDG, OUTPUT);
  digitalWrite(LEDG, HIGH);

  if (!BLE.begin()) {

    Serial.println(
      "Starting Bluetooth Low Energy module failed!"
    );

    while (1) {
      // Stop execution if BLE initialization fails.
    }
  }

  BLE.setDeviceName("Crosland Force");
  BLE.setLocalName("Crosland Force");

  BLE.setAdvertisedService(forceService);

  // Add LED characteristic.
  ledService.addCharacteristic(
    switchCharacteristic
  );

  // Add the five force sensor characteristics.
  forceService.addCharacteristic(
    force1Characteristic
  );

  forceService.addCharacteristic(
    force2Characteristic
  );

  forceService.addCharacteristic(
    force3Characteristic
  );

  forceService.addCharacteristic(
    force4Characteristic
  );

  forceService.addCharacteristic(
    force5Characteristic
  );

  // Add both BLE services.
  BLE.addService(ledService);
  BLE.addService(forceService);

  // Set initial characteristic values.
  switchCharacteristic.writeValue(0);

  force1Characteristic.writeValue(0);
  force2Characteristic.writeValue(0);
  force3Characteristic.writeValue(0);
  force4Characteristic.writeValue(0);
  force5Characteristic.writeValue(0);

  BLE.advertise();

  Serial.println("Advertising");
  
}

// ---------------------------------------------------------
// Main loop
// ---------------------------------------------------------

void loop() {

  BLEDevice central = BLE.central();

  if (central) {

    Serial.print("Connected to central now: ");
    Serial.println(central.address());

    resetSmoothedSensorValues();

    previousSensorReadTime = millis();
    previousSensorSendTime = millis();

    while (central.connected()) {


      // ---------------------------------------------------
      // Handle LED switch commands from the app
      // ---------------------------------------------------

      if (switchCharacteristic.written()) {

        if (switchCharacteristic.value()) {

          Serial.println("LED on");
          digitalWrite(LEDG, LOW);

        } else {

          Serial.println("LED off");
          digitalWrite(LEDG, HIGH);
        }
      }

      unsigned long currentTime = millis();

      // ---------------------------------------------------
      // Read and smooth all five sensors every 10 ms
      // ---------------------------------------------------

      if (
        currentTime - previousSensorReadTime >=
        SENSOR_READ_INTERVAL_MS
      ) {

        previousSensorReadTime = currentTime;

        readAndSmoothSensors();
      }

      // ---------------------------------------------------
      // Send all five smoothed values every 2 seconds
      // ---------------------------------------------------

      if (
        currentTime - previousSensorSendTime >=
        SENSOR_SEND_INTERVAL_MS
      ) {

        previousSensorSendTime = currentTime;

        sendSmoothedSensorValues();

        /*
         * Do not reset the smoothed values here.
         * This allows smoothing to continue between sends.
         */
      }
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());

    resetSmoothedSensorValues();

    digitalWrite(LEDG, HIGH);
  }
}

// ---------------------------------------------------------
// Read one FSR sensor using the default 10-bit ADC
// ---------------------------------------------------------

int readForceSensor(int analogPin) {

  int sensorReading = analogRead(analogPin);

  if (sensorReading < SENSOR_THRESHOLD) {
    return 0;
  }

  if (sensorReading > SENSOR_MAXIMUM) {
    return SENSOR_MAXIMUM;
  }

  return sensorReading;
}

// ---------------------------------------------------------
// Update one exponential moving average
// ---------------------------------------------------------

float updateSmoothedValue(
  float currentSmoothedValue,
  int newReading,
  bool &initialized
) {

  /*
   * Use the first reading as the starting value so the
   * smoothed value does not slowly ramp upward from zero.
   */
  if (!initialized) {

    initialized = true;

    return float(newReading);
  }

  return (
    SMOOTHING_ALPHA * float(newReading)
  ) + (
    (1.0 - SMOOTHING_ALPHA) *
    currentSmoothedValue
  );
}

// ---------------------------------------------------------
// Read and smooth all five sensors
// ---------------------------------------------------------

void readAndSmoothSensors() {

  int fsr1Reading = readForceSensor(fsrAnalogPin1);
  int fsr2Reading = readForceSensor(fsrAnalogPin2);
  int fsr3Reading = readForceSensor(fsrAnalogPin3);
  int fsr4Reading = readForceSensor(fsrAnalogPin4);
  int fsr5Reading = readForceSensor(fsrAnalogPin5);

  smoothedSensor1 = updateSmoothedValue(
    smoothedSensor1,
    fsr1Reading,
    sensor1Initialized
  );

  smoothedSensor2 = updateSmoothedValue(
    smoothedSensor2,
    fsr2Reading,
    sensor2Initialized
  );

  smoothedSensor3 = updateSmoothedValue(
    smoothedSensor3,
    fsr3Reading,
    sensor3Initialized
  );

  smoothedSensor4 = updateSmoothedValue(
    smoothedSensor4,
    fsr4Reading,
    sensor4Initialized
  );

  smoothedSensor5 = updateSmoothedValue(
    smoothedSensor5,
    fsr5Reading,
    sensor5Initialized
  );
}

// ---------------------------------------------------------
// Send all five smoothed sensor values
// ---------------------------------------------------------

void sendSmoothedSensorValues() {

  int sensor1Value = round(smoothedSensor1);
  int sensor2Value = round(smoothedSensor2);
  int sensor3Value = round(smoothedSensor3);
  int sensor4Value = round(smoothedSensor4);
  int sensor5Value = round(smoothedSensor5);

  Serial.println("-----------------------------");

  Serial.print("Sensor 1 smoothed: ");
  Serial.println(sensor1Value);

  Serial.print("Sensor 2 smoothed: ");
  Serial.println(sensor2Value);

  Serial.print("Sensor 3 smoothed: ");
  Serial.println(sensor3Value);

  Serial.print("Sensor 4 smoothed: ");
  Serial.println(sensor4Value);

  Serial.print("Sensor 5 smoothed: ");
  Serial.println(sensor5Value);

  // Send all five values consecutively using existing UUIDs.
  force1Characteristic.writeValue(sensor1Value);
  force2Characteristic.writeValue(sensor2Value);
  force3Characteristic.writeValue(sensor3Value);
  force4Characteristic.writeValue(sensor4Value);
  force5Characteristic.writeValue(sensor5Value);

  Serial.println("All five smoothed sensor values sent");
}

// ---------------------------------------------------------
// Reset smoothing when BLE connects or disconnects
// ---------------------------------------------------------

void resetSmoothedSensorValues() {

  smoothedSensor1 = 0.0;
  smoothedSensor2 = 0.0;
  smoothedSensor3 = 0.0;
  smoothedSensor4 = 0.0;
  smoothedSensor5 = 0.0;

  sensor1Initialized = false;
  sensor2Initialized = false;
  sensor3Initialized = false;
  sensor4Initialized = false;
  sensor5Initialized = false;
}