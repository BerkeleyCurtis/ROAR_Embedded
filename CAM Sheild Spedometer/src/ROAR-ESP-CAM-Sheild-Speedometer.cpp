#include <Arduino.h>
#include <PID_v1.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ESP32Servo.h>
#include <EEPROM.h> 
//#include <movingAvg.h>

#define SERVICE_UUID                  "19B10010-E8F2-537E-4F6C-D104768A1214"
#define CONTROL_CHARACTERISTIC_UUID   "19B10011-E8F2-537E-4F6C-D104768A1214"
#define NAME_CHARACTERISTIC_UUID      "19B10012-E8F2-537E-4F6C-D104768A1214"
#define OVERRIDE_CHARACTERISTIC_UUID  "19B10013-E8F2-537E-4F6C-D104768A1214"

#define THROTTLE_PIN 14
#define STEERING_PIN 2
#define FLASH_LED_PIN 4
#define RED_LED_PIN 33
#define OVERRIDE_PIN 15
#define SPEEDOMETER_PIN 13


#define OFFSET 1 // Start SSID after identifier byte
const int MAXNAME = 50; // MAX characters in BLE name
#define EEPROMSIZE (MAXNAME + OFFSET) // Total EEPROM size

char DeviceName[MAXNAME] = "ROAR default";

const char HANDSHAKE_START = '(';
const unsigned long redLEDToggleDuration = 500; // ms

volatile int32_t ws_throttle_read = 1500;
volatile int32_t ws_steering_read = 1500;
bool isForwardState = true; // car is currently in forward state.
unsigned int latest_throttle = 1500; // set to neutral by default
unsigned int latest_steering = 1500; // set to neutral by default

unsigned int deltaTime = 0; // track time between speed sensor readings
double distofRotation = 0.079; // Measure distance of one rotation of drive shaft
bool newValue = 0; // Has speed sensor picked up new value


//movingAvg speedSensor(4);
Servo throttleServo;
Servo steeringServo;

double target_speed = 0;
double temp_throttle = 1500;
double speed_mps = 0;
double Kp=50, Ki=30, Kd=3;
int maxThrot = 1800;
int minThrot = 1500;
PID speedPID(&speed_mps, &temp_throttle, &target_speed, Kp, Ki, Kd, DIRECT);

bool isFlashLightOn = false;
bool isRedLEDOn = false;
unsigned long lastREDLEDToggleTime = 0;  // will store last time LED was updated

bool deviceConnected = false;

class ControlsCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *controlCharacteristic) {
      std::string value = controlCharacteristic->getValue();
      if (value.length() > 0) {
        String argument = String(value.c_str());
        Serial.print("the argument is ");
        Serial.println(argument);
        char buf[value.length()] = "\0";
        argument.toCharArray(buf, argument.length());
        char *token = strtok(buf, ",");
        if (token[0] == HANDSHAKE_START) {
            if (token != NULL) {
              unsigned int curr_throttle_read = atoi(token + 1);
              if (curr_throttle_read >= 1000 and curr_throttle_read <= 2000) {
                if (curr_throttle_read == 1500) {
                  curr_throttle_read = 1510;
                }
                target_speed = ((float)curr_throttle_read - 1500) / 100;
                Serial.print("target speed is ");
                Serial.println(target_speed);
              } 
            }
            token = strtok(NULL, ",");
            if (token != NULL) {
              unsigned int curr_steering_read = atoi(token);
              if (curr_steering_read >= 1000 and curr_steering_read <= 2000) {
                ws_steering_read = curr_steering_read;
              }
            }
        }

        Serial.print(ws_throttle_read);
        Serial.print(",");
        Serial.println(ws_steering_read);
        
      }
    }

    void onRead(BLECharacteristic *controlCharacteristic, esp_ble_gatts_cb_param_t* param){
      controlCharacteristic->setValue("hello world");
    }
};

class nameChangeCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *nameChangeCharacteristic) {
      std::string value = nameChangeCharacteristic->getValue();
      if (value.length() > 0) {
        int rxLength = value.length();
        for (char i = 0; i < rxLength; i++) {
            if(i > MAXNAME){
              Serial.print("Your BLE is too long for my little memory :)");
              break;
            }
            EEPROM.write(i + OFFSET, value[i]); // Store NAME in permanent memory
            DeviceName[i] = value[i];
            Serial.print(value[i]);
        }
        Serial.println("");
        for(char i = rxLength; i < MAXNAME; i++) {
          EEPROM.write(i + OFFSET, 0); // clear the rest of the EEPROM
          DeviceName[i] = 0;
        }
        EEPROM.write(0, 1); // indicate stored value
        EEPROM.commit(); // Saves new name
      }
    }

    void onRead(BLECharacteristic *nameChangeCharacteristic, esp_ble_gatts_cb_param_t* param){
      nameChangeCharacteristic->setValue("hello world");
    }
};

class overrideCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *overrideCharacteristic) {
      std::string value = overrideCharacteristic->getValue();
      if (value.length() > 0) {
        digitalWrite(OVERRIDE_PIN, !digitalRead(OVERRIDE_PIN));
      }
    }

    void onRead(BLECharacteristic *overrideCharacteristic, esp_ble_gatts_cb_param_t* param){
      overrideCharacteristic->setValue("hello world");
    }
};


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Connected");
      digitalWrite(OVERRIDE_PIN, HIGH);
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Disconnected");
      BLEDevice::startAdvertising();
      digitalWrite(OVERRIDE_PIN, LOW);
      ws_throttle_read = 1500;
      ws_steering_read = 1500;
    }
};



void Rev_Interrupt (){
  unsigned long timeNow = millis();
  static unsigned long lastTime = 0;
  deltaTime = timeNow - lastTime;
  lastTime = timeNow;
  newValue = 1;
}

void loadNewDeviceName() {
  int i = 0;
  //DeviceName[MAXNAME] = {0};
  while(EEPROM.read(OFFSET + i)) {
    if(i > MAXNAME){
        Serial.print("Your entered name is too long for my little memory :)");
        break;
    }
    DeviceName[i] = EEPROM.read(OFFSET + i);
    i++;
  }
  while(i < MAXNAME){
    DeviceName[i] = 0;
    i++;
  }
  Serial.println(DeviceName);
}

void setupBLE() {
    BLEDevice::init(DeviceName);
    BLEServer *pServer = BLEDevice::createServer();
  
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLECharacteristic *controlCharacteristic = pService->createCharacteristic(
                                           CONTROL_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE | 
                                           BLECharacteristic::PROPERTY_WRITE_NR
                                         );

    controlCharacteristic->setCallbacks(new ControlsCallbacks());
    
    BLECharacteristic *nameChangeCharacteristic = pService->createCharacteristic(
                                           NAME_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE | 
                                           BLECharacteristic::PROPERTY_WRITE_NR
                                         );
                                         
    nameChangeCharacteristic->setCallbacks(new nameChangeCallbacks());

    BLECharacteristic *overrideCharacteristic = pService->createCharacteristic(
                                       OVERRIDE_CHARACTERISTIC_UUID,
                                       BLECharacteristic::PROPERTY_READ |
                                       BLECharacteristic::PROPERTY_WRITE | 
                                       BLECharacteristic::PROPERTY_WRITE_NR
                                     );

    overrideCharacteristic->setCallbacks(new overrideCallbacks());
    
    pService->start();
  
//    BLEAdvertising *pAdvertising = pServer->getAdvertising();
//    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
//    pAdvertising->setMinPreferred(0x12);
//    pAdvertising->start();

    BLEDevice::startAdvertising();
    Serial.println("BLE Device Started");
}

void setupServo() {
  ESP32PWM::timerCount[0]=4;
  ESP32PWM::timerCount[1]=4;
  throttleServo.setPeriodHertz(50);
  throttleServo.attach(THROTTLE_PIN, 1000, 2000);
  steeringServo.setPeriodHertz(50);    // standard 50 hz servo
  steeringServo.attach(STEERING_PIN, 1000, 2000); // attaches the servo on pin (whatever you assign)
}

void checkServo() {
  if (throttleServo.attached() == false) {
    throttleServo.attach(THROTTLE_PIN);
  }
  if (steeringServo.attached() == false) {
    steeringServo.attach(STEERING_PIN);
  }
}

void writeToServo(unsigned int throttle, unsigned int steering) {
  checkServo(); // prevent servo from detaching
  latest_throttle = throttle;
  latest_steering = steering;

  throttleServo.writeMicroseconds(latest_throttle);
  steeringServo.writeMicroseconds(latest_steering);
//  writeToSerial(latest_throttle, latest_steering);
}

void writeToSerial(unsigned int throttle, unsigned int steering) {
  /*
   * Write to Serial
   */
  Serial.print(throttle);
  Serial.print(",");
  Serial.println(steering);
}

void ensureSmoothBackTransition() {
  if (isForwardState and latest_throttle < 1500) {
    writeToServo(1500, latest_steering);
    delay(100);
    writeToServo(1450, latest_steering);
    delay(100);
    writeToServo(1500,latest_steering);
    delay(100);
    isForwardState = false;
  } else if (latest_throttle >= 1500) {
    isForwardState = true;
  }
}


// Utility functions
void blinkFlashlight() {
  if (isFlashLightOn) {
    ledcWrite(FLASH_LED_PIN, 0);
    isFlashLightOn = false;
  } else {
    ledcWrite(FLASH_LED_PIN, 100);
    isFlashLightOn = true;
  }
}


void blinkRedLED() {
  unsigned long currentMillis = millis();
  if (isRedLEDOn && (currentMillis - lastREDLEDToggleTime >= redLEDToggleDuration)) {
    digitalWrite(RED_LED_PIN, HIGH);
    isRedLEDOn = false;
    lastREDLEDToggleTime = currentMillis;
  } else if (isRedLEDOn == false && (currentMillis -lastREDLEDToggleTime >= redLEDToggleDuration)) {
    digitalWrite(RED_LED_PIN, LOW);
    isRedLEDOn = true;
    lastREDLEDToggleTime = currentMillis;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SPEEDOMETER_PIN, INPUT_PULLUP);
  attachInterrupt(SPEEDOMETER_PIN,Rev_Interrupt,FALLING);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(FLASH_LED_PIN, OUTPUT);
  pinMode(OVERRIDE_PIN, OUTPUT);
  EEPROM.begin(EEPROMSIZE);

  if(EEPROM.read(0) == 1){
    Serial.println("Loading name from memory");
    loadNewDeviceName();
  }
  setupServo();
  setupBLE();

  //speedSensor.begin(); // moving average
  
  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(20); // 2 seemed to work well w/o BLE. trying 20...
  speedPID.SetOutputLimits(minThrot, maxThrot);

}

void loop() {
  const unsigned int timeout_total = 1000;
  static unsigned long lastTime = 0;
  static unsigned int throttle_output;
 // static int speedMovingAvg = 0;
  while(1){
    if (deviceConnected == false) {
      blinkRedLED();
      writeToServo(1500, 1500);
    } else {
      digitalWrite(RED_LED_PIN, LOW);

      if(newValue){
        noInterrupts();
        float tempdeltaTime = deltaTime;
        interrupts();
        speed_mps = 1000 * distofRotation / tempdeltaTime;
        if (speed_mps < 0) speed_mps = 0;
        newValue = 0;
        lastTime = millis(); // for 0mps timeout
      }
      else{
        unsigned long timeNow = millis();
        if((timeNow - lastTime) > timeout_total){
          speed_mps = 0;
          lastTime = timeNow;
        }
      }
      
      speedPID.Compute();
      if(target_speed < 0.1) temp_throttle = 1500;
      if(temp_throttle > maxThrot) temp_throttle = maxThrot;
      if(temp_throttle < minThrot) temp_throttle = minThrot;  
      throttle_output = temp_throttle;
      writeToServo(throttle_output, ws_steering_read);
      Serial.println("");
      Serial.println(speed_mps);
      Serial.println(throttle_output);
    }
  }
}