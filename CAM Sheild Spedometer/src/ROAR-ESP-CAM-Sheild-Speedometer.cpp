#include <Arduino.h>
#include <PID_v1.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ESP32Servo.h>
#include <EEPROM.h> 


// #define SERVICE_UUID                  "19B10010-E8F2-537E-4F6C-D104768A1214"
// #define SPEED_CHARACTERISTIC_UUID     "19B10011-E8F2-537E-4F6C-D104768A1214"
// #define STEERING_CHARACTERISTIC_UUID  "19B10015-E8F2-537E-4F6C-D104768A1214"
// #define NAME_CHARACTERISTIC_UUID      "19B10012-E8F2-537E-4F6C-D104768A1214"
// #define OVERRIDE_CHARACTERISTIC_UUID  "19B10013-E8F2-537E-4F6C-D104768A1214"
// #define PID_CHARACTERISTIC_UUID       "19B10014-E8F2-537E-4F6C-D104768A1214"

#define SERVICE_UUID                  "19B10010-E8F2-537E-4F6C-D104768A1214"
#define CONTROL_CHAR_UUID             "19B10011-E8F2-537E-4F6C-D104768A1214"
#define VELOCITY_CHAR_UUID            "19B10011-E8F2-537E-4F6C-D104768A1215"
#define PID_KValues_UUID              "19B10011-E8F2-537E-4F6C-D104768A1216"
#define NAME_CHARACTERISTIC_UUID      "19B10014-E8F2-537E-4F6C-D104768A1214"
#define OVERRIDE_CHARACTERISTIC_UUID  "19B10015-E8F2-537E-4F6C-D104768A1214"

#define THROTTLE_PIN 14
#define STEERING_PIN 2
#define FLASH_LED_PIN 4
#define RED_LED_PIN 33
#define OVERRIDE_PIN 15
#define SPEEDOMETER_PIN1 13 // update these pins
#define SPEEDOMETER_PIN2 12 // update these pins


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

Servo throttleServo;
Servo steeringServo;

//==============Speed and PID related===============================
unsigned int deltaTime = 0; // track time between speed sensor readings
double distofRotation = 0.079; // Measure distance of one rotation of drive shaft
bool newValue = 0; // Has speed sensor picked up new value
double target_speed = 0;
double throttle_output = 1500;
double speed_mps = 0; //Speed in meters per second
double Kp=60, Ki=30, Kd=3; //Default Kp, Ki, Kd parameters
bool direction = 1; // 1 is forward 0 is backward
int maxThrot = 1800;
int minThrot = 1500;
PID speedPID(&speed_mps, &throttle_output, &target_speed, Kp, Ki, Kd, DIRECT);
//==================================================================

bool isFlashLightOn = false;
bool isRedLEDOn = false;
unsigned long lastREDLEDToggleTime = 0;  // will store last time LED was updated

bool deviceConnected = false;


class ControlCharCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      // let's not change this since it might break other people's workign code. 
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        // turn buffer into a String object
        String argument = String(value.c_str());
        // terminate the string
        char buf[value.length()] = "\0";
        argument.toCharArray(buf, argument.length());

        // the input is going to be in the format of (1500, 1500)
        // so find the delimiter "," and then exclude the first "("
        // then extract the throttle and steering values
        char *token = strtok(buf, ",");
        if (token[0] == HANDSHAKE_START) {
            if (token != NULL) {
              unsigned int curr_throttle_read = atoi(token + 1);
              if (curr_throttle_read >= 1000 and curr_throttle_read <= 2000) {
                // ws_throttle_read = curr_throttle_read;
                target_speed = ((float)curr_throttle_read - 1500) / 100;
                Serial.println(target_speed);
              } 
            }
            token = strtok(NULL, ",");
            if (token != NULL) {
              unsigned int curr_steering_read = atoi(token);
              if (curr_steering_read >= 1000 and curr_steering_read <= 2000) {
                ws_steering_read = curr_steering_read;
                Serial.println(ws_steering_read);
              }
            }
        }
      }
    }
};

class ConfigCharCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      // We first get values from the buffer
      std::string value = pCharacteristic->getValue();
      if (value.length() == 12) {
        // the value is in little endian, 
        // Arduino is also in little endian
        // so 1.00 in hex should be 0x3f80000 in big endian
        // and we will have received 00 00 08 f3, so we simply map this to a float using the below struct
        float kp; float kd; float ki;

        union tmp {
            byte b[4];
            float fval;
        } t;
        t.b[0] = value[0]; t.b[1] = value[1]; t.b[2] = value[2]; t.b[3] = value[3]; kp = t.fval;
        t.b[0] = value[4]; t.b[1] = value[5]; t.b[2] = value[6]; t.b[3] = value[7]; kd = t.fval;
        t.b[0] = value[8]; t.b[1] = value[9]; t.b[2] = value[10]; t.b[3] = value[11]; ki = t.fval;
        if (kp < 1000 && kp >= 0) Kp = kp;
        if (kd < 100 && kd >= 0) Kd = kd;
        if (ki < 100 && ki >= 0) Ki = ki;
        speedPID.SetTunings(Kp, Ki, Kd);
        // print it out for display
        Serial.print(kp); Serial.print(","); Serial.print(kd); Serial.print(","); Serial.print(ki); Serial.println();
      }
    }
};

class VelocityCharCallback: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic){
      float my_velocity_reading = (float)(speed_mps * direction);
      pCharacteristic->setValue(my_velocity_reading);
      Serial.println("Sent velocity reading");
      Serial.println(my_velocity_reading);
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
      digitalWrite(OVERRIDE_PIN, !digitalRead(OVERRIDE_PIN));
      Serial.println("Override activated");
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
      throttle_output = 1500;
      target_speed = 0;
    }
};

void Rev_Interrupt_forward_only_deprecated (){
  unsigned long timeNow = millis();
  static unsigned long lastTime = 0;
  deltaTime = timeNow - lastTime;
  lastTime = timeNow;
  newValue = 1;
}

void Rev_Interrupt (){
  unsigned long timeNow = millis();
  static unsigned long lastTime = 0;
  deltaTime = timeNow - lastTime;
  lastTime = timeNow;
  if(digitalRead(SPEEDOMETER_PIN2))
    direction = 1;
  else
    direction = 0;
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
    
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                           CONTROL_CHAR_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE | 
                                           BLECharacteristic::PROPERTY_WRITE_NR);
    pCharacteristic->setCallbacks(new ControlCharCallback());

    BLECharacteristic *nameChangeCharacteristic = pService->createCharacteristic(
                                           NAME_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE | 
                                           BLECharacteristic::PROPERTY_WRITE_NR);                                                                    
    nameChangeCharacteristic->setCallbacks(new nameChangeCallbacks());

    BLECharacteristic *overrideCharacteristic = pService->createCharacteristic(
                                       OVERRIDE_CHARACTERISTIC_UUID,
                                       BLECharacteristic::PROPERTY_READ |
                                       BLECharacteristic::PROPERTY_WRITE | 
                                       BLECharacteristic::PROPERTY_WRITE_NR);
    overrideCharacteristic->setCallbacks(new overrideCallbacks());

    BLECharacteristic *pidChar = pService->createCharacteristic(
                                           PID_KValues_UUID,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_WRITE_NR);                     
    pidChar->setCallbacks(new ConfigCharCallback());

    BLECharacteristic *vCharacteristic = pService->createCharacteristic(
                                            VELOCITY_CHAR_UUID,
                                            BLECharacteristic::PROPERTY_READ);
    vCharacteristic->setCallbacks(new VelocityCharCallback());
    
    pService->start();

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
  return;
}

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
  pinMode(SPEEDOMETER_PIN1, INPUT_PULLUP);
  pinMode(SPEEDOMETER_PIN2, INPUT_PULLUP);
  attachInterrupt(SPEEDOMETER_PIN1, Rev_Interrupt, FALLING);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(FLASH_LED_PIN, OUTPUT);
  pinMode(OVERRIDE_PIN, OUTPUT);
  EEPROM.begin(EEPROMSIZE);

  if(EEPROM.read(0) == 1){
    Serial.println("Loading name from memory");
    loadNewDeviceName();
  }
  setupServo();
  // Serial.println("1");
  setupBLE();
  // Serial.println("2");

  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(20); // 2 seemed to work well w/o BLE. trying 20...
  speedPID.SetOutputLimits(minThrot, maxThrot);

}

void loop() {
  static unsigned int timeout_total = 500;
  static unsigned long lastTime = 0;
 // static int speedMovingAvg = 0;
 
  if (deviceConnected == false) {
    blinkRedLED();
    writeToServo(1500, 1500);
  } else {
    digitalWrite(RED_LED_PIN, LOW);

    if(newValue){
      noInterrupts();
      unsigned int tempdeltaTime = deltaTime;
      interrupts();
      speed_mps = 1000 * distofRotation / (double) tempdeltaTime;
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
    
    if(target_speed < 0.5){
      target_speed = 0;
    }

    speedPID.Compute();

    if(throttle_output > maxThrot) throttle_output = maxThrot;
    if(throttle_output < minThrot) throttle_output = minThrot;  
    if(target_speed == 0) throttle_output = 1500;
    writeToServo(throttle_output, ws_steering_read);
    // Serial.println("");
    // Serial.println(speed_mps);
    // Serial.println(throttle_output);
  }
}