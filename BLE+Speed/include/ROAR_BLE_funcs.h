#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#ifndef DEFINITIONS_H 
#include <ROAR_Definitions.h>
#endif
#ifndef GLOBALNAMES_H
#include <ROAR_globalnames.h>
#endif


const char HANDSHAKE_START = '(';
bool deviceConnected = false;

void setupBLE();
long long pack(float lo, float hi);

//===============EEPROM Setup for name storage======================
#define OFFSET 1 // Start SSID after identifier byte
const int MAXNAME = 50; // MAX characters in BLE name
#define EEPROMSIZE (MAXNAME + OFFSET) // Total EEPROM size
char DeviceName[MAXNAME] = "ROAR default";
//==================================================================


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
                target_speed = ((float)curr_throttle_read - 1500.0) / 100.0;
                if(target_speed == 0){
                    speedPID.SetOutputLimits(1500, 1501);
                }
                else speedPID.SetOutputLimits(minThrot, maxThrot);
                //Serial.println(target_speed);
              } 
            }
            token = strtok(NULL, ",");
            if (token != NULL) {
              unsigned int curr_steering_read = atoi(token);
              if (curr_steering_read >= 1000 and curr_steering_read <= 2000) {
                ws_steering_read = curr_steering_read;
                // Serial.println(ws_steering_read);
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
        //values get passed in order kp, kd, ki
        t.b[0] = value[0]; t.b[1] = value[1]; t.b[2] = value[2]; t.b[3] = value[3]; kp = t.fval;
        t.b[0] = value[4]; t.b[1] = value[5]; t.b[2] = value[6]; t.b[3] = value[7]; kd = t.fval; 
        t.b[0] = value[8]; t.b[1] = value[9]; t.b[2] = value[10]; t.b[3] = value[11]; ki = t.fval;
        if (kp < 1000 && kp >= 0) Kp = kp;
        if (ki < 100 && ki >= 0) Ki = ki;
        if (kd < 100 && kd >= 0) Kd = kd;
        speedPID.SetTunings(Kp, Ki, Kd);
        // print it out for display
        Serial.print(kp); Serial.print(","); Serial.print(kd); Serial.print(","); Serial.print(ki); Serial.println();
      }
    }
};

#if HARD_ML
class rewardCallback: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic){
      float return_reward = float(reward);
      pCharacteristic->setValue(return_reward);
      Serial.print("Sent reward");
      Serial.println(return_reward);
    }
};
#endif

class VelocityCharCallback: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic){
      float my_velocity_reading = (float)(speed_mps);
      pCharacteristic->setValue(my_velocity_reading);
      // Serial.print("Sent velocity reading");
      // Serial.println(my_velocity_reading);
    }
};

// class ThrottleCharCallback: public BLECharacteristicCallbacks {
//     void onRead(BLECharacteristic *pCharacteristic){
//       float my_throttle_reading = (float)(throttle_output);
//       pCharacteristic->setValue(my_throttle_reading);
//       Serial.println("Sent throttle reading");
//       Serial.println(my_throttle_reading);
//     }
// };

class VelocityAndThrottleCharCallback: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic){
      long long velocityThrottleOut = pack((float)speed_mps, (float)throttle_output);
      /*WARNING Header file changed to support long long type in BLECharacteristic.cpp and .h
      Add the code below at line 708 of BLECharacteristic.cpp
      ==========================================================
      void BLECharacteristic::setValue(long long& data64) {
	    long long temp = data64;
	    setValue((uint8_t*)&temp, 8);
      } // setValue
      ==========================================================
      Add this line to BLECharacteristic.h after line 79:
      ==========================================================
      void setValue(long long& data64); 
      ==========================================================
      */
      pCharacteristic->setValue(velocityThrottleOut); 
      // Serial.println("Sent throttle reading");
      // Serial.println(velocityThrottleOut);
    }
};

class nameChangeCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *nameChangeCharacteristic) {
      std::string value = nameChangeCharacteristic->getValue();
      if (value.length() > 0) {
        int rxLength = value.length();
        for (uint8_t i = 0; i < rxLength; i++) {
            if(i > MAXNAME){
              Serial.print("Your BLE is too long for my little memory :)");
              break;
            }
            EEPROM.write(i + OFFSET, value[i]); // Store NAME in permanent memory
            DeviceName[i] = value[i];
            Serial.print(value[i]);
        }
        Serial.println("");
        for(uint8_t i = rxLength; i < MAXNAME; i++) {
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
                                            VELOCITY_RETURN_UUID,
                                            BLECharacteristic::PROPERTY_READ);
    vCharacteristic->setCallbacks(new VelocityCharCallback());

    BLECharacteristic *throtAndVelocityReturnCharacteristic = pService->createCharacteristic(
                                            THROTVELOCITY_RETURN_UUID,
                                            BLECharacteristic::PROPERTY_READ);
    throtAndVelocityReturnCharacteristic->setCallbacks(new VelocityAndThrottleCharCallback());
    
    #if HARD_ML
    BLECharacteristic *rewardCharacteristic = pService->createCharacteristic(
                                            REWARD_UUID,
                                            BLECharacteristic::PROPERTY_READ);
    rewardCharacteristic->setCallbacks(new rewardCallback());
    #endif

    pService->start();

    BLEDevice::startAdvertising();
    Serial.println("BLE Device Started");
}

// from https://stackoverflow.com/questions/4650489/packing-floats-into-a-long-long
long long pack(float lo, float hi) {
    assert(sizeof(float) == 4);
    assert(sizeof(long long) == 8);
    assert(CHAR_BIT == 8);

    uint32_t target;
    memcpy(&target, &lo, sizeof(target));
    long long result = target;
    memcpy(&target, &hi, sizeof(target));
    result += ((long long)target) << 32;
    return result;
}