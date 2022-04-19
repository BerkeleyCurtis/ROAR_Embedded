#define DEFINITIONS_H

//===========Define only one of these two as 1 and the other as zero========
#define ESP32_CAM 0
#define CUSTOM_ROAR_PCB 0 
#define HARD_ML 1
//==========================================================================

#if ((ESP32_CAM + CUSTOM_ROAR_PCB + HARD_ML) == 1) // only one board can be defined

#define SERVICE_UUID                  "19B10010-E8F2-537E-4F6C-D104768A1214"
#define CONTROL_CHAR_UUID             "19B10011-E8F2-537E-4F6C-D104768A1214"
#define VELOCITY_RETURN_UUID          "19B10011-E8F2-537E-4F6C-D104768A1215"
#define PID_KValues_UUID              "19B10011-E8F2-537E-4F6C-D104768A1216"
#define THROTVELOCITY_RETURN_UUID     "19B10011-E8F2-537E-4F6C-D104768A1217"
#define NAME_CHARACTERISTIC_UUID      "19B10012-E8F2-537E-4F6C-D104768A1214"
#define OVERRIDE_CHARACTERISTIC_UUID  "19B10015-E8F2-537E-4F6C-D104768A1214"


#if ESP32_CAM
#define THROTTLE_PIN 14
#define STEERING_PIN 2
#define FLASH_LED_PIN 4
#define RED_LED_PIN 33
#define OVERRIDE_PIN 15
#define SPEEDOMETER_PIN1 13
#define SPEEDOMETER_PIN2 12
#define echoPin 255 //Not implimented
#define trigPin 255 //Not implimented
#define IR 255 // Infrared Analog pin
#endif

#if CUSTOM_ROAR_PCB
#define THROTTLE_PIN 17
#define STEERING_PIN 16
#define FLASH_LED_PIN 255
#define RED_LED_PIN 32
#define OVERRIDE_PIN 18
#define SPEEDOMETER_PIN1 26
#define SPEEDOMETER_PIN2 27
#define echoPin 255 //Not implimented
#define trigPin 255 //Not implimented
#define IR 255 // Infrared Analog pin
#endif

#if HARD_ML
#define THROTTLE_PIN 17
#define STEERING_PIN 19
#define FLASH_LED_PIN 255
#define RED_LED_PIN 32
#define OVERRIDE_PIN 255
#define SPEEDOMETER_PIN1 26
#define SPEEDOMETER_PIN2 27
#define echoPin 18 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 4 //attach pin D3 Arduino to pin Trig of HC-SR04
#define IR_R 32 // Infrared Analog pin
#define IR_L 33 // Infrared Analog pin
#define RESET 35 // To reset the loop
#endif

#else 
#error "One and only one board must be set equal to 1 at the top of ROAR_Definitions.h"
#endif