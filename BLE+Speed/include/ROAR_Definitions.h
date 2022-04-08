#define DEFINITIONS_H

//===========Define only one of these two as 1 and the other as zero========
#define ESP32_CAM 0
#define CUSTOM_ROAR_PCB 1 
//==========================================================================

#if ((ESP32_CAM + CUSTOM_ROAR_PCB) == 1) // only one board can be defined

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
#endif

#if CUSTOM_ROAR_PCB
#define THROTTLE_PIN 17
#define STEERING_PIN 16
#define FLASH_LED_PIN "ERROR"
#define RED_LED_PIN 32
#define OVERRIDE_PIN 18
#define SPEEDOMETER_PIN1 26
#define SPEEDOMETER_PIN2 27
#endif

#else 
#error "One and only one board must be set equal to 1 at the top of ROAR_Definitions.h"
#endif