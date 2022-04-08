#include <PID_v1.h>
#include <EEPROM.h>
#include <ROAR_Servo.h>
#include <ROAR_BLE_funcs.h>
#ifndef DEFINITIONS_H 
#include <ROAR_Definitions.h>
#endif
#ifndef GLOBALNAMES_H
#include <ROAR_globalnames.h>
#endif


/* -------------------------------------------------------------------------- */
/*            Define which PCB you are using in ROAR_Definitions.h            */
/* -------------------------------------------------------------------------- */

#if ((ESP32_CAM + CUSTOM_ROAR_PCB) == 1) // only one board can be defined

/* -------------------------- Function Declarations ------------------------- */
long long pack(float lo, float hi);
void Rev_Interrupt_forward_only_deprecated ();
void Rev_Interrupt ();
void loadNewDeviceName();
void setupBLE();
void setupServo();
void checkServo();
void writeToServo(unsigned int throttle, unsigned int steering);
void ensureSmoothBackTransition();
void blinkFlashlight();
void blinkRedLED();

/* ---------------------- Main Setup and Loop functions --------------------- */

void setup() {
  Serial.begin(115200);
  pinMode(SPEEDOMETER_PIN1, INPUT_PULLUP);
  pinMode(SPEEDOMETER_PIN2, INPUT_PULLUP);
  attachInterrupt(SPEEDOMETER_PIN1, Rev_Interrupt, FALLING);
  pinMode(RED_LED_PIN, OUTPUT);
  #if ESP32_CAM
  pinMode(FLASH_LED_PIN, OUTPUT);
  #endif
  pinMode(OVERRIDE_PIN, OUTPUT);
  EEPROM.begin(EEPROMSIZE);

  if(EEPROM.read(0) == 1){
    Serial.println("Loading name from memory");
    loadNewDeviceName();
  }
  setupServo();
  setupBLE();

  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(10); // 20 Worked well. Changed to 10 for ROAR-FLOW
  speedPID.SetOutputLimits(1500, 1501);
  speedPID.SetOutputLimits(minThrot, maxThrot);
  //throttle_output = 1500;

}

void loop() {
  static unsigned long timeout_total = 200000; //200 if using millis()
  static unsigned long lastTime = 0;
  static unsigned long unscaleTime = 1000000; // 1000 if using millis() 
  static int print_timing = 1000;
  static int cycles = 0;

  if (deviceConnected == false) {
    blinkRedLED();
    writeToServo(1500, 1500);
  } else {
    digitalWrite(RED_LED_PIN, LOW);

    if(newValue){
      noInterrupts();
      unsigned long tempdeltaTime = deltaTime;
      interrupts();
      speed_mps = direction * (unscaleTime * distofRotation / (double) tempdeltaTime);
      newValue = 0; // clear new value 
      // lastTime = millis();
      lastTime = micros(); // for 0mps timeout
    }
    else{
      // unsigned long timeNow = millis();
      unsigned long timeNow = micros();
      if((timeNow - lastTime) > timeout_total){
        speed_mps = 0;
        lastTime = timeNow;
      }
    }
    
    if(target_speed < 0.2 && target_speed > -0.2){
      target_speed = 0;
    }

    speedPID.Compute();

    if(throttle_output > maxThrot) throttle_output = maxThrot;
    if(throttle_output < minThrot) throttle_output = minThrot;  
    //if(target_speed == 0) throttle_output = 1500;
    writeToServo(throttle_output, ws_steering_read);
    // Serial.println("");
    if(cycles > print_timing){
      Serial.println("Speed");
      Serial.println(speed_mps);
      Serial.println("Throttle");
      Serial.println(throttle_output);
    }
    cycles++;
  }
}

/* -------------------------- Function Descriptions ------------------------- */

void Rev_Interrupt_forward_only_deprecated (){
  unsigned long timeNow = millis();
  static unsigned long lastTime = 0;
  deltaTime = timeNow - lastTime;
  lastTime = timeNow;
  newValue = 1;
}

void Rev_Interrupt (){
  // static unsigned int start_offset = 10000; //should be 1 to 10 if using millis()
  // unsigned long timeNow = millis();
  unsigned long timeNow = micros();
  static unsigned long lastTime = 0;//timeNow - start_offset; //This causes jumpy acceleration I think
  if(timeNow <= lastTime){ // should take care of micros() rollover every 70 mins
    lastTime = timeNow;
    return;
  }
  deltaTime = timeNow - lastTime;
  lastTime = timeNow;
  if(digitalRead(SPEEDOMETER_PIN2))
    direction = setDirection; // forward or backward set at top
  else
    direction = -setDirection;
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

void blinkFlashlight() {
static bool isFlashLightOn = false;
  if (isFlashLightOn) {
    #if ESP32_CAM
    ledcWrite(FLASH_LED_PIN, 0);
    #endif 
    isFlashLightOn = false;
  } else {
    #if ESP32_CAM
    ledcWrite(FLASH_LED_PIN, 100);
    #endif
    isFlashLightOn = true;
  }
}

void blinkRedLED() {
  static bool isRedLEDOn = false;
  static unsigned long lastREDLEDToggleTime = 0;
  const static unsigned long redLEDToggleDuration = 500; // ms
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

#endif // this ends the preprocessor #if statement from the very top
// when you defined which board you are using.