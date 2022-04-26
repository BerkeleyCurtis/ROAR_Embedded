#define GLOBALNAMES_H



volatile int32_t ws_throttle_read = 1500;
volatile int32_t ws_steering_read = 1500;
bool isForwardState = true; // car is currently in forward state.
unsigned int latest_throttle = 1500; // set to neutral by default
unsigned int latest_steering = 1500; // set to neutral by default

/* ------------------------------ Speed and PID ----------------------------- */
//user settable values...
int setDirection = 1; // set this from 1 to -1 if your hardware hall effect sensor is installed backwards. 
double distofRotation = (0.079 / 3.0); // Measured distance of one third rotation of drive shaft
double Kp=60, Ki=30, Kd=3; //Default Kp, Ki, Kd parameters
int maxThrot = 1800;
int minThrot = 1200;
unsigned int deltaTime = 0; // track time between speed sensor readings
bool newValue = 0; // Has speed sensor picked up new value
double target_speed = 0;
double throttle_output = 1500;
double speed_mps = 0; //Speed in meters per second
int direction = setDirection; // 1 is forward -1 is backward ??? system dependant
PID speedPID(&speed_mps, &throttle_output, &target_speed, Kp, Ki, Kd, DIRECT);

/* ------------------------------ Hard_ML Only ------------------------------ */
int reward = 0;