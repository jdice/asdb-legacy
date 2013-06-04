// GPS
TinyGPS gps;
unsigned long gpsLockAge;
float currentLat, currentLon, goalLat, goalLon;
float currentHeading, currentGPSHeading, goalHeading, adjustedGoalHeading;
float initialLat, initialLon, distanceToGoal;
const int lockLED =  13;
boolean gpsLock = false;
char waypointBehavior = 'l'; // r means reverse at end of goalpoints, l means loop through goalpoints, s means stop at final goalpoint
int pointid = 0; // id of current waypoint, managed by the car
boolean waypointDirection = true; // true means traverse waypoints in order, false reverses the order, managed by the car

// Steering
boolean steeringEnabled = true;
PWMServo steeringServo;
const int centerSteeringTuningValue = 90; // lower means more right, higher means more left
const int fullRightSteeringTuningValue = 58;
const int fullLeftSteeringTuningValue = 116;
int steeringAngle = centerSteeringTuningValue;
float turnAngle = 0;
const int turnAngleHistorySize = 25;
float turnAngleHistory[turnAngleHistorySize];
float turnAngleHistoryAverage;
const float derivativeTuning = 0.9;
const float integralTuning = 0.1;


// Drivetrain
PWMServo speedControl;
const unsigned long brakeDuration = 1600;
float goalSpeed = 6.5; //miles per hour
float speedTolerance = 0.5; // +/- miles per hour
boolean drivetrainEnabled = false;
boolean adjustMySpeed = true;
boolean slowMode = true;
boolean brakeMode = false;
float currentSpeed = 0;
int brakeSpeedValue = 110;
int neutralSpeedValue = 100;
int slowSpeedValue = 89;
int fastSpeedValue = 25;
int speedValue = 100;
int adjustedSpeedValue = speedValue;
float maxSpeed;

// Bumpers
boolean leftBumper, rightBumper;
boolean bumperStop = false;
const int leftBumperPin = 25;
const int rightBumperPin = 23;

// Headlight
const int headlightPin = 53;
boolean headlightEnabled = false;

// Rangefinder
const int rangefinderPin = 0;
float rangefinderValue = 0;
boolean rangefinderStop = false;

// XBee
boolean xbeeStop = true;
boolean stopOverrides = false;

// IMU Gyro
float imuEulerAngles[3];
float imuValues[6];
const float imuAngularTrust = 5; // in degrees per second
FreeSixIMU imuSixDOF = FreeSixIMU();

// Magnetometer Compass
HMC5883L compass;
float declinationAngle = -0.033452;

// Drive Battery Monitor
const int driveBatteryMonitorPin = 10;
const float driveBatteryVoltageRatio = 2.175;
float driveBatteryVoltage = 0;

// Process Tracking and aJSON
int loopNumber = 0;
const boolean jsonPrint = true;
unsigned long timer100, timer500, brakeTimer;
char* debugInfo;
aJsonStream serial_stream(&Serial3);
