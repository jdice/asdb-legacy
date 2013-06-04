import processing.serial.*;
import org.json.*;
import org.json.JSONObject;
import java.util.*;
import googlemapper.*;
import java.lang.Math;
import controlP5.*;

// Application Settings
boolean fullScreen = true;
boolean doSerial = true;
boolean doTestPointMove = false;
float defaultPointSize = 5.0;

// Clipboard Setup
ClipHelper clippy = new ClipHelper();

// Make a Queue for history location data
String inString;
String theData;
JSONObject jsonData;
String debugInfo;
Serial myPort;  // Create object from Serial class

// location data for drawing
XYPt initialXY = new XYPt(1000, 1000, defaultPointSize);
XYPt currentXY = new XYPt(600, 700, defaultPointSize);
XYPt goalXY = new XYPt(600, 1300, defaultPointSize);
XYPt viewXY = initialXY;
XYPt cameraXY = initialXY;
XYPt mouseXY = initialXY;
float mouseSceneX, mouseSceneY;
LatLonPt currentLocation, goalLocation, mouseLocation;

// Camera Setup
boolean cameraFollowMouse = false;
boolean debugVisible = false;
float cameraHeight;
double cameraMoveAmount = 100.0; // suit to taste
PImage mapImg;
GoogleMapper gMapper;
PFont uiFont;
int textLineSize = 26;
int windowWidth, windowHeight;
int maxGoalSpeed = 15;

// controlP5 objects
ControlP5 cp5;
float testTheta = 0;
Numberbox guiCurrentLat, guiCurrentLon, guiGoalLat, guiGoalLon, guiMouseLat, guiMouseLon, guiServo, guiESC, guiRangefinder, guiDriveBattery, guiLoopNumber, guiCurrentSpeed, guiMaxSpeed, guiDistanceToGoal;
Toggle guiDrivetrain, guiSteering, guiHeadlight, guiGPSLock, guiFollowMouse, guiLeftBumper, guiRightBumper, guiManualAutomatic, guiDebugVisible;
Button guiCopyPosition;
DropdownList guiWaypointBehavior;
Knob guiCurrentHeading, guiGoalHeading;
Slider guiGoalSpeed;

// data from car
boolean stopOverrides = false, steeringEnabled = false, xbeeStop = true, drivetrainEnabled = false, headlightEnabled = false, adjustMySpeed = false, gpsLock = false; // myStatus
int loopNumber = 0; //myStatus
double rangefinderValue = 0, driveBatteryVoltage = 0; //mySensors
boolean leftBumper = false, rightBumper = false; //mySensors
double currentLat = 39.24745, currentLon = -94.40990, goalLat = 39.24745, goalLon = -94.40990, distanceToGoal = 0; //myLocation
double currentHeading = 0, goalHeading = 0, turnAngle = 0, currentSpeed = 0, goalSpeed = 0, maxSpeed = 0; //mySpeed
int steeringAngle = 0, adjustedSpeedValue = 0; // steering and esc values
double imuYaw = 0, imuPitch = 0, imuRoll = 0; //myImu

boolean jsonError = true;

void setup(){
  frame.setBackground(new java.awt.Color(0,0,0));
  if(fullScreen){
    windowWidth = displayWidth;
    windowHeight = displayHeight;
  }else{
    windowWidth = 668;
    windowHeight = 690;
  }
  size(windowWidth, windowHeight, P3D);
  frameRate(60);
  uiFont = loadFont("uiFont.vlw");
  if(doSerial){
    String serialPort = "COM5"; //Serial.list()[0];
    myPort = new Serial(this, serialPort, 57600);
    println("Connecting serial " + serialPort);
    delay(100);
    myPort.clear();
    myPort.bufferUntil('~');
  }
  int mapImgSize = 2000;
  double mapCenterLat = 39.24630;
  double mapCenterLon = -94.40987;
  int zoomLevel = 20;
  String mapType = GoogleMapper.MAPTYPE_HYBRID;
  int mapWidth = mapImgSize;
  int mapHeight = mapImgSize;
  gMapper = new GoogleMapper(mapCenterLat,mapCenterLon,zoomLevel,mapType,mapWidth,mapHeight);
  LatLonPt.setMapper(gMapper);
  XYPt.setMapper(gMapper);
  mapImg = loadImage("data/map2.jpg");
  cameraHeight = (windowHeight/2.0) / tan(PI/6);
  
  guiSetup();
}

void draw(){
  hint(ENABLE_DEPTH_TEST);
  
  if(jsonData != null){
    processJSONData();
  }else{
    if(doTestPointMove){
      testPointMove();
    }
  }
  
  cameraUpdate();
  guiUpdate();
  clear();
  textFont(uiFont);
  pushMatrix();
    perspective(PI/3.0,(float)width/height,1/(cameraHeight*2),cameraHeight*2);
    camera(
      (float) cameraXY.getX(), // Camera X position
      (float) cameraXY.getY(), // Camera Y position
      cameraHeight, // Camera Z position
      (float) cameraXY.getX(), (float) cameraXY.getY(), 0, //looking at X,Y,Z
      0, 1, 0 // Screen-Up vector
    );
    updateMouseXY();
    image(mapImg,0,0);
    drawPoint(color(0,255,0),goalXY);
    drawPoint(color(255,255,0),currentXY);
    drawPoint(color(255,0,0),mouseXY);
    //drawPoint(color(0,0,255),viewXY);
    //drawPoint(color(0,255,255),cameraXY);
  popMatrix();
  camera(); // resets view
  hint(DISABLE_DEPTH_TEST); // Draw UI and "static" 2D elements after this line
  pushMatrix();
    if(debugInfo != null){
      text("Debug: " + debugInfo, textLineSize, height-(textLineSize*1));
    }else{
      text("Debug: ", textLineSize, height-(textLineSize*1));
    }
    translate(width-180,height-180,0);
    ortho();
    rotateY(HALF_PI);
    rotateX(-radians((float) currentHeading));
    rotateZ(-radians((float) imuPitch));
    rotateY(radians((float) imuRoll));
    cone(0,0,40,180);
  popMatrix();
}

void processJSONData(){
  try{
    JSONObject statusData = jsonData.optJSONObject("status");
      stopOverrides = statusData.optBoolean("stopOverrides", stopOverrides);
      steeringEnabled = statusData.optBoolean("steeringEnabled", steeringEnabled);
      xbeeStop = statusData.optBoolean("xbeeStop", xbeeStop);
      drivetrainEnabled = statusData.optBoolean("drivetrainEnabled", drivetrainEnabled);
      headlightEnabled = statusData.optBoolean("headlightEnabled", headlightEnabled);
      adjustMySpeed = statusData.optBoolean("adjustMySpeed", adjustMySpeed);
      gpsLock = statusData.optBoolean("gpsLock", gpsLock);
      loopNumber = statusData.optInt("loopNumber", loopNumber);
    JSONObject sensorsData = jsonData.optJSONObject("sensors");
      rangefinderValue = sensorsData.optDouble("rangefinder", rangefinderValue);
      leftBumper = sensorsData.optBoolean("leftBumper", leftBumper);
      rightBumper = sensorsData.optBoolean("rightBumper", rightBumper);
      driveBatteryVoltage = sensorsData.optDouble("driveBatteryVoltage", driveBatteryVoltage);
    JSONObject locationData = jsonData.optJSONObject("location");
      JSONObject currentLocationData = locationData.optJSONObject("current");
        currentLat = currentLocationData.optDouble("lat",currentLat);
        currentLon = currentLocationData.optDouble("lon",currentLon);
      JSONObject goalLocationData = locationData.optJSONObject("goal");
        goalLat = goalLocationData.optDouble("lat",goalLat);
        goalLon = goalLocationData.optDouble("lon",goalLon);
        distanceToGoal = goalLocationData.optDouble("distanceToGoal", distanceToGoal);
      JSONObject headingLocationData = locationData.optJSONObject("heading");
        currentHeading = headingLocationData.optDouble("current", currentHeading);
        goalHeading = headingLocationData.optDouble("goal", goalHeading);
        turnAngle = headingLocationData.optDouble("turnAngle", turnAngle);
        steeringAngle = headingLocationData.optInt("steeringAngle", steeringAngle);
      JSONObject speedLocationData = locationData.optJSONObject("speed");
        currentSpeed = speedLocationData.optDouble("current", currentSpeed);
        goalSpeed = speedLocationData.optDouble("goal", goalSpeed);
        maxSpeed = speedLocationData.optDouble("max", maxSpeed);
        adjustedSpeedValue = speedLocationData.optInt("adjustedSpeedValue", adjustedSpeedValue);
    JSONObject imuData = jsonData.optJSONObject("imu");
      imuYaw = imuData.optDouble("yaw", imuYaw);
      imuPitch = imuData.optDouble("pitch", imuPitch);
      imuRoll = imuData.optDouble("roll", imuRoll);
    debugInfo = jsonData.optString("debug","Failed to read data.");
    jsonError = false;
  }
  catch(Exception e){
    System.out.println("Encountered error. Will not process.");
    jsonError = true;
  }
  if(!jsonError){
    // Status Buttons
    guiDrivetrain.setValue(!xbeeStop);
    guiGPSLock.setValue(gpsLock);
    // Sensor Buttons
    guiLeftBumper.setValue(leftBumper);
    guiRightBumper.setValue(rightBumper);
    guiHeadlight.setValue(headlightEnabled);
    guiSteering.setValue(steeringEnabled);
    guiRangefinder.setValue((float) rangefinderValue);
    guiDriveBattery.setValue((float) driveBatteryVoltage);
    // Current Position
    currentLocation = new LatLonPt(currentLat, currentLon, defaultPointSize);
    currentXY = new XYPt(currentLocation);
    guiCurrentLat.setValue((float) currentLocation.getLat());
    guiCurrentLon.setValue((float) currentLocation.getLon());
    // Goal Position
    goalLocation = new LatLonPt(goalLat, goalLon, defaultPointSize);
    goalXY = new XYPt(goalLocation);
    guiGoalLat.setValue((float) goalLocation.getLat());
    guiGoalLon.setValue((float) goalLocation.getLon());
    guiDistanceToGoal.setValue((float) distanceToGoal);
    // Headings
    guiCurrentHeading.setStartAngle(radians((float)currentHeading-90));
    guiGoalHeading.setStartAngle(radians((float)goalHeading-90));
    guiServo.setValue(steeringAngle);
    
    // Speed
    guiCurrentSpeed.setValue((float) currentSpeed);
    guiGoalSpeed.setValue((float) goalSpeed);
    guiMaxSpeed.setValue((float) maxSpeed);
    guiESC.setValue(adjustedSpeedValue);
    // Debug Info
    guiLoopNumber.setValue(loopNumber);
  }
}

void cameraUpdate(){
  viewXY = currentXY.lerp(goalXY, 0.5);
  cameraFollowMouse = guiFollowMouse.getState();
  if(cameraFollowMouse){
    viewXY = viewXY.lerp(mouseXY, 0.5);
  }
  double cameraMoveDistance = cameraXY.distanceTo(viewXY);
  double cameraViewDistance;
  if(cameraFollowMouse){
    cameraViewDistance = currentXY.distanceTo(goalXY)+currentXY.distanceTo(mouseXY);
  }else{
    cameraViewDistance = currentXY.distanceTo(goalXY);
  }
  double cameraMoveAngle = cameraXY.angleTo(viewXY);
  double cameraDelta = Math.min(Math.sqrt(cameraMoveDistance)/4.0,cameraMoveAmount);
  double cameraDeltaX = Math.cos(cameraMoveAngle)*cameraDelta;
  double cameraDeltaY = Math.sin(cameraMoveAngle)*cameraDelta;
  cameraXY.add(cameraDeltaX, cameraDeltaY);
  cameraHeight = lerp(cameraHeight, Math.max((float) cameraViewDistance, (windowHeight/2.0) / tan(PI/6)), 0.02);
}

void serialEvent(Serial p){
  inString = (myPort.readString());
  String[] stringList = split(inString, '~');
  theData = trim(stringList[0]);
  jsonData = new JSONObject(theData);
}

void drawPoint(color myColor, XYPt xy){
  pushStyle();
    ellipseMode(CENTER);
    fill(myColor);
    float radius = max((float) (2*xy.getRadius()), (float) cameraHeight/80);
    ellipse( (float) xy.getX(), (float) xy.getY(), radius, radius);
  popStyle();
}

void keyPressed(){
  if(key != CODED){
    myPort.write(key);
  }
}

boolean sketchFullScreen() {
  return fullScreen;
}

void testPointMove(){
  testTheta += 0.0020;
  currentXY = new XYPt(1000+400*cos(testTheta+PI), 1000-730*sin(testTheta+PI));
  currentLocation = new LatLonPt(currentXY);
  guiCurrentLat.setValue((float) currentLocation.getLat());
  guiCurrentLon.setValue((float) currentLocation.getLon());
  float goalTheta = testTheta;
  goalTheta %= (2*PI);
  goalTheta /= (2*PI);
  goalTheta *= 12;
  goalTheta = ceil(goalTheta);
  goalTheta /= 12;
  goalTheta *= (2*PI);
  goalXY = new XYPt(1000+400*cos(goalTheta+PI), 1000-730*sin(goalTheta+PI));
  if(goalXY != null){
    goalLocation = new LatLonPt(goalXY);
    guiGoalLat.setValue((float) goalLocation.getLat());
    guiGoalLon.setValue((float) goalLocation.getLon());
  }
  debugInfo = "Tracking test data ellipse.";
}

void updateMouseXY(){
  mouseSceneX = (float) (cameraXY.getX() + (0.57735 * cameraHeight * width/height * (mouseX - 0.5*width) / (0.5*width) ) );
  mouseSceneY = (float) (cameraXY.getY() + (0.57735 * cameraHeight * (mouseY - 0.5*height) / (0.5*height) ) );
  mouseXY = new XYPt(mouseSceneX, mouseSceneY);
  mouseLocation = new LatLonPt(mouseXY);
  guiMouseLat.setValue((float) mouseLocation.getLat());
  guiMouseLon.setValue((float) mouseLocation.getLon());
}

void guiSetup(){
  // GUI setup
  cp5 = new ControlP5(this);
  
  guiCurrentLat = cp5.addNumberbox("Car Latitude")
    .setPosition(5, 5)
    .setSize(70,20)
    ;
  guiLatLon(guiCurrentLat);
    
  guiCurrentLon = cp5.addNumberbox("Car Longitude")
    .setPosition(75, 5)
    .setSize(70,20)
    ;
  guiLatLon(guiCurrentLon);
    
  guiGoalLat = cp5.addNumberbox("Goal Latitude")
    .setPosition(5, 41)
    .setSize(70,20)
    ;
  guiLatLon(guiGoalLat);
    
  guiGoalLon = cp5.addNumberbox("Goal Longitude")
    .setPosition(75, 41)
    .setSize(70,20)
    ;
  guiLatLon(guiGoalLon);
    
  guiMouseLat = cp5.addNumberbox("Mouse Latitude")
    .setPosition(5, 77)
    .setSize(70,20)
    ;
  guiLatLon(guiMouseLat);
    
  guiMouseLon = cp5.addNumberbox("Mouse Longitude")
    .setPosition(75, 77)
    .setSize(70,20)
    ;
  guiLatLon(guiMouseLon);
    
  guiDrivetrain = cp5.addToggle("Drivetrain")
    .setPosition(5, 115)
    .setSize(40,40)
    ;
  
  guiSteering = cp5.addToggle("Steering")
    .setPosition(55, 115)
    .setSize(40,40)
    ;
  
  guiHeadlight = cp5.addToggle("Headlight")
    .setPosition(105, 115)
    .setSize(40,40)
    ;
      
  guiGPSLock = cp5.addToggle("GPS Lock")
    .setPosition(5, 171)
    .setSize(40,40)
    .setValue(false)
    ;
  deactivateToggle(guiGPSLock);
  
  guiLeftBumper = cp5.addToggle("L Bumper")
    .setPosition(55, 171)
    .setSize(40,40)
    .setValue(false)
    ;
  deactivateToggle(guiLeftBumper);
  
  guiRightBumper = cp5.addToggle("R Bumper")
    .setPosition(105, 171)
    .setSize(40,40)
    .setValue(false)
    ;
  deactivateToggle(guiRightBumper);
    
  guiManualAutomatic = cp5.addToggle("Manual/Automatic")
    .setPosition(5, 228)
    .setSize(140,40)
    .setValue(false)
    .setMode(ControlP5.SWITCH)
    ;
      
  guiFollowMouse = cp5.addToggle("Follow Mouse")
    .setPosition(5, 285)
    .setSize(140,20)
    ;
      
  
  guiGoalSpeed = cp5.addSlider("Goal Speed")
    .setPosition(10,350)
    .setSize(10,200)
    .setRange(0,maxGoalSpeed)
    .setValue((float) goalSpeed)
    .setSliderMode(Slider.FLEXIBLE)
    ;
    
  guiCopyPosition = cp5.addButton("copyPosition")
    .setPosition(40, 350)
    .setValue(0)
    .setCaptionLabel("Copy Position")
    .setSize(105,20)
    ;
    

  guiCurrentHeading = cp5.addKnob("Heading")
    .setRange(0,1)
    .setValue(0.02)
    .setPosition(45,410)
    .setRadius(50)
    .setLock(true)
    .setTickMarkWeight(0)
    .setStartAngle(0)
    ;
    
  guiGoalHeading = cp5.addKnob("HeadingGoal")
    .setRange(0,1)
    .setValue(0.02)
    .setPosition(66,431)
    .setRadius(29)
    .setLock(true)
    .setTickMarkWeight(0)
    .setLabelVisible(false)
    .setColorBackground(color(1, 108, 158))
    .setColorForeground(color(2, 52, 77))
    .setStartAngle(0)
    ;

  guiWaypointBehavior = cp5.addDropdownList("Waypoint Behavior")
    .setPosition(5,340)
    .setSize(140,20)
    .setItemHeight(15)
    .setBarHeight(15)
    .setHeight(100)
    ;
  guiWaypointBehavior.captionLabel().set("Waypoint Behavior");
  guiWaypointBehavior.captionLabel().style().marginTop = 3;
  guiWaypointBehavior.captionLabel().style().marginLeft = 3;
  guiWaypointBehavior.valueLabel().style().marginTop = 3;
  guiWaypointBehavior.addItem("Stop", 0);
  guiWaypointBehavior.addItem("Loop", 1);
  guiWaypointBehavior.addItem("Reverse at End", 2);
  
  guiCurrentSpeed = cp5.addNumberbox("Current Speed")
    .setPosition(5, 570)
    .setSize(70,20)
    ;
    guiLatLon(guiCurrentSpeed);
    
   guiMaxSpeed = cp5.addNumberbox("Max Speed")
    .setPosition(75,570)
    .setSize(70,20)
    ;
    guiLatLon(guiMaxSpeed);
      
    guiDistanceToGoal = cp5.addNumberbox("Distance to Goal")
    .setPosition(5, 610)
    .setSize(140,20)
    ;
    guiLatLon(guiDistanceToGoal);
  
  guiDebugVisible = cp5.addToggle("Debug Visible")
    .setPosition(5, 650)
    .setSize(140,20)
    .setValue(false)
    ;
  
  guiESC = cp5.addNumberbox("ESC")
    .setPosition(180,5)
    .setSize(75,20)
    .setDecimalPrecision(0)
    .setColorBackground(color(35))
    .setColorForeground(color(160))
    .setLock(true)
    .setVisible(debugVisible)
    ;
    
  guiServo = cp5.addNumberbox("Servo")
    .setPosition(260, 5)
    .setSize(75,20)
    .setDecimalPrecision(0)
    .setColorBackground(color(35))
    .setColorForeground(color(160))
    .setLock(true)
    .setVisible(debugVisible)
    ;
    
  guiRangefinder = cp5.addNumberbox("Rangefinder")
    .setPosition(180, 41)
    .setSize(75,20)
    .setDecimalPrecision(2)
    .setColorBackground(color(35))
    .setColorForeground(color(160))
    .setLock(true)
    .setVisible(debugVisible)
    ;
    
  guiDriveBattery = cp5.addNumberbox("Drive Battery")
    .setPosition(260, 41)
    .setSize(75,20)
    .setDecimalPrecision(3)
    .setColorBackground(color(35))
    .setColorForeground(color(160))
    .setLock(true)
    .setVisible(debugVisible)
    ;
  
  guiLoopNumber = cp5.addNumberbox("Loop Number")
    .setPosition(180, 77)
    .setSize(75,20)
    .setDecimalPrecision(3)
    .setColorBackground(color(35))
    .setColorForeground(color(160))
    .setLock(true)
    .setVisible(debugVisible)
    ;
  
/*  guiImuHistoryStdev = cp5.addNumberbox("imuStdev")
    .setPosition(260, 77)
    .setSize(75,20)   
    .setDecimalPrecision(3)
    .setColorBackground(color(35))
    .setColorForeground(color(160))
    .setLock(true)
    .setVisible(debugVisible)
    ;
*/
/*  guiGpsHeadingHistoryStdev = cp5.addNumberbox("GpsHeadingHistoryStdev")
    .setPosition(260, 113)
    .setSize(75,20)
    .setDecimalPrecision(3)
    .setColorBackground(color(35))
    .setColorForeground(color(160))
    .setLock(true)
    .setVisible(debugVisible)
    ;
*/
}

void guiUpdate(){
  debugVisible = guiDebugVisible.getState();
  guiESC.setVisible(debugVisible);
  guiServo.setVisible(debugVisible);
  guiRangefinder.setVisible(debugVisible);
  guiDriveBattery.setVisible(debugVisible);
  guiLoopNumber.setVisible(debugVisible);
  //guiImuHistoryStdev.setVisible(debugVisible);
  //guiGpsHeadingHistoryStdev.setVisible(debugVisible);
}

void guiLatLon(Numberbox theNumberbox){
  theNumberbox
    .setDecimalPrecision(5)
    .setValue(0)
    .setColorBackground(color(35))
    .setColorForeground(color(160))
    .setLock(true)
    ;
}

void deactivateToggle(Toggle theToggle){
  theToggle
    .setColorBackground(color(35))
    .setColorActive(color(200))
    .setLock(true)
    ;
}

public void copyPosition(int buttonValue){
  clippy.copyString(currentLocation.toString() + ", 3.0,\r\n");
}
