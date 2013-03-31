import procontroll.*;
import java.io.*;

ControllIO controll;
ControllDevice device;
ControllButton buttonA;
ControllButton buttonB;
ControllButton buttonX;
ControllButton buttonY;
ControllButton buttonLB;
ControllButton buttonRB;
ControllButton buttonBack;
ControllButton buttonStart;
ControllButton buttonLeftStick;
ControllButton buttonRightStick;
ControllCoolieHat cooliehat;
ControllSlider leftStickX;
ControllSlider leftStickY;
ControllSlider rightStickX;
ControllSlider rightStickY;
ControllSlider topSliders;



void setup(){
  size(400,400);
  
  controll = ControllIO.getInstance(this);
  device = controll.getDevice("XUSB Gamepad (Controller)");
  device.setTolerance(0.05f);
  
  buttonA = device.getButton(0);
  buttonB = device.getButton(1);
  buttonX = device.getButton(2);
  buttonY = device.getButton(3);
  buttonLB = device.getButton(4);
  buttonRB= device.getButton(5);
  buttonStart = device.getButton(6);
  buttonBack = device.getButton(7);
  buttonLeftStick = device.getButton(8);
  buttonRightStick = device.getButton(9);
  
  cooliehat = device.getCoolieHat(10);
  cooliehat.setMultiplier(4);
  
  leftStickY = device.getSlider(0);
  leftStickX = device.getSlider(1);
  rightStickY = device.getSlider(2);
  rightStickX = device.getSlider(3);
  topSliders = device.getSlider(4);
  
  
}

  
