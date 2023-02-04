// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.signalling;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import team3176.robot.constants.RobotConstants.Status;

public class Signalling extends SubsystemBase {
  
  private static Signalling instance = new Signalling();
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private int m_rainbowFirstPixelHue;

  
  /**
   * Creates the default references for VisionClient, specifically for Limelight values
   */
  public Signalling(){
    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    
    m_led.setData(m_ledBuffer);
  }
public void setState(Status ledState){

  switch (ledState) {
    case STABLE:
      green();
      break;
    case OK:
     yellow();
     break;
  case OPTIONALCHECK:
     orange();
      break;
  case WARNING:
     red();
      break;
  case ISCONE:
    yellow();
    break;
  case ISCUBE:
    blue();
    break;
  case REQUESTCONE:
    yellow();
    break;
  case REQUESTCUBE:
    blue();
    break;
  case OFF:
    turnOff();
    break;

    default:


}
  
 
public void red() {
  red(STARTINGLED, ENDINGLED);
 }

 public void red(int startingLed, int endingLed) {
  for (var i = startingLed; i < endingLed; i++) {
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(i, 225, 0, 0);
  }

  m_led.setData(m_ledBuffer);
}
public void green() {
  green(STARTINGLED, ENDINGLED);
 }
public void green(int startingLed, int endingLed) {
  for (var i = startingLed; i < endingLed; i++) {
    // Sets the specified LED to the RGB values for green
    m_ledBuffer.setRGB(i, 0, 225, 0);
  }
 
  m_led.setData(m_ledBuffer);
}
public void yellow() {
  yellow(STARTINGLED, ENDINGLED);
  
  public void yellow(int startingLed, int endingLed) {
  for (var i = startingLed; i < endingLed; i++) {
    // Sets the specified LED to the RGB values for yellow
    m_ledBuffer.setRGB(i, 225, 225, 0);
  }
  
  m_led.setData(m_ledBuffer);
}
public void orange() {
  orange(STARTINGLED, ENDINGLED);
  
  public void orange(int startingLed, int endingLed) {
  for (var i = startingLed; i < endingLed; i++) {
    // Sets the specified LED to the RGB values for orange
    m_ledBuffer.setRGB(i, 255, 117, 25);
  }  
  
  m_led.setData(m_ledBuffer);
}
public void orange(int startingLed, int endingLed) {
  for (var i = startingLed; i < endingLed; i++) {
    // Sets the specified LED to the RGB values for orange
    m_ledBuffer.setRGB(i, 255, 117, 25);
  }  
  
  m_led.setData(m_ledBuffer);
}
public void gold() {
  gold(STARTINGLED, ENDINGLED);
  
  public void gold(int startingLed, int endingLed) {
  for (var i = startingLed; i < endingLed; i++) {
    // Sets the specified LED to the RGB values for gold
    m_ledBuffer.setRGB(i, 204, 204, 0);
  }  
  
  m_led.setData(m_ledBuffer);
}
public void purple() {
  purple(STARTINGLED, ENDINGLED);
  
  public void purple(int startingLed, int endingLed) {
  for (var i = startingLed; i < endingLed; i++) {
    // Sets the specified LED to the RGB values for purple
    m_ledBuffer.setRGB(i, 102, 0, 204);
  }  
  
  m_led.setData(m_ledBuffer);
}
public void blue() {
  blue(STARTINGLED, ENDINGLED);
  
  public void blue(int startingLed, int endingLed) {
  for (var i = startingLed; i < endingLed; i++) {
    // Sets the specified LED to the RGB values for blue
    m_ledBuffer.setRGB(i, 0, 0, 255);
  }  
  
  m_led.setData(m_ledBuffer);
}
public void turnOff() {
  turnOff(STARTINGLED, ENDINGLED);

public void turnOff(int startingLed, int endingLed) {
  for (var i = startingLed; i < endingLed; i++) {
    // Turn off all the LEDS
    m_ledBuffer.setRGB(i, 0, 0, 0);
  }

  m_led.setData(m_ledBuffer);
}

public static Signalling getInstance(){
    return instance;
  }
  
}
