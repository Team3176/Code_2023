// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.signalling;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import team3176.robot.constants.RobotConstants.Status;
import team3176.robot.constants.SignalingConstants;


public class Signalling extends SubsystemBase {
  
  private static Signalling instance = new Signalling();
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private int m_flashcounter = 0;
  private boolean leftflash = false;
  private boolean rightflash = false;
  private boolean crossflash = false;
  private Color leftflcolor = Color.kRed;
  private Color rightflcolor = Color.kOrange;
  private Color crossflcolor = Color.kGreen;

  
  /**
   * Creates the default references for VisionClient, specifically for Limelight values
   */
  
   
   public Signalling(){
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  public static Signalling getInstance(){
    if (instance == null) {
      instance = new Signalling();
    }
    return instance;
  }


    public void setSegment(int start, int end, int red, int green, int blue)
    {
      for (var i=start; i < end; i++)
      {
        m_ledBuffer.setRGB(i, red , green, blue);
      }
    }

    public void setSegment(int start, int end, Color color)
    {
      for (var i=start; i < end; i++)
      {
        m_ledBuffer.setLED(i, color);
      }
      // m_led.setData(m_ledBuffer);
    }

    public void setleft(Status s)
    {
      leftflcolor = LookUpColor(s);
      leftflash = LookUpFlash(s);
      setSegment(SignalingConstants.LEFTENDSTART, SignalingConstants.LEFTENDSTOP, leftflcolor);
      m_led.setData(m_ledBuffer);
    }
    public void setright(Status s)
    {
      rightflcolor = LookUpColor(s);
      rightflash = LookUpFlash(s);
      setSegment(SignalingConstants.RIGHTENDSTART, SignalingConstants.RIGHTENDSTOP, rightflcolor);
      m_led.setData(m_ledBuffer);
    }
    public void setcrossbar(Status s)
    {
      crossflcolor = LookUpColor(s);
      crossflash = LookUpFlash(s);
      setSegment(SignalingConstants.CROSSENDSTART, SignalingConstants.CROSSENDSTOP, crossflcolor);
      m_led.setData(m_ledBuffer);
    }
  private Color LookUpColor(Status s){
    Color c = Color.kBlack;
    switch(s){
      case STABLE: c = Color.kGreen;
      break;
      case OK: c = Color.kDarkRed;
      break;
      case OPTIONALCHECK: c = Color.kLightYellow;
      break;
      case WARNING: c = Color.kFuchsia;
      break;
      case GOOD: c = Color.kLightCoral;
      break;
      case ERROR: c = Color.kLime;
      break;
      case CONE: c = Color.kOrange;
      break;
      case CUBE: c = Color.kPurple;
      break;
      case CONEFLASH: c = Color.kOrange;
      break;
      case CUBEFLASH: c = Color.kPurple;
      break;
      case NONE: c = Color.kBlack;
      break;
    }
    return c;
  }
  private Boolean LookUpFlash(Status s){
      return ((s == Status.CUBEFLASH) || (s == Status.CONEFLASH));
  }

  @Override
  public void periodic() {      
    m_flashcounter++;
    if (m_flashcounter == 25){
     if (leftflash){
      setSegment(SignalingConstants.LEFTENDSTART, SignalingConstants.LEFTENDSTOP,leftflcolor);
     }
     if (rightflash){
      setSegment(SignalingConstants.RIGHTENDSTART, SignalingConstants.RIGHTENDSTOP, rightflcolor);
     }
     if (crossflash){
      setSegment(SignalingConstants.CROSSENDSTART, SignalingConstants.CROSSENDSTOP, crossflcolor);
     }
     m_led.setData(m_ledBuffer);
     }
    if (m_flashcounter == 50){
      if (leftflash){
        setSegment(SignalingConstants.LEFTENDSTART, SignalingConstants.LEFTENDSTOP, Color.kBlack);
       }
       if (rightflash){
        setSegment(SignalingConstants.RIGHTENDSTART, SignalingConstants.RIGHTENDSTOP, Color.kBlack);
       }
       if (crossflash){
        setSegment(SignalingConstants.CROSSENDSTART, SignalingConstants.CROSSENDSTOP, Color.kBlack);
       }
       m_led.setData(m_ledBuffer);
      m_flashcounter = 0;
     }
    }
   }


