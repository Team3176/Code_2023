// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.signalling;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import team3176.robot.constants.RobotConstants.Status;
import team3176.robot.subsystems.RobotState.e_CurrentGameElementImWanting;
import team3176.robot.subsystems.RobotState.e_CurrentGameElementImHolding;
import team3176.robot.subsystems.RobotState.e_ArmElbowPositionState;
import team3176.robot.subsystems.RobotState.e_IntakeDetectsImHolding;
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
      leftflcolor = LookUpStatusColor(s);
      setSegment(SignalingConstants.LEFTENDSTART, SignalingConstants.LEFTENDSTOP, leftflcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setleft(e_CurrentGameElementImWanting w)
    {
      leftflcolor = LookUpGPColor(w);
      leftflash = LookUpFlash(w);
      setSegment(SignalingConstants.LEFTENDSTART, SignalingConstants.LEFTENDSTOP, leftflcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setleft(e_CurrentGameElementImHolding h)
    {
      leftflcolor = LookUpGPColor(h);
      setSegment(SignalingConstants.LEFTENDSTART, SignalingConstants.LEFTENDSTOP, leftflcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setIntakeGPColor(e_IntakeDetectsImHolding h)
    {
      leftflcolor = LookUpIntakeGPColor(h);
      setSegment(SignalingConstants.LEFTENDSTART, 5, leftflcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setArmElbowPosColor(e_ArmElbowPositionState a)
    {
      leftflcolor = LookUpElbowPosColor(a);
      if (a == e_ArmElbowPositionState.IsPICKUP)
      {
        setSegment(6, 8, leftflcolor);
      }
      else if (a == e_ArmElbowPositionState.IsFLOORCONE || a == e_ArmElbowPositionState.IsFLOORCUBE)
      {
        setSegment(7, 9, leftflcolor);
      }
      else if (a == e_ArmElbowPositionState.IsMIDCONE || a == e_ArmElbowPositionState.IsMIDCUBE)
      {
        setSegment(8, 10, leftflcolor);
      }
      else if (a == e_ArmElbowPositionState.IsHIGHCONE || a == e_ArmElbowPositionState.IsHIGHCUBE)
      {
        setSegment(9, 11, leftflcolor);
      }
    }

    public void setright(Status s)
    {
      rightflcolor = LookUpStatusColor(s);
      setSegment(SignalingConstants.RIGHTENDSTART, SignalingConstants.RIGHTENDSTOP, rightflcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setright(e_CurrentGameElementImWanting w)
    {
      rightflcolor = LookUpGPColor(w);
      rightflash = LookUpFlash(w);
      setSegment(SignalingConstants.RIGHTENDSTART, SignalingConstants.RIGHTENDSTOP, rightflcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setcrossbar(Status s)
    {
      crossflcolor = LookUpStatusColor(s);
      setSegment(SignalingConstants.CROSSENDSTART, SignalingConstants.CROSSENDSTOP, crossflcolor);
      m_led.setData(m_ledBuffer);
    }

    public void setcrossbar(e_CurrentGameElementImWanting w)
    {
      crossflcolor = LookUpGPColor(w);
      crossflash = LookUpFlash(w);
      setSegment(SignalingConstants.CROSSENDSTART, SignalingConstants.CROSSENDSTOP, crossflcolor);
      m_led.setData(m_ledBuffer);
    }

  private Color LookUpStatusColor(Status s){
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
      case NONE: c = Color.kBlack;
      break;
    }
    return c;
  }

  private Color LookUpElbowPosColor(e_ArmElbowPositionState a)
  {
    Color c = Color.kLightBlue;
    switch(a){
      case IsPICKUP: c = Color.kLightBlue; 
      break;
      case IsHIGHCONE: c = Color.kYellow;
      break;
      case IsHIGHCUBE: c = Color.kPink;
      break;
      case IsMIDCONE: c = Color.kOrange;
      break;
      case IsMIDCUBE: c = Color.kMagenta;
      break;
      case IsFLOORCONE: c = Color.kDarkOrange;
      break;
      case IsFLOORCUBE: c = Color.kPurple;
      break;
    }
    return c;
  }

  private Color LookUpGPColor(e_CurrentGameElementImWanting w)
  {
    Color c = Color.kBlack;
    switch(w){
      case CONE: c = Color.kOrange;
      break;
      case CUBE: c = Color.kPurple;
      break;
      case NONE: c = Color.kBlack;
      break;
    }
    return c;
  }

  private Color LookUpGPColor(e_CurrentGameElementImHolding h)
  {
    Color c = Color.kBlack;
    switch(h){
      case CONE: c = Color.kOrange;
      break;
      case CUBE: c = Color.kPurple;
      break;
      case NONE: c = Color.kBlack;
      break;
    }
    return c;
  }

  private Color LookUpIntakeGPColor(e_IntakeDetectsImHolding h)
  {
    Color c = Color.kBlack;
    switch(h){
      case CONE: c = Color.kOrange;
      break;
      case CUBE: c = Color.kPurple;
      break;
      case NOTHING: c = Color.kBlack;
      break;
      case ERROR: c = Color.kRed;
      break;
    }
    return c;
  }

  private Boolean LookUpFlash(e_CurrentGameElementImWanting w){
    return ((w == e_CurrentGameElementImWanting.CUBE) || (w == e_CurrentGameElementImWanting.CONE));
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


