// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3176.robot.constants.ControllerConstants;
// import team3176.robot.util.XboxController.XboxAxisAsButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
// import team3176.robot.util.XboxController.*;

public class Controller {
  private static Controller instance = new Controller();

  public static Controller getInstance() {
    if (instance == null ) {
      instance = new Controller();
    }
      return instance;}

  /* The Three Physical Controllers that we have */

  private final Joystick transStick;
  private final Joystick rotStick;
  public CommandXboxController operator;

  /* First Part of Creating the Buttons on the Joysticks */

  private final JoystickButton transStick_Button1;
  private final JoystickButton transStick_Button2;
  private final JoystickButton transStick_Button3;
  private final JoystickButton transStick_Button4;
  private final JoystickButton transStick_Button5;
  private final JoystickButton transStick_Button6;
  private final JoystickButton transStick_Button7;
  private final JoystickButton transStick_Button8;
  private final JoystickButton transStick_Button9;
  private final JoystickButton transStick_Button10;
  private final JoystickButton transStick_Button11;
  private final JoystickButton transStick_Button12;
  private final JoystickButton transStick_Button13;
  private final JoystickButton transStick_Button14;
  private final JoystickButton transStick_Button15;
  private final JoystickButton transStick_Button16;
  private final POVButton transStick_HAT_0;
  private final POVButton transStick_HAT_45;
  private final POVButton transStick_HAT_90;
  private final POVButton transStick_HAT_135;
  private final POVButton transStick_HAT_180;
  private final POVButton transStick_HAT_225;
  private final POVButton transStick_HAT_270;
  private final POVButton transStick_HAT_315;


  private final JoystickButton rotStick_Button1;
  private final JoystickButton rotStick_Button2;
  private final JoystickButton rotStick_Button3;
  private final JoystickButton rotStick_Button4;
  private final JoystickButton rotStick_Button5;
  private final JoystickButton rotStick_Button6;
  private final JoystickButton rotStick_Button7;
  private final JoystickButton rotStick_Button8;
  private final JoystickButton rotStick_Button9;
  private final JoystickButton rotStick_Button10;
  private final JoystickButton rotStick_Button11;
  private final JoystickButton rotStick_Button12;
  private final JoystickButton rotStick_Button13;
  private final JoystickButton rotStick_Button14;
  private final JoystickButton rotStick_Button15;
  private final JoystickButton rotStick_Button16;
  private final POVButton rotStick_HAT_0;
  private final POVButton rotStick_HAT_45;
  private final POVButton rotStick_HAT_90;
  private final POVButton rotStick_HAT_135;
  private final POVButton rotStick_HAT_180;
  private final POVButton rotStick_HAT_225;
  private final POVButton rotStick_HAT_270;
  private final POVButton rotStick_HAT_315;

  //TODO: Add slider

  // private final Trigger op_A;
  // private final Trigger op_A_Shift;
  // private final Trigger op_A_Double_Shift;
  // private final Trigger op_B;
  // private final Trigger op_B_Shift;
  // private final Trigger op_B_Double_Shift;
  // private final Trigger op_X;
  // private final Trigger op_X_Shift;
  // private final Trigger op_X_Double_Shift;
  // private final Trigger op_Y;
  // private final Trigger op_Y_Shift;
  // private final Trigger op_Y_Double_Shift;
  // private final Trigger op_Start;
  // private final Trigger op_Start_Shift;
  // private final Trigger op_Start_Double_Shift;
  // private final Trigger op_Back;
  // private final Trigger op_Back_Shift;
  // private final Trigger op_Back_Double_Shift;
  // private final Trigger op_LTrigger;
  // private final Trigger op_RTrigger;
  // private final POVButton op_DPAD_Up;
  // private final POVButton op_DPAD_Left;
  // private final POVButton op_DPAD_Down;
  // private final POVButton op_DPAD_Right;

  public Controller() {
    /* Finish Creating the Objects */

    transStick = new Joystick(ControllerConstants.TRANS_ID);
    rotStick = new Joystick(ControllerConstants.ROT_ID);
    operator = new CommandXboxController(ControllerConstants.OP_ID);

    transStick_Button1 = new JoystickButton(transStick, 1);
    transStick_Button2 = new JoystickButton(transStick, 2);
    transStick_Button3 = new JoystickButton(transStick, 3);
    transStick_Button4 = new JoystickButton(transStick, 4);
    transStick_Button5 = new JoystickButton(transStick, 5);
    transStick_Button6 = new JoystickButton(transStick, 6);
    transStick_Button7 = new JoystickButton(transStick, 7);
    transStick_Button8 = new JoystickButton(transStick, 8);
    transStick_Button9 = new JoystickButton(transStick, 9);
    transStick_Button10 = new JoystickButton(transStick, 10);
    transStick_Button11 = new JoystickButton(transStick, 11);
    transStick_Button12 = new JoystickButton(transStick, 12);
    transStick_Button13 = new JoystickButton(transStick, 13);
    transStick_Button14 = new JoystickButton(transStick, 14);
    transStick_Button15 = new JoystickButton(transStick, 15);
    transStick_Button16 = new JoystickButton(transStick, 16);
    
    /** 
     * The HAT on the transStick
     * The values are 0 as UP going in a 360 circle CCW
     */
    transStick_HAT_0 = new POVButton(transStick, 0);
    transStick_HAT_45 = new POVButton(transStick, 45);
    transStick_HAT_90 = new POVButton(transStick, 90);
    transStick_HAT_135 = new POVButton(transStick, 135);
    transStick_HAT_180 = new POVButton(transStick, 180);
    transStick_HAT_225 = new POVButton(transStick, 225);
    transStick_HAT_270 = new POVButton(transStick, 270);
    transStick_HAT_315 = new POVButton(transStick, 315);


    
    rotStick_Button1 = new JoystickButton(rotStick, 1);
    rotStick_Button2 = new JoystickButton(rotStick, 2);
    rotStick_Button3 = new JoystickButton(rotStick, 3);
    rotStick_Button4 = new JoystickButton(rotStick, 4);
    rotStick_Button5 = new JoystickButton(rotStick, 5);
    rotStick_Button6 = new JoystickButton(rotStick, 6);
    rotStick_Button7 = new JoystickButton(rotStick, 7);
    rotStick_Button8 = new JoystickButton(rotStick, 8);
    rotStick_Button9 = new JoystickButton(rotStick, 9);
    rotStick_Button10 = new JoystickButton(rotStick, 10);
    rotStick_Button11 = new JoystickButton(rotStick, 11);
    rotStick_Button12 = new JoystickButton(rotStick, 12);
    rotStick_Button13 = new JoystickButton(rotStick, 13);
    rotStick_Button14 = new JoystickButton(rotStick, 14);
    rotStick_Button15 = new JoystickButton(rotStick, 15);
    rotStick_Button16 = new JoystickButton(rotStick, 16);
    
    /** 
     * The HAT on the rotStick
     * The values are 0 as UP going in a 360 circle CCW
     */
    rotStick_HAT_0 = new POVButton(rotStick, 0);
    rotStick_HAT_45 = new POVButton(rotStick, 45);
    rotStick_HAT_90 = new POVButton(rotStick, 90);
    rotStick_HAT_135 = new POVButton(rotStick, 135);
    rotStick_HAT_180 = new POVButton(rotStick, 180);
    rotStick_HAT_225 = new POVButton(rotStick, 225);
    rotStick_HAT_270 = new POVButton(rotStick, 270);
    rotStick_HAT_315= new POVButton(rotStick, 315);

    /* 
     * The Xbox Controller Buttons 
     * XboxMain: The first level of control; NAME + NOT FIRST SHIFT + NOT SECOND SHIFT
     * XboxShift: The second level of control; The first shift; NAME + First SHIFT + NOT SECOND SHIFT
     * XboxDBLShift: The third level of control; The second shift; NAME + FIRST SHIFT + SECOND SHIFT
     */

    // op_A = new XboxMain(operator, Button.kA.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_A_Shift = new XboxShift(operator, Button.kA.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_A_Double_Shift = new XboxDBLShift(operator, Button.kA.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_B = new XboxMain(operator, Button.kB.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_B_Shift = new XboxShift(operator, Button.kB.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_B_Double_Shift = new XboxDBLShift(operator, Button.kB.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_X = new XboxMain(operator, Button.kX.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_X_Shift = new XboxShift(operator, Button.kX.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_X_Double_Shift = new XboxDBLShift(operator, Button.kX.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_Y = new XboxMain(operator, Button.kY.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_Y_Shift = new XboxShift(operator, Button.kY.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_Y_Double_Shift = new XboxDBLShift(operator, Button.kY.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_Start = new XboxMain(operator, Button.kStart.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_Start_Shift = new XboxShift(operator, Button.kStart.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_Start_Double_Shift = new XboxDBLShift(operator, Button.kStart.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_Back = new XboxMain(operator, Button.kBack.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_Back_Shift = new XboxShift(operator, Button.kBack.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_Back_Double_Shift = new XboxDBLShift(operator, Button.kBack.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    // op_LTrigger = new XboxAxisAsButton(operator, Axis.kLeftTrigger.value, ControllerConstants.TRIGGER_THRESHOLD); //TODO: CHANGE THRESHOLD
    // op_RTrigger = new XboxAxisAsButton(operator, Axis.kRightTrigger.value, ControllerConstants.TRIGGER_THRESHOLD);
   
    
    
    /**
     * The DPAD of the Xbox Controller
     * The values are 0 as UP going in a 360 circle CCW
     */

    // op_DPAD_Up = new POVButton(operator, 0);
    // op_DPAD_Right = new POVButton(operator, 90);
    // op_DPAD_Down = new POVButton(operator, 180);
    // op_DPAD_Left = new POVButton(operator, 270);
  }

  /**
   * Scale is the power of 1
   * Deadband of 0.06
   * @return The scales magnitude vector of the Y axis of TransStick if it breaks deadband
   */

  public double getForward() {
    if(Math.abs(transStick.getY()) < 0.06) return 0.0;
    return ControllerConstants.FORWARD_AXIS_INVERSION * Math.pow(transStick.getY(), 1);
  }

  /**
   * Scale is the power of 1
   * Deadband of 0.06
   * @return The scales magnitude vector of the X axis of TransStick if it breaks deadband
   */

  public double getStrafe() {
    if(Math.abs(transStick.getX()) < 0.06) return 0.0;
    return ControllerConstants.STRAFE_AXIS_INVERSION * Math.pow(transStick.getX(), 1);
  }

  /**
   * Scale is the power of 1
   * Deadband of 0.06
   * @return The scales magnitude vector of the X axis of RotStick if it breaks deadband
   */

  public double getSpin() {
    if(Math.abs(rotStick.getX()) < 0.06) return 0.0;
    return 0.2 * ControllerConstants.SPIN_AXIS_INVERSION * (Math.pow(rotStick.getX(), 1) / 7.0);
  }

  /**
   * Joystick response curve: Linear 
   * @return (f(x) = x) 
   */
  public static double joyResponseLinear(double joyInput) {
    return Math.pow(joyInput, 1);
  }

  /**
   * Joystick response curve: 8-2 Cubic 
   * @return (f(x) = 0.8x+0.2x^3)
   */
  public static double joyResponse82Cubic(double joyInput) {
    double a = 0.8;
    double b = 0.2;
    return ((a * Math.pow(joyInput,1)) + (b * Math.pow(joyInput,3)));
  } 
  
  /**
   * Joystick response curve: 6-4 Cubic 
   * @return (f(x) = 0.6x+0.4x^3)
   */
  public static double joyResponse64Cubic(double joyInput) {
    double a = 0.6;
    double b = 0.4;
    return ((a * Math.pow(joyInput,1)) + (b * Math.pow(joyInput,3)));
  } 
 
  /**
   * Joystick response curve: 4-6 Cubic 
   * @return (f(x) = 0.4x+0.6x^3)
   */
  public static double joyResponse46Cubic(double joyInput) {
    double a = 0.4;
    double b = 0.6;
    return ((a * Math.pow(joyInput,1)) + (b * Math.pow(joyInput,3)));
  } 
 
  /**
   * Joystick response curve: 2-8 Cubic 
   * @return (f(x) = 0.4x+0.6x^3)
   */
  public static double joyResponse28Cubic(double joyInput) {
    double a = 0.2;
    double b = 0.8;
    return ((a * Math.pow(joyInput,1)) + (b * Math.pow(joyInput,3)));
  } 
 
  /**
   * Joystick response curve: Full Cubic 
   * @return (f(x) = x^3)
   */
  public static double joyResponseFullCubic(double joyInput) {
    return (Math.pow(joyInput,3));
  }
  
  /**
   * Joystick response curve: 2parm25
   * @return (f(x) = a + (1 - a) * (b*x^3+b*x))
   * a = 0.2  
   * b = 0.5  
   */
  public static double joyResponse2parm25(double joyInput) {
    double a = 0.2;
    double b = 0.5;
    if (joyInput < 0) { 
      return ( -a + (1 - a) * (b * Math.pow(joyInput,3) + b * joyInput));
    } else {
      return ( a + (1 - a) * (b * Math.pow(joyInput,3) + b * joyInput));
    }
  }
  
  /**
   * Joystick response curve: 2parm28
   * @return (f(x) = a + (1 - a) * (b*x^3+b*x))
   * a = 0.2  
   * b = 0.8  
   */
  public static double joyResponse2parm28(double joyInput) {
    double a = 0.2;
    double b = 0.8;
    if (joyInput < 0) { 
      return ( -a + (1 - a) * (b * Math.pow(joyInput,3) + b * joyInput));
    } else {
      return ( a + (1 - a) * (b * Math.pow(joyInput,3) + b * joyInput));
    }
  }
  
  

  





  /**
   * Scale is the power of 1
   * Deadband of 0.06
   * @return The scales magnitude vector of the Y axis of RotStick if it breaks deadband
   */

  public double getOrbitSpeed() { //TODO: FIND IF WE NEED
    if(Math.abs(rotStick.getY()) < 0.06) return 0.0;
    return Math.pow(rotStick.getY(), 1);
  }

  /**
   * Scale is the power of 1
   * @return The position of the POV on TransStick (The mini-joystick on top)
   */

  public int getTransStickPOV() {
    return transStick.getPOV();
  }

  /**
   * Scale is the power of 1
   * @return The position of the POV on RotStick (The mini-joystick on top)
   */

  public int getRotStickPOV() {
    return rotStick.getPOV();
  }

  /**
   * Scale is the power of 1
   * @return The value of the y axis of the left joystick of the Xbox Controller
   */

  public double getOp_LeftY() {
    if(Math.abs(operator.getLeftY()) < 0.06) return 0.0;
    return Math.pow(operator.getLeftY(), 1);
  }

  /**
   * Scale is the power of 1
   * @return The value of the x axis of the left joystick of the Xbox Controller
   */

  public double getOp_LeftX() {
    if(Math.abs(operator.getLeftX()) < 0.06) return 0.0;
    return Math.pow(operator.getLeftX(), 1);
  }

  /**
   * Scale is the power of 1
   * @return The value of the y axis of the right joystick of the Xbox Controller
   */

  public double getOp_RightY() {
    if(Math.abs(operator.getRightY()) < 0.06) return 0.0;
    return Math.pow(operator.getLeftY(), 1);
  }

  /**
   * Scale is the power of 1
   * @return The scales value of the x axis of the right joystick of the Xbox Controller
   */

  public double getOp_RightX() {
    if(Math.abs(operator.getRightX()) < 0.06) return 0.0;
    return Math.pow(operator.getRightX(), 1);
  }

  /* Returns the object of the named button */

  public JoystickButton getTransStick_Button1() {return transStick_Button1;}
  public JoystickButton getTransStick_Button2() {return transStick_Button2;}
  public JoystickButton getTransStick_Button3() {return transStick_Button3;}
  public JoystickButton getTransStick_Button4() {return transStick_Button4;}
  public JoystickButton getTransStick_Button5() {return transStick_Button5;}
  public JoystickButton getTransStick_Button6() {return transStick_Button6;}
  public JoystickButton getTransStick_Button7() {return transStick_Button7;}
  public JoystickButton getTransStick_Button8() {return transStick_Button8;}
  public JoystickButton getTransStick_Button9() {return transStick_Button9;}
  public JoystickButton getTransStick_Button10() {return transStick_Button10;}
  public JoystickButton getTransStick_Button11() {return transStick_Button11;}
  public JoystickButton getTransStick_Button12() {return transStick_Button12;}
  public JoystickButton getTransStick_Button13() {return transStick_Button13;}
  public JoystickButton getTransStick_Button14() {return transStick_Button14;}
  public JoystickButton getTransStick_Button15() {return transStick_Button15;}
  public JoystickButton getTransStick_Button16() {return transStick_Button16;}
 
  public POVButton getTransStick_HAT_0() {return transStick_HAT_0;}
  public POVButton getTransStick_HAT_45() {return transStick_HAT_45;}
  public POVButton getTransStick_HAT_90() {return transStick_HAT_90;}
  public POVButton getTransStick_HAT_135() {return transStick_HAT_135;}
  public POVButton getTransStick_HAT_180() {return transStick_HAT_180;}
  public POVButton getTransStick_HAT_225() {return transStick_HAT_225;}
  public POVButton getTransStick_HAT_270() {return transStick_HAT_270;}
  public POVButton getTransStick_HAT_315() {return transStick_HAT_315;}


  
  public JoystickButton getRotStick_Button1() {return rotStick_Button1;}
  public JoystickButton getRotStick_Button2() {return rotStick_Button2;}
  public JoystickButton getRotStick_Button3() {return rotStick_Button3;}
  public JoystickButton getRotStick_Button4() {return rotStick_Button4;}
  public JoystickButton getRotStick_Button5() {return rotStick_Button5;}
  public JoystickButton getRotStick_Button6() {return rotStick_Button6;}
  public JoystickButton getRotStick_Button7() {return rotStick_Button7;}
  public JoystickButton getRotStick_Button8() {return rotStick_Button8;}
  public JoystickButton getRotStick_Button9() {return rotStick_Button9;}
  public JoystickButton getRotStick_Button10() {return rotStick_Button10;}
  public JoystickButton getRotStick_Button11() {return rotStick_Button11;}
  public JoystickButton getRotStick_Button12() {return rotStick_Button12;}
  public JoystickButton getRotStick_Button13() {return rotStick_Button13;}
  public JoystickButton getRotStick_Button14() {return rotStick_Button14;}
  public JoystickButton getRotStick_Button15() {return rotStick_Button15;}
  public JoystickButton getRotStick_Button16() {return rotStick_Button16;}

  public POVButton getRotStick_HAT_0() {return rotStick_HAT_0;}
  public POVButton getRotStick_HAT_45() {return rotStick_HAT_45;}
  public POVButton getRotStick_HAT_90() {return rotStick_HAT_90;}
  public POVButton getRotStick_HAT_135() {return rotStick_HAT_135;}
  public POVButton getRotStick_HAT_180() {return rotStick_HAT_180;}
  public POVButton getRotStick_HAT_225() {return rotStick_HAT_225;}
  public POVButton getRotStick_HAT_270() {return rotStick_HAT_270;}
  public POVButton getRotStick_HAT_315() {return rotStick_HAT_315;}


  // public Trigger getOp_A() {return op_A;}
  // public Trigger getOp_A_FS() {return op_A_Shift;}
  // public Trigger getOp_A_DS() {return op_A_Double_Shift;}
  // public Trigger getOp_B() {return op_B;}
  // public Trigger getOp_B_FS() {return op_B_Shift;}
  // public Trigger getOp_B_DS() {return op_B_Double_Shift;}
  // public Trigger getOp_X() {return op_X;}
  // public Trigger getOp_X_FS() {return op_X_Shift;}
  // public Trigger getOp_X_DS() {return op_X_Double_Shift;}
  // public Trigger getOp_Y() {return op_Y;}
  // public Trigger getOp_Y_FS() {return op_Y_Shift;}
  // public Trigger getOp_Y_DS() {return op_Y_Double_Shift;}
  // public Trigger getOp_Start() {return op_Start;}
  // public Trigger getOp_Start_FS() {return op_Start_Shift;}
  // public Trigger getOp_Start_DS() {return op_Start_Double_Shift;}
  // public Trigger getOp_Back() {return op_Back;}
  // public Trigger getOp_Back_FS() {return op_Back_Shift;}
  // public Trigger getOp_Back_DS() {return op_Back_Double_Shift;}
  // public Trigger getOp_LeftTrigger() {return op_LTrigger;}
  // public Trigger getOp_RightTrigger() {return op_RTrigger;}
  
  // public POVButton getOp_DPAD_UP() {return op_DPAD_Up;}
  // public POVButton getOp_DPAD_RIGHT() {return op_DPAD_Right;}
  // public POVButton getOp_DPAD_DOWN() {return op_DPAD_Down;}
  // public POVButton getOp_DPAD_LEFT() {return op_DPAD_Left;}
}
