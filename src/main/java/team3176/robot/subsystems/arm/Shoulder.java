// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.ArmConstants;
//import team3176.robot.subsystems.FlywheelIO.FlywheelIOInputs;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoulder extends SubsystemBase {

  private TalonFX jointMotor;

 // private final FlywheelIO io;
 // private final FlywheelIOInputs inputs = new FlywheelIOInputs();
  private static Shoulder instance;
  private boolean isSmartDashboardTestControlsShown;
  public String mode = "";
  private DigitalInput extendLimiter;
  private DigitalInput retractLimiter;
  private int intent;
  
  public Shoulder()
  {
    //this.io = io;
    System.out.println("Shoulder has been constructed");
    jointMotor = new TalonFX(ArmConstants.SHOULDER_FALCON_CAN_ID);
    extendLimiter = new DigitalInput(ArmConstants.SHOULDER_EXTENDED_LIMIT_CHAN);
    retractLimiter = new DigitalInput(ArmConstants.SHOULDER_RETRACTED_LIMIT_CHAN);
    this.intent = 0;

    jointMotor.configFactoryDefault();
    jointMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    jointMotor.configAllowableClosedloopError(0, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    jointMotor.setSensorPhase(true);
    jointMotor.configClosedloopRamp(ArmConstants.kRampRate, ArmConstants.kTimeoutMS);
    
    jointMotor.config_kP(0, ArmConstants.Shoulder_PIDFConstants[0][0]);
    jointMotor.config_kI(0, ArmConstants.Shoulder_PIDFConstants[0][1]);
    jointMotor.config_kD(0, ArmConstants.Shoulder_PIDFConstants[0][2]);
    jointMotor.config_kF(0, ArmConstants.Shoulder_PIDFConstants[0][3]);
    jointMotor.config_IntegralZone(0, ArmConstants.Shoulder_PIDFConstants[0][4]);

    jointMotor.config_kP(0, ArmConstants.Shoulder_PIDFConstants[0][0]);
    jointMotor.config_kI(0, ArmConstants.Shoulder_PIDFConstants[0][1]);
    jointMotor.config_kD(0, ArmConstants.Shoulder_PIDFConstants[0][2]);
    jointMotor.config_kF(0, ArmConstants.Shoulder_PIDFConstants[0][3]);
    jointMotor.config_IntegralZone(0, ArmConstants.Shoulder_PIDFConstants[0][4]);
  }

  public void spinMotors(double ticksPer100ms) 
  {
    jointMotor.set(TalonFXControlMode.Velocity, ticksPer100ms);
  }

  public void spinMotors(double ticksPer100msForMotor1, double ticksPer100msForMotor2) 
  {
    jointMotor.set(TalonFXControlMode.Velocity, ticksPer100msForMotor1);
  }

  /*
  public void spinMotors2(double metersPerSecond)
  {
    // double ticsPer100ms = --MATH!-- (will need radius of flywheel for v = r(omega))
    flywheelMotor1.set(TalonFXControlMode.Velocity, ticksPer100ms);
  }
  */

  public void percentOutput_1() 
  {
    //double output = SmartDashboard.getNumber(ArmConstants.kShuffleboardPercentName1, 0.0);
    //if (output >= -1 && output <= 1) { flywheelMotor1.set(ControlMode.PercentOutput, output); }
    SmartDashboard.putNumber("Fly1Tics/100msOut", jointMotor.getSelectedSensorVelocity());
  }

  public void stopMotors()
  {
    jointMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

    public void putSmartDashboardControlCommands() 
    {
    SmartDashboard.putNumber("Arm 1 PCT", 0);
    isSmartDashboardTestControlsShown = true;
    }

    public void setValuesFromSmartDashboard() 
    {
      jointMotor.set(TalonFXControlMode.PercentOutput, SmartDashboard.getNumber("Arm 1 PCT", 0));
    }

  // Called by the engageMotor methods in this class. The engageMotor methods are accessed from outside commands, and that
  // method accesses this one to actually set the motor
  public void setMotorPosWithLimiterBound(ControlMode mode, double set)
  {
    if (retractLimiter.get() && extendLimiter.get()) {
      jointMotor.set(mode, set);
    } else if (!retractLimiter.get() && (this.intent == -1 || intent == 0) && extendLimiter.get()) {
      jointMotor.set(mode, set);
    } else if (!extendLimiter.get() && (this.intent == 1 || this.intent == 0) && retractLimiter.get()) {
      jointMotor.set(mode, set);
    } else {
      jointMotor.set(mode, set);
    }
  }
  public void setMotorSpdWithLimiterBound(ControlMode mode, double set)
  {
    if (retractLimiter.get() && extendLimiter.get()) {
      jointMotor.set(ControlMode.PercentOutput, set);
    } else if (!retractLimiter.get() && (this.intent == -1 || intent == 0) && extendLimiter.get()) {
      jointMotor.set(ControlMode.PercentOutput, set);
    } else if (!extendLimiter.get() && (this.intent == 1 || this.intent == 0) && retractLimiter.get()) {
      jointMotor.set(ControlMode.PercentOutput, set);
    } else {
      jointMotor.set(ControlMode.PercentOutput, set);
    }
  }
  /* 
  public void runVoltage(double volts) 
  {
    io.setVoltage(volts);
  }

  public double getArmVelocity1()
  {
    return inputs.velocity_1;
  }

  public void setArmVelocity1(double velocity)
  {
    io.setArmVelocity1(velocity);
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
  */
  public double getCurrentPosition(){
    return jointMotor.getSelectedSensorPosition();
  }

  public boolean getExtendLimiter(){
    return extendLimiter.get();
  }

  public boolean getRetractLimiter(){
    return retractLimiter.get();
  }
  public static Shoulder getInstance() {
    if(instance == null) {instance = new Shoulder();}
    return instance;
  }
}

