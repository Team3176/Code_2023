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
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JointTalonFX {

  private TalonFX jointMotor;

 // private final FlywheelIO io;
 // private final FlywheelIOInputs inputs = new FlywheelIOInputs();
  private static JointTalonFX instance;
  private boolean isSmartDashboardTestControlsShown;
  public String mode = "";

  public JointTalonFX(int can_id)
  {
    //this.io = io;

    jointMotor = new TalonFX(can_id);


    jointMotor.configFactoryDefault();
    jointMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    jointMotor.configAllowableClosedloopError(0, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    jointMotor.setSensorPhase(true);
    jointMotor.configClosedloopRamp(ArmConstants.kRampRate, ArmConstants.kTimeoutMS);
    
    jointMotor.config_kP(0, ArmConstants.PIDFConstants[0][0]);
    jointMotor.config_kI(0, ArmConstants.PIDFConstants[0][1]);
    jointMotor.config_kD(0, ArmConstants.PIDFConstants[0][2]);
    jointMotor.config_kF(0, ArmConstants.PIDFConstants[0][3]);
    jointMotor.config_IntegralZone(0, ArmConstants.PIDFConstants[0][4]);

    jointMotor.config_kP(0, ArmConstants.PIDFConstants[0][0]);
    jointMotor.config_kI(0, ArmConstants.PIDFConstants[0][1]);
    jointMotor.config_kD(0, ArmConstants.PIDFConstants[0][2]);
    jointMotor.config_kF(0, ArmConstants.PIDFConstants[0][3]);
    jointMotor.config_IntegralZone(0, ArmConstants.PIDFConstants[0][4]);
  }

  public void spinMotors(double ticksPer100ms) {
    jointMotor.set(TalonFXControlMode.Velocity, ticksPer100ms);
  }

  public void spinMotors(double ticksPer100msForMotor1, double ticksPer100msForMotor2) {
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
    double output = SmartDashboard.getNumber(ArmConstants.kShuffleboardPercentName1, 0.0);
    //if (output >= -1 && output <= 1) { flywheelMotor1.set(ControlMode.PercentOutput, output); }
    SmartDashboard.putNumber("Fly1Tics/100msOut", jointMotor.getSelectedSensorVelocity());
  }

  public void stopMotors()
  {
    jointMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

    public void putSmartDashboardControlCommands() {
    SmartDashboard.putNumber("Arm 1 PCT", 0);
    isSmartDashboardTestControlsShown = true;
    }

    public void setValuesFromSmartDashboard() 
    {
      jointMotor.set(TalonFXControlMode.PercentOutput, SmartDashboard.getNumber("Arm 1 PCT", 0));
    }

  /* 
  public void runVoltage(double volts) {
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
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  */

  /* 
  public static JointTalonFX getInstance() {
    if(instance == null) {instance = new Arm(new ArmIO() {});}
    return instance;
  }
  */
  
}

