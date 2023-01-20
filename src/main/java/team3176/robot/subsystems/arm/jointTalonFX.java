// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.ArmConstants;
import team3176.robot.subsystems.FlywheelIO.FlywheelIOInputs;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class jointTalonFX extends SubsystemBase {

  private TalonFX flywheelMotor1;

  private final FlywheelIO io;
  private final FlywheelIOInputs inputs = new FlywheelIOInputs();
  private static Flywheel instance;
  private boolean isSmartDashboardTestControlsShown;
  public String mode = "";

  public Flywheel(FlywheelIO io)
  {
    this.io = io;

    flywheelMotor1 = new TalonFX(FlywheelConstants.FLYWHEEL_FALCON1_CAN_ID);


    ArmMotor1.configFactoryDefault();
    ArmMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    ArmMotor1.configAllowableClosedloopError(0, ArmConstants.kPIDLoopIndex, ArmConstants.kTimeoutMS);
    ArmMotor1.setSensorPhase(true);
    ArmMotor1.configClosedloopRamp(ArmConstants.kRampRate, ArmConstants.kTimeoutMS);
    
    ArmMotor1.config_kP(0, ArmConstants.PIDFConstants[0][0]);
    ArmMotor1.config_kI(0, ArmConstants.PIDFConstants[0][1]);
    ArmMotor1.config_kD(0, ArmConstants.PIDFConstants[0][2]);
    ArmMotor1.config_kF(0, ArmConstants.PIDFConstants[0][3]);
    ArmMotor1.config_IntegralZone(0, ArmConstants.PIDFConstants[0][4]);

    ArmMotor1.config_kP(0, ArmConstants.PIDFConstants[0][0]);
    ArmMotor1.config_kI(0, ArmConstants.PIDFConstants[0][1]);
    ArmMotor1.config_kD(0, ArmConstants.PIDFConstants[0][2]);
    ArmMotor1.config_kF(0, ArmConstants.PIDFConstants[0][3]);
    ArmMotor1.config_IntegralZone(0, ArmConstants.PIDFConstants[0][4]);
  }

  public void spinMotors(double ticksPer100ms) {
    ArmMotor1.set(TalonFXControlMode.Velocity, ticksPer100ms);
  }

  public void spinMotors(double ticksPer100msForMotor1, double ticksPer100msForMotor2) {
    ArmMotor1.set(TalonFXControlMode.Velocity, ticksPer100msForMotor1);
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
    SmartDashboard.putNumber("Fly1Tics/100msOut", ArmMotor1.getSelectedSensorVelocity());
  }

  public void stopMotors()
  {
    ArmMotor1.set(TalonFXControlMode.PercentOutput, 0.0);
  }

    public void putSmartDashboardControlCommands() {
    SmartDashboard.putNumber("Arm 1 PCT", 0);
    isSmartDashboardTestControlsShown = true;
    }

    public void setValuesFromSmartDashboard() 
    {
      ArmMotor1.set(TalonFXControlMode.PercentOutput, SmartDashboard.getNumber("Arm 1 PCT", 0));
    }

  @Override
  public void periodic() {
    Logger.getInstance().processInputs("Arm", inputs);
    Logger.getInstance().recordOutput("Arm/Velocity 1", getArmVelocity1());

    if(mode.equals("test"))
    {
      if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
      setValuesFromSmartDashboard();
    }
  }

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

  public static Arm getInstance() {
    if(instance == null) {instance = new Arm(new ArmIO() {});}
    return instance;
  }
  
}
