// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3176.robot.constants.IntakeConstants;
import team3176.robot.subsystems.claw.ClawIO.ClawIOInputs;

public class Claw extends SubsystemBase {
  private DoubleSolenoid piston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.DSOLENOID1_FWD_CHAN, IntakeConstants.DSOLENOID1_REV_CHAN);
//  private DoubleSolenoid piston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.DSOLENOID2_FWD_CHAN, IntakeConstants.DSOLENOID2_REV_CHAN);
  private TalonSRX intakeMotor = new TalonSRX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
  private boolean pistonSetting = false;
  private boolean motorSetting = false;
  private DigitalInput ballSensor;
  private boolean isIntaking = false;
  public boolean isExtended;

  private final ClawIO io;
  private final ClawIOInputs inputs = new ClawIOInputs();
  private static Claw instance;

  private Claw(ClawIO io) {
    this.io = io;
    intakeMotor.setInverted(true);
    ballSensor = new DigitalInput(IntakeConstants.BALL_SENSOR_DIO);
  }

  public void Extend() {
    isIntaking = true;
    pistonSetting = true;
    piston1.set(Value.kForward);
    // piston2.set(Value.kForward);
  }

  public boolean isExtended() {
    return this.isIntaking;
  }

  public void Retract() {
    isIntaking = false;
    pistonSetting = false;
    piston1.set(Value.kReverse);
    // piston2.set(Value.kReverse);
  }

  public void spinVelocityPercent(double pct) {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, pct);
    motorSetting = true;
    if(pct == 0) {motorSetting = false;}
  }

  public void stopMotor() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    motorSetting = false;
  }

  public boolean getPistonSetting() {
    return pistonSetting;
  }

  public boolean getMotorSetting() {
    return motorSetting;
  }

  public static Claw getInstance() {
    if(instance == null) {instance = new Claw(new ClawIO() {});}
    return instance;
  }

  public boolean getLine() {return ballSensor.get();}



  public void shuffleboardPercentOutput()
  {
    double percent = SmartDashboard.getNumber(IntakeConstants.kShuffleboardPercentName, 0.0);
    intakeMotor.set(ControlMode.PercentOutput, percent);
    SmartDashboard.putNumber("IntakeTics/100ms", intakeMotor.getSelectedSensorVelocity());
    if (percent != 0) {
      motorSetting = true;
    }
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
    Logger.getInstance().recordOutput("Intake/Velocity", getIntakeVelocity());
    Logger.getInstance().recordOutput("Intake/PistonState", getPistonState());

  }

  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setVelocity(double intakeVelocity) {
    io.setVelocity(intakeVelocity);
  }

  public void setPiston(boolean isExtend) {
    io.setPiston(isExtend);
  }

  public double getIntakeVelocity() {
    return inputs.velocity;
  }

  public boolean getPistonState() {
    return inputs.isExtend;
  }

  @Override
  public void simulationPeriodic() {}
}