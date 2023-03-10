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

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3176.robot.constants.IntakeConstants;
import team3176.robot.subsystems.claw.ClawIO.ClawIOInputs;

public class Claw extends SubsystemBase {
  private DoubleSolenoid piston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  //private DoubleSolenoid piston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.DSOLENOID2_FWD_CHAN, IntakeConstants.DSOLENOID2_REV_CHAN);
  // private TalonSRX intakeMotor = new TalonSRX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
  private CANSparkMax clawMotor = new CANSparkMax(0, MotorType.kBrushless);
  private boolean pistonSetting = false;
  private boolean motorSetting = false;
  private boolean isIdle = false;

  private final ClawIO io;
  private final ClawIOInputs inputs = new ClawIOInputs();
  private static Claw instance;

  private Claw(ClawIO io) {
    this.io = io;
    // clawMotor.setInverted(true);
  }

  public void Open() {
    isIdle = false;
    pistonSetting = true;
    piston1.set(Value.kForward);
    //piston2.set(Value.kForward);
  }

  public boolean isIdle() {
    return this.isIdle;
  }

  public void Close() {
    isIdle = false;
    pistonSetting = false;
    piston1.set(Value.kReverse);
    //piston2.set(Value.kReverse);
  }

  public void Idle()
  {
    isIdle = true;
    pistonSetting = false;
    piston1.set(Value.kOff);
    //piston2.set(Value.kOff);
  }

  public void spinVelocityPercent(double pct) {
    clawMotor.set(pct);
    motorSetting = true;
    if(pct == 0) {motorSetting = false;}
  }

  public void stopMotor() {
    clawMotor.set(0.0);
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



  public void shuffleboardPercentOutput()
  {
    double percent = SmartDashboard.getNumber(IntakeConstants.kShuffleboardPercentName, 0.0);
    clawMotor.set(percent);
    SmartDashboard.putNumber("ClawTics/100ms", clawMotor.get());
    if (percent != 0) {
      motorSetting = true;
    }
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Claw", inputs);
    Logger.getInstance().recordOutput("Claw/Velocity", getClawVelocity());
    Logger.getInstance().recordOutput("Claw/isOpen", getClawOpen());
    Logger.getInstance().recordOutput("Claw/isIdle", getClawIdle());
    Logger.getInstance().recordOutput("Claw/isClose", getClawClose());

  }

  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setVelocity(double intakeVelocity) {
    io.setVelocity(intakeVelocity);
  }

  public void setClawState(boolean isOpen, boolean isClose, boolean isIdle) {
    io.setClawState(isOpen, isClose, isIdle);
  }

  public double getClawVelocity() {
    return inputs.velocity;
  }

  public boolean getClawOpen() {
    return inputs.isOpen;
  }

  public boolean getClawIdle() {
    return inputs.isIdle;
  }

  public boolean getClawClose() {
    return inputs.isClose;
  }

  @Override
  public void simulationPeriodic() {}
}