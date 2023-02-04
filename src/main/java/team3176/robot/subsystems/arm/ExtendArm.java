// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class ExtendArm extends CommandBase {
  public String mode = "";
  JointTalonFX m_Andrew = JointTalonFX.getInstance();
  private int set;
  private static ExtendArm instance;

  public ExtendArm() {
    addRequirements(m_Andrew);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_Andrew.setMotorWithLimiterBound(ControlMode.Position, 100);
  }


  @Override
  public void end(boolean interrupted) {
    m_Andrew.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static ExtendArm getInstance() {
    if(instance == null) {instance = new ExtendArm();}
    return instance;
  }
}
