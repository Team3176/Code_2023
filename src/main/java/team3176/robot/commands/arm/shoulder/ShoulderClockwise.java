// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.arm.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.arm.Shoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class ShoulderClockwise extends CommandBase {
  public String mode = "";
  Shoulder m_Andrew = Shoulder.getInstance();
  private double set;
  //private static ExtendArm instance;

  public ShoulderClockwise() {
    addRequirements(m_Andrew);
  }

  @Override
  public void initialize() {
    System.out.println("Shoulder position:  " + m_Andrew.getCurrentPosition());
  }

  @Override
  public void execute() {
    set = -0.3;
    m_Andrew.setMotorCWithLimiterBound(ControlMode.PercentOutput, set); //Andrew is actually really smart
    //System.out.println("Extend Limiter:  " + m_Andrew.getExtendLimiter());
    //System.out.println("Retract Limiter:  " + m_Andrew.getRetractLimiter());
  }


  @Override
  public void end(boolean interrupted) {
    m_Andrew.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

 
}
