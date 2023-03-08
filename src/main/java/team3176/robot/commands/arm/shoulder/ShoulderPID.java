// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.arm.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.ArmConstants;
import team3176.robot.subsystems.arm.Shoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class ShoulderPID extends CommandBase {
  public String mode = "";
  Shoulder m_Andrew = Shoulder.getInstance();
  private double set;
  //private static ExtendArm instance;

  public ShoulderPID() {
    addRequirements(m_Andrew);
  }

  @Override
  public void initialize() {
    ArmConstants.wasLimitSwitchPressed = 0;
    ArmConstants.PidTime = 0;
  }

  @Override
  public void execute() {
    ArmConstants.SHOULDER_POSITION = 0;
    m_Andrew.setMotorPIDWithLimiterBound(ControlMode.Position, ArmConstants.SHOULDER_POSITION); //Andrew is actually really smart
    if (ArmConstants.PidTime == 5) {
      //System.out.println("Shoulder position:  " + m_Andrew.getCurrentPosition());
      ArmConstants.PidTime = 0;
    } else {
      ArmConstants.PidTime++;
    }

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
