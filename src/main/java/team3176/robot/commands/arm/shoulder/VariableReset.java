// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.arm.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.ArmConstants;
import team3176.robot.subsystems.arm.Shoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class VariableReset extends CommandBase {
  public String mode = "";
  Shoulder m_Andrew = Shoulder.getInstance();
  private double set;
  //private static ExtendArm instance;

  public VariableReset() {
    addRequirements(m_Andrew);
  }

  @Override
  public void initialize() {
    //System.out.println("Shoulder position:  " + m_Andrew.getCurrentPosition());
  }

  @Override
  public void execute() {
    ArmConstants.isLimitSwitch = 0;
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
