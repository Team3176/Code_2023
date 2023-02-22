// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.arm.elbow;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.ArmConstants;
import team3176.robot.subsystems.arm.Elbow;
import team3176.robot.subsystems.arm.Elbow.ElbowState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class ElbowPickup extends CommandBase {
  public String mode = "";
  Elbow m_Elbow = Elbow.getInstance();
  private int set;
  //private static ExtendArm instance;

  public ElbowPickup() {
    addRequirements(m_Elbow);
  }

  @Override
  public void initialize() {
    System.out.println("Elbow position:  " + m_Elbow.getCurrentPosition());
  }

  @Override
  public void execute() {
    m_Elbow.setElbowState(ElbowState.PICKUP);
    
			m_Elbow.setPosition(ArmConstants.ELBOW_PICKUP_POSITION_DEG);

  }


  @Override
  public void end(boolean interrupted) {
   // m_Elbow.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

 
}
