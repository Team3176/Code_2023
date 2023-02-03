// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.intake.Intake;

public class IntakeSpin extends CommandBase {
  private Intake m_Intake = Intake.getInstance();
  /** Creates a new intakeSpin. */
  public IntakeSpin() {
    addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  m_Intake.spinVelocityPercent(.30);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
