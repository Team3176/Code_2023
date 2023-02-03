// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.claw.Claw;

public class ClawClose extends CommandBase {
  /** Creates a new ClawClose. */
  private Claw m_Claw = Claw.getInstance();
  public ClawClose() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (m_Claw.isIdle() == false)
    {
      m_Claw.Close();
      m_Claw.spinVelocityPercent(0);
    }
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
