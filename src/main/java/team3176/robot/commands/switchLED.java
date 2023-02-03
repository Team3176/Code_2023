// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;
import team3176.robot.subsystems.vision.Vision.LEDState;

public class switchLED extends CommandBase {
  private final Vision m_Vision = Vision.getInstance();
  public LEDState lState;
  public boolean done;
  /** Creates a new switchLED. */
  public switchLED() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_Vision.getLEDState() == 1){
      lState = LEDState.ON;
    }
    else{
      lState = LEDState.OFF;
    }
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Vision.switchLEDs(lState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
