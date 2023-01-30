
package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;

public class SwerveDefense extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  public SwerveDefense() {
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DEFENSE);
  }

  @Override
  public void execute() {
    drivetrain.drive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) { 
    drivetrain.setDriveMode(driveMode.DRIVE);
   }
}