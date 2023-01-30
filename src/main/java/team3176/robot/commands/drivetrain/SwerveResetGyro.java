package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.drivetrain.Drivetrain;

/**
 * Makes the gyro's "zero point" its current position, for recallibration.
 */
public class SwerveResetGyro extends InstantCommand {
  public SwerveResetGyro() {
    super(()->Drivetrain.getInstance().resetFieldOrientation());
  }
}

