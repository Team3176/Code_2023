package team3176.robot.commands.drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;


public class SwervePodsAzimuthGoHome extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  public SwervePodsAzimuthGoHome()  {
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
  }

  @Override
  public void execute() {
    drivetrain.sendPodsAzimuthToHome();
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setCurrentPodPosAsHome();
  }
}