package team3176.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;


public class SwerveDrive extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();


  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;
  private DoubleSupplier spinCommand;

  public SwerveDrive( DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
    drivetrain.setSpinLock(false);
  }

  @Override
  public void execute() {
    drivetrain.drive(forwardCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND *0.3, 
    strafeCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND *0.3, 
    spinCommand.getAsDouble()*100);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}