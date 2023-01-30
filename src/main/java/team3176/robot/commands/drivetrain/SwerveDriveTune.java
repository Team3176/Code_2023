package team3176.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;


public class SwerveDriveTune extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();


  private BooleanSupplier isFieldCentric;
  private BooleanSupplier isRobotCentric;

  public SwerveDriveTune() {
    //this.isFieldCentric = isFieldCentric;
    //this.isRobotCentric = isRobotCentric;
    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
    drivetrain.setSpinLock(false);
  }

  @Override
  public void execute() {

    if(drivetrain.currentCoordType == coordType.FIELD_CENTRIC) {
      //drivetrain.setCoordType(coordType.FIELD_CENTRIC);
      drivetrain.resetFieldOrientation();
    }
    //if(isRobotCentric.getAsBoolean()) {
    //  drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    //}
    drivetrain.drive(0,0,0);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}