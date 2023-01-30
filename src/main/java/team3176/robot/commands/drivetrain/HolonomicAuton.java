package team3176.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;

import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain.coordType;

public class HolonomicAuton extends CommandBase {

  Drivetrain drivetrain = Drivetrain.getInstance();
  SwerveDriveOdometry odometry = drivetrain.odom;

  HolonomicDriveController holonomicController;
  Trajectory trajectory;

  double startTime;
  
  int stateNumber;
  
  public HolonomicAuton(Trajectory trajectory) {
    holonomicController = new HolonomicDriveController(
      new PIDController(1, 0, 0), // X Controller
      new PIDController(1, 0, 0), // Y Controller
      new ProfiledPIDController(1, 0, 0, // Theta Controller
        new TrapezoidProfile.Constraints(DrivetrainConstants.MAX_ROT_SPEED_RADIANS_PER_SECOND, 2 * Math.PI))); // Constraints are maxVel and maxAccel both in radians

    this.trajectory = trajectory;
    stateNumber = 0;
  }

  @Override
  public void initialize() {
      startTime = Timer.getFPGATimestamp();
  }

  
  @Override
  public void execute() {
    double trajTime = 0.0;
    for(int idx = stateNumber; idx < trajectory.getStates().size(); idx++) {
      if(trajectory.getStates().get(idx).timeSeconds < Timer.getFPGATimestamp() - startTime) {
        trajTime = trajectory.getStates().get(idx - 1).timeSeconds;
        stateNumber = idx - 1;
        break;
      }
    }
       
    Trajectory.State nextState = trajectory.sample(trajTime);
    ChassisSpeeds adjustedSpeeds = holonomicController.calculate(odometry.getPoseMeters(), nextState, nextState.poseMeters.getRotation());

    // Simpily normalizing to get -1 to 1
    double forwardCommand = adjustedSpeeds.vxMetersPerSecond / DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND;
    double strafeCommand = adjustedSpeeds.vyMetersPerSecond / DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND;
    double spinCommand = adjustedSpeeds.omegaRadiansPerSecond / DrivetrainConstants.MAX_ROT_SPEED_RADIANS_PER_SECOND;

    drivetrain.drive(forwardCommand, strafeCommand, spinCommand, coordType.FIELD_CENTRIC);
  }

  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return trajectory.getTotalTimeSeconds() < Timer.getFPGATimestamp() - startTime;
  } 
}