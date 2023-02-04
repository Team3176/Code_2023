package team3176.robot.commands.drivetrain;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class PathPlannerAuto {
    Command auto;
    public PathPlannerAuto() {

    Drivetrain driveSubsystem = Drivetrain.getInstance();
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("auto1", new PathConstraints(1.5, 0.5));

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    // eventMap.put("intakeDown", new IntakeDown());

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        driveSubsystem::getPose, // Pose2d supplier
        driveSubsystem::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        DrivetrainConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );

    auto = autoBuilder.fullAuto(pathGroup);
    }
    public Command getauto(){
        return auto;
    }

}
