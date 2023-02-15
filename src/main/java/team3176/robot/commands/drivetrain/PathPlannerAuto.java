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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class PathPlannerAuto {
    Command auto;
    public PathPlannerAuto() {

        Drivetrain driveSubsystem = Drivetrain.getInstance();
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("auto1", new PathConstraints(1.5, 1.5));
        System.out.println("length" + pathGroup.size());
        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // eventMap.put("intakeDown", new IntakeDown());
        driveSubsystem.resetPose(pathGroup.get(0).getInitialHolonomicPose());
        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveSubsystem::getPose,
            driveSubsystem::resetPose,
            DrivetrainConstants.DRIVE_KINEMATICS,
            new PIDConstants(5.0,0.0,0.0),
            new PIDConstants(0.5,0.0,0.0),
            driveSubsystem::setModuleStates,
            eventMap,
            true,
            driveSubsystem);


        auto = autoBuilder.followPath(pathGroup.get(0));
    }
    public Command getauto(){
        return auto;
    }

}
